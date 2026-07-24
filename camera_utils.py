"""
相机工具模块 —— A4 纸检测、透视变换、焦距标定
所有视觉测量模块的共享基础设施
"""

import cv2
import numpy as np
from maix import image

import config


# ============================================================
# 距离滤波器
# ============================================================
class DistanceFilter:
    """滑动窗口均值滤波器，用于平滑距离测量值"""

    def __init__(self, window_size=None):
        self._history = []
        self._size = window_size or config.DISTANCE_FILTER_SIZE

    def update(self, value: float) -> float:
        """添加新值，返回滤波后的结果"""
        self._history.append(value)
        if len(self._history) > self._size:
            self._history.pop(0)
        return self.value

    @property
    def value(self) -> float:
        """当前滤波值"""
        if not self._history:
            return 0.0
        return sum(self._history) / len(self._history)

    def reset(self):
        self._history.clear()


# ============================================================
# Maix 图像 ↔ OpenCV 图像转换
# ============================================================
def maix_to_cv(maix_img):
    """Maix 图像 → OpenCV numpy 数组 (BGR)"""
    return image.image2cv(maix_img)


def cv_to_maix(cv_img):
    """OpenCV numpy 数组 → Maix 图像"""
    return image.cv2image(cv_img)


# ============================================================
# A4 纸轮廓检测 —— 自适应阈值法（通用场景）
# ============================================================
def detect_a4_adaptive(cv_img):
    """
    使用自适应阈值检测 A4 纸黑色边框轮廓

    算法流程:
        灰度化 → 自适应高斯阈值 → 形态学开运算 →
        轮廓查找 → 筛选最大凸四边形

    Args:
        cv_img: OpenCV BGR 图像

    Returns:
        (contour, thresh_image) — 轮廓为 (N,1,2) numpy 数组或 None
    """
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

    # 自适应阈值：对光照变化鲁棒
    thresh = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        config.ADAPTIVE_THRESH_BLOCK,
        config.ADAPTIVE_THRESH_C,
    )
    thresh_copy = thresh.copy()

    # 形态学去噪
    kernel = np.ones(config.A4_MORPH_KERNEL, np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    # 查找外轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    best_approx = None

    for cnt in contours:
        if len(cnt) < 4:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, config.A4_POLY_EPSILON_SCALE * peri, True)

        # 必须为凸四边形
        if len(approx) == 4 and cv2.isContourConvex(approx):
            area = cv2.contourArea(approx)
            img_area = cv_img.shape[0] * cv_img.shape[1]
            area_ratio = area / img_area

            # 面积比例过滤
            if config.A4_AREA_RATIO_MIN < area_ratio < config.A4_AREA_RATIO_MAX:
                if area > max_area:
                    max_area = area
                    best_approx = approx

    if best_approx is not None:
        return best_approx, thresh_copy
    return None, None


# ============================================================
# A4 纸轮廓检测 —— HSV 黑色掩码法（旋转场景）
# ============================================================
# 帧间继承变量
_prev_a4_contour = None

def detect_a4_hsv(cv_img):
    """
    使用 HSV 色彩空间检测 A4 纸黑色边框（旋转场景专用）

    特点:
        - 对梯形畸变有更高容忍度（poly epsilon = 0.15）
        - 帧间轮廓继承（防止短暂丢失）
        - 更强的形态学处理（膨胀×2 + 腐蚀）

    Args:
        cv_img: OpenCV BGR 图像

    Returns:
        (contour, mask) — 轮廓为 (N,1,2) numpy 数组或 None
    """
    global _prev_a4_contour

    hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, config.HSV_BLACK_LOWER, config.HSV_BLACK_UPPER)

    # 更激进的形态学处理，连接可能断裂的黑边
    kernel = np.ones(config.SPIN_KERNEL, np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=config.SPIN_DILATE_ITER)
    mask = cv2.erode(mask, kernel, iterations=config.SPIN_ERODE_ITER)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_area = 0
    best_approx = None

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < config.A4_MIN_CONTOUR_AREA:
            continue

        peri = cv2.arcLength(cnt, True)
        # 旋转场景使用更大容差，容忍梯形畸变
        approx = cv2.approxPolyDP(cnt, config.SPIN_POLY_EPSILON_SCALE * peri, True)

        if len(approx) == 4 and cv2.isContourConvex(approx):
            img_area = cv_img.shape[0] * cv_img.shape[1]
            area_ratio = area / img_area
            if config.A4_AREA_RATIO_MIN < area_ratio < config.A4_AREA_RATIO_MAX:
                if area > max_area:
                    max_area = area
                    best_approx = approx

    # 帧间继承：当前帧未检测到时，回退到上一帧
    if best_approx is None and _prev_a4_contour is not None:
        prev_mask = np.zeros_like(mask)
        cv2.drawContours(prev_mask, [_prev_a4_contour], -1, 255, -1)
        overlap = cv2.bitwise_and(mask, prev_mask)
        prev_area = cv2.contourArea(_prev_a4_contour)
        if prev_area > 0:
            overlap_ratio = cv2.countNonZero(overlap) / prev_area
            if overlap_ratio > config.SPIN_CONTOUR_STABILITY:
                best_approx = _prev_a4_contour

    if best_approx is not None:
        _prev_a4_contour = best_approx

    return best_approx, mask


def reset_a4_history():
    """重置 A4 轮廓帧间历史"""
    global _prev_a4_contour
    _prev_a4_contour = None


# ============================================================
# 点集排序
# ============================================================
def order_points(pts):
    """
    对四个角点排序：左上 → 右上 → 右下 → 左下

    算法：sum(x,y) 最小 = 左上，sum(x,y) 最大 = 右下；
          diff(y,x) 最小 = 右上，diff(y,x) 最大 = 左下

    Args:
        pts: (4, 2) numpy 数组

    Returns:
        (4, 2) float32 数组
    """
    pts = pts.astype(np.float32)
    rect = np.zeros((4, 2), dtype="float32")

    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]   # 左上
    rect[2] = pts[np.argmax(s)]   # 右下

    diff = np.diff(pts, axis=1).flatten()
    rect[1] = pts[np.argmin(diff)]  # 右上
    rect[3] = pts[np.argmax(diff)]  # 左下

    return rect


def sort_points_by_center(pts):
    """
    按点与中心的相对位置排序（备用方法）
    适合轮廓形状较规则的情况
    """
    center = np.mean(pts, axis=0)
    rect = np.zeros((4, 2), dtype="float32")
    for point in pts:
        if point[0] < center[0] and point[1] < center[1]:
            rect[0] = point  # 左上
        elif point[0] > center[0] and point[1] < center[1]:
            rect[1] = point  # 右上
        elif point[0] > center[0] and point[1] > center[1]:
            rect[2] = point  # 右下
        else:
            rect[3] = point  # 左下
    return rect


# ============================================================
# 透视变换
# ============================================================
def perspective_transform(cv_img, contour, target_w=None, target_h=None):
    """
    将 A4 纸区域透视校正为矩形

    Args:
        cv_img: 输入图像
        contour: A4 纸四顶点轮廓
        target_w: 目标宽度（像素），None 则用检测到的宽
        target_h: 目标高度（像素），None 则用检测到的高

    Returns:
        (warped, M) — 校正后图像和 3×3 变换矩阵
    """
    # 提取并排序点
    pts = contour.reshape(4, 2)
    rect = order_points(pts)

    # 计算目标尺寸
    if target_w is None or target_h is None:
        width_top = np.linalg.norm(rect[0] - rect[1])
        width_bot = np.linalg.norm(rect[2] - rect[3])
        target_w = int(max(width_top, width_bot))

        height_left = np.linalg.norm(rect[0] - rect[3])
        height_right = np.linalg.norm(rect[1] - rect[2])
        target_h = int(max(height_left, height_right))

    dst = np.array(
        [[0, 0], [target_w - 1, 0], [target_w - 1, target_h - 1], [0, target_h - 1]],
        dtype="float32",
    )

    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(cv_img, M, (target_w, target_h))

    return warped, M


def perspective_transform_mm(contour, a4_short_mm=None, a4_long_mm=None):
    """
    透视变换到毫米坐标空间（用于直接物理测量）

    将 A4 纸区域校正到以毫米为单位的固定尺寸矩形。
    在校正后的图像上，1 像素 ≈ 1mm。

    Args:
        contour: A4 纸四顶点轮廓
        a4_short_mm: A4 短边 mm 数
        a4_long_mm: A4 长边 mm 数

    Returns:
        (M, target_width_mm, target_height_mm)
    """
    short_mm = a4_short_mm or config.A4_SHORT_SIDE_MM
    long_mm = a4_long_mm or config.A4_LONG_SIDE_MM

    pts = contour.reshape(4, 2)
    rect = order_points(pts)

    dst = np.array(
        [[0, 0], [short_mm - 1, 0], [short_mm - 1, long_mm - 1], [0, long_mm - 1]],
        dtype="float32",
    )
    M = cv2.getPerspectiveTransform(rect, dst)

    return M, int(short_mm), int(long_mm)


def inverse_map_contour(contour, M):
    """
    将校正图像上的轮廓映射回原始图像坐标

    Args:
        contour: 校正图像中的轮廓 (N,1,2)
        M: 3×3 透视变换矩阵（正方向）

    Returns:
        原始图像中的轮廓 (N,1,2)
    """
    M_inv = cv2.invert(M)[1]
    pts = contour.reshape(-1, 2).astype(np.float32)
    mapped = cv2.perspectiveTransform(pts[None, :, :], M_inv)[0]
    return mapped.astype(np.int32).reshape(-1, 1, 2)


# ============================================================
# 焦距标定
# ============================================================
def calibrate_focal_length(
    cv_img,
    known_distance_mm=None,
    known_side_mm=None,
    detector=None,
):
    """
    标定相机焦距

    将 A4 纸放在已知距离处（默认 1000mm），系统自动计算焦距。

    公式: F = (pixel_size × known_distance) / known_real_size

    Args:
        cv_img: 当前帧
        known_distance_mm: 已知距离（mm）
        known_side_mm: 用于计算的 A4 边实际长度（mm）
        detector: A4 检测函数，默认 detect_a4_adaptive

    Returns:
        focal_length（像素单位），失败返回 None
    """
    dist = known_distance_mm or config.CALIBRATION_DISTANCE_MM
    side = known_side_mm or config.CALIBRATION_A4_SIDE_MM
    detect_fn = detector or detect_a4_adaptive

    contour, _ = detect_fn(cv_img)
    if contour is None:
        return None

    # 使用最小外接矩形的长边计算像素宽度
    rect = cv2.minAreaRect(contour)
    pixel_width = max(rect[1])

    if pixel_width <= 10:
        return None

    focal = (pixel_width * dist) / side
    return round(focal, 3)


# ============================================================
# A4 纸像素尺寸与距离计算
# ============================================================
def get_a4_pixel_size(contour):
    """
    获取 A4 纸在图像中的像素尺寸

    Returns:
        (long_side_px, short_side_px) — 长边和短边的像素长度
    """
    rect = cv2.minAreaRect(contour)
    w, h = rect[1]
    return max(w, h), min(w, h)


def calculate_distance(focal_length_mm, a4_pixel_long, a4_real_mm=None):
    """
    针孔模型计算摄像头到 A4 纸的距离

    公式: D = (F × W_real) / W_pixel

    Args:
        focal_length_mm: 标定好的焦距（像素单位）
        a4_pixel_long: A4 纸长边在当前图像中的像素长度
        a4_real_mm: A4 纸长边实际长度

    Returns:
        距离（mm）
    """
    real_mm = a4_real_mm or config.A4_LONG_SIDE_MM
    if a4_pixel_long <= 0 or focal_length_mm <= 0:
        return 0.0
    return (focal_length_mm * real_mm) / a4_pixel_long
