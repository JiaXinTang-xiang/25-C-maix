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
    dbg_total = len(contours)
    dbg_passed_quad = 0
    dbg_passed_ratio = 0
    dbg_area_hint = 0  # 最大四边形面积

    for cnt in contours:
        if len(cnt) < 4:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, config.A4_POLY_EPSILON_SCALE * peri, True)

        # 必须为凸四边形
        if len(approx) == 4 and cv2.isContourConvex(approx):
            dbg_passed_quad += 1
            area = cv2.contourArea(approx)
            if area > dbg_area_hint:
                dbg_area_hint = area
            img_area = cv_img.shape[0] * cv_img.shape[1]
            area_ratio = area / img_area

            # 面积比例过滤
            if config.A4_AREA_RATIO_MIN < area_ratio < config.A4_AREA_RATIO_MAX:
                dbg_passed_ratio += 1
                if area > max_area:
                    max_area = area
                    best_approx = approx

    if best_approx is None and dbg_passed_quad > 0:
        # 找到了四边形但没通过面积比——打印提示
        ratio_pct = round(dbg_area_hint / (cv_img.shape[0] * cv_img.shape[1]) * 100, 1)
        print(f"[A4] {dbg_passed_quad} quads found, "
              f"max area={dbg_area_hint:.0f}px ({ratio_pct}% of image), "
              f"need {config.A4_AREA_RATIO_MIN*100:.0f}%~{config.A4_AREA_RATIO_MAX*100:.0f}%")
        print(f"    total contours={dbg_total}, epsilon_scale={config.A4_POLY_EPSILON_SCALE}, "
              f"min_area={config.A4_MIN_CONTOUR_AREA}")

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
"""
基础形状检测模块 —— 正方形、等边三角形、圆形
用于基本要求部分（基础题）
"""

# ============================================================
# 通用工具
# ============================================================
def _angle_at(p1, p2, p3):
    """计算三点夹角（p2 为顶点），返回角度（度）"""
    v1 = p1 - p2
    v2 = p3 - p2
    dot = np.dot(v1, v2)
    norm = np.linalg.norm(v1) * np.linalg.norm(v2)
    if norm == 0:
        return 0.0
    cos_val = max(-1.0, min(1.0, dot / norm))
    return np.degrees(np.arccos(cos_val))


def _preprocess_warped(warped):
    """
    预处理校正后的 A4 区域图像用于形状检测
    反色 + Otsu 自适应二值化
    """
    if len(warped.shape) == 3:
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    else:
        gray = warped
    inverted = cv2.bitwise_not(gray)
    _, binary = cv2.threshold(inverted, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return binary


# ============================================================
# 正方形检测
# ============================================================
def detect_square(warped):
    """
    在校正后的 A4 区域内检测正方形

    Returns:
        (side_length_px, contour) 或 (None, None)
    """
    binary = _preprocess_warped(warped)
    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < config.SHAPE_MIN_AREA:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) != 4:
            continue

        # 边长一致性检查
        sides = [
            np.linalg.norm(approx[i][0] - approx[(i + 1) % 4][0])
            for i in range(4)
        ]
        max_s, min_s = max(sides), min(sides)
        ratio = abs(max_s - min_s) / ((max_s + min_s) / 2)
        if ratio > config.SQUARE_ASPECT_MAX:
            continue

        # 角度检查
        angles_ok = True
        for i in range(4):
            angle = _angle_at(
                approx[i][0],
                approx[(i + 1) % 4][0],
                approx[(i + 2) % 4][0],
            )
            if not (config.SQUARE_ANGLE_MIN <= angle <= config.SQUARE_ANGLE_MAX):
                angles_ok = False
                break
        if not angles_ok:
            continue

        # 取两种边长算法的均值
        avg_side = np.mean(sides)
        rect_w, rect_h = cv2.minAreaRect(cnt)[1]
        min_rect_side = min(rect_w, rect_h)
        side_px = (avg_side + min_rect_side) / 2

        return side_px, approx

    return None, None


# ============================================================
# 等边三角形检测
# ============================================================
def detect_triangle(warped):
    """
    在校正后的 A4 区域内检测等边三角形

    使用三法加权融合计算边长:
        - 直接测边 (0.6)
        - 外接圆半径法 (0.3)
        - 面积法 (0.1)

    Returns:
        (side_length_px, contour) 或 (None, None)
    """
    binary = _preprocess_warped(warped)
    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < config.SHAPE_MIN_AREA:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
        if len(approx) != 3:
            continue

        # 边长一致性
        points = approx.reshape(3, 2)
        sides = [
            np.linalg.norm(points[1] - points[0]),
            np.linalg.norm(points[2] - points[1]),
            np.linalg.norm(points[0] - points[2]),
        ]
        max_s, min_s = max(sides), min(sides)
        ratio = abs(max_s - min_s) / ((max_s + min_s) / 2)
        if ratio > config.TRIANGLE_ASPECT_MAX:
            continue

        # 角度检查
        angles_ok = True
        for i in range(3):
            angle = _angle_at(points[i], points[(i + 1) % 3], points[(i + 2) % 3])
            if not (config.TRIANGLE_ANGLE_MIN <= angle <= config.TRIANGLE_ANGLE_MAX):
                angles_ok = False
                break
        if not angles_ok:
            continue

        # 三法加权
        avg_side = np.mean(sides)
        _, radius = cv2.minEnclosingCircle(cnt)
        theo_side = radius * np.sqrt(3)
        area_side = np.sqrt(4 * area / np.sqrt(3))
        side_px = avg_side * 0.6 + theo_side * 0.3 + area_side * 0.1

        return side_px, approx

    return None, None


# ============================================================
# 圆形检测
# ============================================================
def detect_circle(warped):
    """
    在校正后的 A4 区域内检测圆形

    Returns:
        (diameter_px, contour) 或 (None, None)
    """
    binary = _preprocess_warped(warped)
    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    best_diameter = None
    best_contour = None
    max_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < config.CIRCLE_AREA_MIN or area > config.CIRCLE_AREA_MAX:
            continue

        peri = cv2.arcLength(cnt, True)
        if peri == 0:
            continue

        # 角点数过滤
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        if len(approx) < config.CIRCLE_MIN_CORNERS:
            continue

        # 圆度
        circularity = (4 * np.pi * area) / (peri * peri)
        if circularity < config.CIRCLE_MIN_CIRCULARITY:
            continue

        if area > max_area:
            max_area = area
            diameter_px = np.sqrt(4 * area / np.pi) * circularity
            best_diameter = diameter_px
            best_contour = cnt

    return best_diameter, best_contour


# ============================================================
# 综合检测（按优先级尝试）
# ============================================================
_SHAPE_DETECTORS = [
    ("square", detect_square),
    ("triangle", detect_triangle),
    ("circle", detect_circle),
]


def detect_any_shape(warped):
    """
    按优先级检测任意形状：正方形 → 三角形 → 圆形

    Returns:
        (label, size_px, contour) 或 (None, 0, None)
    """
    for label, detector in _SHAPE_DETECTORS:
        size_px, contour = detector(warped)
        if size_px is not None and contour is not None:
            return label, size_px, contour
    return None, 0, None


# ============================================================
# 主入口：完整基础测量流程
# ============================================================
def detect_basic_shape(disp, img, focal_length, distance_filter):
    """
    基础测量主函数

    流程:
        1. A4 纸检测（自适应阈值法）
        2. 焦距标定（首次） / 距离计算
        3. 透视变换校正 A4 区域
        4. 区域内形状检测（正方/三角/圆）
        5. 轮廓逆映射回原始图像
        6. 显示结果

    Args:
        disp: Maix display 对象
        img: Maix 相机帧
        focal_length: 当前焦距（None 表示未标定，将自动标定）
        distance_filter: DistanceFilter 实例（会修改其内部状态）

    Returns:
        (distance_cm, side_cm, focal_length) — 距离、边长、焦距
    """
    cv_img = maix_to_cv(img)
    h, w = cv_img.shape[:2]

    # 1. A4 检测
    outer_approx, thresh_copy = detect_a4_adaptive(cv_img)

    if outer_approx is None:
        # 未检测到 A4 纸
        disp.show(img)
        return 0.0, 0.0, focal_length

    # 2. 像素尺寸
    a4_long_px, a4_short_px = get_a4_pixel_size(outer_approx)

    # 焦距自动标定
    if focal_length is None and a4_long_px > 10:
        focal_length = (a4_long_px * config.CALIBRATION_DISTANCE_MM) / config.CALIBRATION_A4_SIDE_MM

    # 3. 距离计算（带滤波）
    if focal_length is not None and a4_long_px > 10:
        raw_distance = calculate_distance(focal_length, a4_long_px)
        filtered_distance = distance_filter.update(raw_distance)
    else:
        filtered_distance = 0.0

    # 4. 透视变换校正
    try:
        warped, M = perspective_transform(thresh_copy, outer_approx)
        warped_h, warped_w = warped.shape[:2]
        if warped_h == 0:
            raise ValueError("校正后高度为 0")
        scale = config.A4_LONG_SIDE_MM / warped_h  # mm/pixel
    except Exception:
        disp.show(img)
        return round(filtered_distance / 10, 3), 0.0, focal_length

    # 5. 形状检测
    shape_label, size_px, shape_contour = detect_any_shape(warped)
    side_mm = 0.0

    if shape_label and size_px and shape_contour is not None:
        side_mm = size_px * scale

        # 逆映射轮廓到原始图像
        try:
            M_inv = cv2.invert(M)[1]
            pts = shape_contour.reshape(-1, 2).astype(np.float32)
            original_pts = cv2.perspectiveTransform(pts[None, :, :], M_inv)[0]
            original_contour = original_pts.astype(np.int32).reshape(-1, 1, 2)
            cv2.drawContours(cv_img, [original_contour], -1, (0, 0, 255), 2, cv2.LINE_AA)
        except Exception:
            pass

    # 6. 绘制 A4 外框和信息
    cv2.drawContours(cv_img, [outer_approx], -1, (255, 255, 0), 4, cv2.LINE_AA)

    if focal_length is not None:
        cv2.putText(cv_img, f"F: {focal_length:.1f}mm", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(cv_img, f"D: {filtered_distance / 10:.2f}cm", (10, 220),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if shape_label:
        cv2.putText(cv_img, f"{shape_label}: {side_mm / 10:.2f}cm",
                    (180, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.putText(cv_img, "BASIC", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 7. 显示
    if config.DEBUG_BINARY_VIEW and config.RUN_MODE != "uart":
        debug_view = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        disp.show(cv_to_maix(debug_view))
    else:
        disp.show(cv_to_maix(cv_img))

    # 8. 返回值（单位：cm）
    distance_cm = round(filtered_distance / 10, 3)
    side_cm = round(side_mm / 10, 3)

    # 经验补偿（原作者调参）
    if distance_cm >= 17.6:
        distance_cm = round(distance_cm * config.COMPENSATION_LONG_RANGE, 3)

    return distance_cm, side_cm, focal_length
"""
重叠/分离正方形检测模块 —— 发挥部分第 (1)(2) 项

算法思路:
    当多个正方形重叠时，外轮廓不再是简单的四边形。
    本模块通过"红绿点系统"分析轮廓顶点：
    - 红点：凸角且角度 ≈ 90°（可能是正方形的外角）
    - 绿点：其他顶点
    - 仅当两个红点在极角上相邻且中间无绿点时，才视为有效边
    - 有效边对的中点 + 半边长 = 内切圆，取最小半径圆 = 最小正方形
"""

# ============================================================
# 工具函数
# ============================================================
def _is_point_in_a4(pt, a4_contour):
    """判断点是否在 A4 纸轮廓内部"""
    if a4_contour is None:
        return False
    return cv2.pointPolygonTest(a4_contour, (int(pt[0]), int(pt[1])), False) >= 0


def _sort_by_angle(points):
    """
    按极角排序点（0~2π 范围）

    Returns:
        (sorted_points, center, angles)
    """
    if len(points) < 2:
        return points, None, []
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    angles = np.where(angles < 0, angles + 2 * np.pi, angles)
    idx = np.argsort(angles)
    return points[idx], center, angles[idx]


def _angle_between(angle, a1, a2):
    """判断 angle 是否在按逆时针方向从 a1 到 a2 的区间内（0~2π）"""
    if a1 <= a2:
        return a1 <= angle <= a2
    else:
        return angle >= a1 or angle <= a2


# ============================================================
# 主函数：重叠/分离正方形检测
# ============================================================
def detect_overlap_squares(disp, img, focal_length):
    """
    检测 A4 区域内重叠或分离的正方形组合图形，测量最小正方形边长。

    适用场景:
        - 发挥部分 (1)：多个彼此分离的正方形
        - 发挥部分 (2)：多个局部重叠的正方形

    Args:
        disp: Maix display 对象
        img: Maix 相机帧
        focal_length: 已标定的焦距

    Returns:
        (a4_distance_cm, min_edge_cm)
    """
    cv_img = maix_to_cv(img)
    result = cv_img.copy()

    # 1. A4 纸检测
    a4_contour, a4_thresh = detect_a4_adaptive(cv_img)
    if a4_contour is None:
        cv2.putText(result, "No A4 detected", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        disp.show(cv_to_maix(result))
        return 0.0, 0.0

    # 2. 计算距离
    a4_long_px = get_a4_pixel_size(a4_contour)[0]
    a4_distance_mm = calculate_distance(focal_length, a4_long_px)

    cv2.drawContours(result, [a4_contour], -1, (0, 255, 0), 2, cv2.LINE_AA)

    # 3. A4 内部区域掩码与二值化
    a4_mask = np.zeros(cv_img.shape[:2], np.uint8)
    cv2.drawContours(a4_mask, [a4_contour], -1, 255, -1)

    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
    thresh_a4 = cv2.bitwise_and(thresh, thresh, mask=a4_mask)

    contours, hierarchy = cv2.findContours(
        thresh_a4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    # 4. 找最大外轮廓（无父轮廓 = hierarchy[0][i][3] == -1）
    max_area = 0
    outer_contour = None
    if contours and hierarchy is not None:
        for i, cnt in enumerate(contours):
            if hierarchy[0][i][3] == -1 and cv2.contourArea(cnt) > 0:
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if _is_point_in_a4((cx, cy), a4_contour):
                    area = cv2.contourArea(cnt)
                    if area > max_area:
                        max_area = area
                        outer_contour = cnt

    # 5. 轮廓分类
    cnt_useful = []
    cnt_number = []
    cnt_useless = []
    if outer_contour is not None and max_area > 0:
        for cnt in contours:
            if cnt is outer_contour:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            if not _is_point_in_a4((cx, cy), a4_contour):
                continue

            area = cv2.contourArea(cnt)
            ratio = area / max_area
            if config.OVERLAP_AREA_RATIO_MIN < ratio < config.OVERLAP_AREA_RATIO_MAX:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                if len(approx) == 4:
                    cnt_useful.append(cnt)
                else:
                    cnt_number.append(cnt)
            else:
                cnt_useless.append(cnt)

    cv2.drawContours(result, cnt_number, -1, (0, 255, 255), 2, cv2.LINE_AA)

    # 6. 顶点分类：红点 / 绿点
    half_size = 8
    red_corners = []
    green_corners = []

    for contour in cnt_number:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
        vertices = [(int(p[0][0]), int(p[0][1])) for p in approx]
        n = len(vertices)
        if n < 3:
            continue

        for i in range(n):
            x, y = vertices[i]
            if not _is_point_in_a4((x, y), a4_contour):
                continue

            p_prev = vertices[(i - 1) % n]
            p_next = vertices[(i + 1) % n]
            v1 = (p_prev[0] - x, p_prev[1] - y)
            v2 = (p_next[0] - x, p_next[1] - y)
            len1 = np.hypot(*v1)
            len2 = np.hypot(*v2)

            if len1 > 0 and len2 > 0:
                dot = v1[0] * v2[0] + v1[1] * v2[1]
                cos_val = max(-1.0, min(1.0, dot / (len1 * len2)))
                angle = np.degrees(np.arccos(cos_val))
            else:
                angle = 180

            cross = v1[0] * v2[1] - v1[1] * v2[0]
            is_convex = cross > 0

            # 红点条件：凸角 + 角度 80°~105° + 边长足够
            if is_convex and config.RED_ANGLE_MIN <= angle <= config.RED_ANGLE_MAX:
                if len1 > config.RED_EDGE_MIN_LENGTH and len2 > config.RED_EDGE_MIN_LENGTH:
                    cv2.circle(result, (x, y), 3, (0, 0, 255), -1)
                    red_corners.append([x, y])
                else:
                    cv2.circle(result, (x, y), 3, (0, 255, 0), -1)
                    green_corners.append([x, y])
            else:
                cv2.circle(result, (x, y), 3, (0, 255, 0), -1)
                green_corners.append([x, y])

    # 7. 红点排序 + 圆分析
    all_circles = []
    min_radius = float("inf")
    min_radius_circle = None

    if len(red_corners) >= 2:
        red_np = np.array(red_corners)
        sorted_red, center, red_angles = _sort_by_angle(red_np)

        # 绿点极角
        green_angles = []
        for gx, gy in green_corners:
            if center is not None:
                ga = np.arctan2(gy - center[1], gx - center[0])
                if ga < 0:
                    ga += 2 * np.pi
                green_angles.append(ga)

        for i in range(len(sorted_red)):
            p1 = sorted_red[i]
            p2 = sorted_red[(i + 1) % len(sorted_red)]
            a1 = red_angles[i]
            a2 = red_angles[(i + 1) % len(sorted_red)]

            # 检查红点对之间是否有绿点（有绿点则跳过）
            has_green = any(_angle_between(ga, a1, a2) for ga in green_angles)
            if has_green:
                continue

            # 圆心在 A4 内才有效
            cx = int((p1[0] + p2[0]) / 2)
            cy = int((p1[1] + p2[1]) / 2)
            if not _is_point_in_a4((cx, cy), a4_contour):
                continue

            radius = int(np.hypot(p2[0] - p1[0], p2[1] - p1[1]) / 2)
            circle_info = {
                "center": (cx, cy),
                "radius": radius,
                "points": (tuple(p1), tuple(p2)),
            }
            all_circles.append(circle_info)
            cv2.circle(result, (cx, cy), radius, (0, 0, 255), 1, cv2.LINE_AA)

            if radius < min_radius:
                min_radius = radius
                min_radius_circle = circle_info

    # 8. 物理尺寸计算
    min_edge_mm = 0.0
    if min_radius_circle is not None:
        center = min_radius_circle["center"]
        radius = min_radius_circle["radius"]
        if _is_point_in_a4(center, a4_contour):
            # 直径 = 正方形边长（像素）
            edge_px = radius * 2
            min_edge_mm = (edge_px * a4_distance_mm) / focal_length

            cv2.circle(result, center, radius, (255, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(result, f"R:{min_edge_mm:.1f}mm",
                        (center[0] + 10, center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    # 9. 信息显示
    cv2.putText(result, f"A4 Dist: {a4_distance_mm:.1f}mm", (10, 220),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.putText(result, "ADV-OVERLAP", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 经验补偿
    min_edge_mm = min_edge_mm * config.COMPENSATION_OVERLAP

    disp.show(cv_to_maix(result))

    return round(a4_distance_mm / 10, 3), round(min_edge_mm / 10, 3)


# ============================================================
# 简化的分离正方形检测（发挥1）
# ============================================================
def detect_separated_squares(disp, img, focal_length):
    """
    检测 A4 区域内多个分离正方形，测量最小面积正方形的边长。

    比重叠版更简单：直接找所有正方形，取最小的。

    Args:
        disp: Maix display 对象
        img: Maix 相机帧
        focal_length: 已标定的焦距

    Returns:
        (a4_distance_cm, min_square_side_cm)
    """
    cv_img = maix_to_cv(img)
    result = cv_img.copy()

    # 1. A4 纸检测
    a4_contour, a4_thresh = detect_a4_adaptive(cv_img)
    if a4_contour is None:
        cv2.putText(result, "No A4 detected", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        disp.show(cv_to_maix(result))
        return 0.0, 0.0

    # 2. 距离
    a4_long_px = get_a4_pixel_size(a4_contour)[0]
    a4_distance_mm = calculate_distance(focal_length, a4_long_px)

    cv2.drawContours(result, [a4_contour], -1, (0, 255, 0), 4, cv2.LINE_AA)

    # 3. 透视校正
    try:
        warped, M = perspective_transform(a4_thresh, a4_contour)
        warped_h = warped.shape[0]
        if warped_h == 0:
            raise ValueError
        scale_mm_per_px = config.A4_LONG_SIDE_MM / warped_h
    except Exception:
        disp.show(cv_to_maix(result))
        a4_distance_cm = round(a4_distance_mm / 10, 3)
        if a4_distance_cm >= 17.6:
            a4_distance_cm = round(a4_distance_cm * config.COMPENSATION_LONG_RANGE, 3)
        return a4_distance_cm, 0.0

    # 4. 检测 A4 内部所有正方形
    if len(warped.shape) == 3:
        warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    else:
        warped_gray = warped

    # 反色 + Otsu
    inverted = cv2.bitwise_not(warped_gray)
    _, binary = cv2.threshold(inverted, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    squares = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < config.SHAPE_MIN_AREA:
            continue

        # A4 内边框过滤：排除 2cm 黑边区域
        M_m = cv2.moments(cnt)
        if M_m["m00"] == 0:
            continue
        cx = int(M_m["m10"] / M_m["m00"])
        border_px = int(config.A4_BLACK_BORDER_MM / scale_mm_per_px)
        if cx < border_px or cx > warped.shape[1] - border_px:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
        if len(approx) != 4:
            continue

        rect = cv2.minAreaRect(cnt)
        w_r, h_r = rect[1]
        ratio = min(w_r, h_r) / max(w_r, h_r)
        area_ratio = area / (w_r * h_r) if w_r * h_r > 0 else 0

        # 正方形验证
        if ratio > 0.85 and area_ratio > 0.85:
            squares.append((cnt, area, w_r, h_r))

    # 5. 找最小正方形
    min_square_side_mm = 0.0
    if squares:
        _, _, w_r, h_r = min(squares, key=lambda x: x[1])
        min_side_px = (w_r + h_r) / 2.0
        min_square_side_mm = min_side_px * scale_mm_per_px

        # 画最小正方形
        min_cnt = min(squares, key=lambda x: x[1])[0]
        M_inv = cv2.invert(M)[1]
        pts = min_cnt.reshape(-1, 2).astype(np.float32)
        org_pts = cv2.perspectiveTransform(pts[None, :, :], M_inv)[0]
        org_contour = org_pts.astype(np.int32).reshape(-1, 1, 2)
        cv2.drawContours(result, [org_contour], -1, (255, 255, 0), 2, cv2.LINE_AA)

    # 6. 显示
    cv2.putText(result, f"D: {a4_distance_mm:.1f}mm", (10, 220),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
    cv2.putText(result, f"Min: {min_square_side_mm:.1f}mm", (10, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(result, "ADV-SPLIT", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    disp.show(cv_to_maix(result))

    a4_distance_cm = round(a4_distance_mm / 10, 3)
    # 经验补偿
    if a4_distance_cm >= 16.0:
        a4_distance_cm = round(a4_distance_cm * config.COMPENSATION_FINE_MAZY, 3)
    min_square_side_cm = round(min_square_side_mm / 10, 3)

    return a4_distance_cm, min_square_side_cm
"""
旋转目标物检测模块 —— 发挥部分第 (4) 项

处理 A4 纸与摄像头轴线不成 90° 的情况（旋转 30°~60°）。
透视畸变更严重（梯形畸变），需要更大的轮廓逼近容差和帧间继承。

算法:
    - A4 检测使用 HSV 黑色掩码法（对梯形畸变更鲁棒）
    - 透视变换到 mm 坐标空间（1px ≈ 1mm）
    - 在 2cm 黑边内的有效区域检测正方形/三角形/圆形
    - 帧间轮廓继承防止短暂丢失
"""

def _is_equilateral(approx):
    """判断三角形是否为等边三角形"""
    if len(approx) != 3:
        return False
    pts = approx.reshape(3, 2)
    sides = [
        np.linalg.norm(pts[1] - pts[0]),
        np.linalg.norm(pts[2] - pts[1]),
        np.linalg.norm(pts[0] - pts[2]),
    ]
    avg = np.mean(sides)
    return all(abs(s - avg) / avg < 0.2 for s in sides)


def _detect_shapes_in_warped(warped):
    """
    在校正到 mm 坐标的 A4 区域内检测所有形状

    校正后图像：1 px ≈ 1 mm，210×297 尺寸。

    Returns:
        dict: {"square": size_mm, "triangle": size_mm, "circle": size_mm}
              size_mm 为 0 表示未检测到
    """
    border_px = int(config.A4_BLACK_BORDER_MM)  # 黑边宽度，mm ≈ px

    if len(warped.shape) == 3:
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    else:
        gray = warped

    binary = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        31, 10,
    )

    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    results = {"square": 0.0, "triangle": 0.0, "circle": 0.0}
    valid_w = warped.shape[1] - 2 * border_px
    valid_h = warped.shape[0] - 2 * border_px
    if valid_w <= 0 or valid_h <= 0:
        return results

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 10:
            continue

        M_m = cv2.moments(cnt)
        if M_m["m00"] == 0:
            continue
        cx = int(M_m["m10"] / M_m["m00"])
        cy = int(M_m["m01"] / M_m["m00"])

        # 确保在 2cm 黑边以内
        if (cx < border_px or cx > warped.shape[1] - border_px or
                cy < border_px or cy > warped.shape[0] - border_px):
            continue

        max_valid_area = 0.8 * valid_w * valid_h
        if area > max_valid_area:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        nsides = len(approx)

        x, y, w, h = cv2.boundingRect(cnt)

        if nsides == 4:
            aspect = w / float(h)
            if 0.85 < aspect < 1.15:
                if w > results["square"]:
                    results["square"] = w

        elif nsides == 3:
            if _is_equilateral(approx):
                if h > results["triangle"]:
                    results["triangle"] = h

        else:
            circularity = 4 * np.pi * area / (peri * peri)
            if circularity > 0.7:
                diameter = np.sqrt(4 * area / np.pi)
                if diameter > results["circle"]:
                    results["circle"] = diameter

    return results


# ============================================================
# 主函数
# ============================================================
def detect_rotation_shape(disp, img, focal_length):
    """
    旋转目标物检测

    流程:
        1. HSV 黑色掩码法检测 A4 纸（容忍梯形畸变）
        2. 使用长边计算距离
        3. 透视变换到 mm 坐标
        4. 在有效区域内检测所有形状
        5. 按优先级选择：正方形 > 三角形 > 圆形

    Args:
        disp: Maix display 对象
        img: Maix 相机帧
        focal_length: 已标定的焦距（必填）

    Returns:
        (distance_mm, shape_size_cm) — 注意：distance_mm 单位为 mm
    """
    cv_img = maix_to_cv(img)
    result = cv_img.copy()

    # 1. HSV 法检测 A4
    outer_approx, mask = detect_a4_hsv(cv_img)

    if outer_approx is None or focal_length is None:
        cv2.putText(result, "No A4 / No Calib", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        disp.show(cv_to_maix(result))
        return 0.0, 0.0

    # 2. 距离（使用长边）
    rect = cv2.minAreaRect(outer_approx)
    pixel_long = max(rect[1])
    if pixel_long <= 10:
        disp.show(cv_to_maix(result))
        return 0.0, 0.0

    a4_distance_mm = calculate_distance(focal_length, pixel_long, config.A4_LONG_SIDE_MM)

    # 3. 透视变换到 mm 坐标
    try:
        warped, M = perspective_transform(
            cv_img, outer_approx,
            target_w=int(config.A4_SHORT_SIDE_MM),
            target_h=int(config.A4_LONG_SIDE_MM),
        )
    except Exception:
        disp.show(img)
        return round(a4_distance_mm, 3), 0.0

    # 4. 检测形状
    shapes = _detect_shapes_in_warped(warped)

    shape_size_mm = 0.0
    shape_label = ""

    for label in ["square", "triangle", "circle"]:
        if shapes[label] > 0:
            shape_size_mm = shapes[label]
            shape_label = label
            break

    # 5. 显示
    cv2.drawContours(result, [outer_approx], -1, (255, 255, 0), 4, cv2.LINE_AA)

    if focal_length is not None:
        cv2.putText(result, f"F: {focal_length:.1f}mm", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    if shape_label:
        cv2.putText(result, f"{shape_label}: {shape_size_mm:.2f}mm",
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(result, "ADV-ROTATE", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 经验补偿
    shape_size_mm = shape_size_mm * config.COMPENSATION_ROTATION

    disp.show(cv_to_maix(result))

    # 返回（统一单位: cm）
    distance_cm = round(a4_distance_mm / 10, 3)
    size_cm = round(shape_size_mm / 10, 3)

    return distance_cm, size_cm

