"""
基础形状检测模块 —— 正方形、等边三角形、圆形
用于基本要求部分（基础题）
"""

import cv2
import numpy as np

import config
import camera_utils as cu


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
def find_shape(disp, img, focal_length, distance_filter):
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
    cv_img = cu.maix_to_cv(img)
    h, w = cv_img.shape[:2]

    # 1. A4 检测
    outer_approx, thresh_copy = cu.detect_a4_adaptive(cv_img)

    if outer_approx is None:
        # 未检测到 A4 纸
        disp.show(img)
        return 0.0, 0.0, focal_length

    # 2. 像素尺寸
    a4_long_px, a4_short_px = cu.get_a4_pixel_size(outer_approx)

    # 焦距自动标定
    if focal_length is None and a4_long_px > 10:
        focal_length = (a4_long_px * config.CALIBRATION_DISTANCE_MM) / config.CALIBRATION_A4_SIDE_MM

    # 3. 距离计算（带滤波）
    if focal_length is not None and a4_long_px > 10:
        raw_distance = cu.calculate_distance(focal_length, a4_long_px)
        filtered_distance = distance_filter.update(raw_distance)
    else:
        filtered_distance = 0.0

    # 4. 透视变换校正
    try:
        warped, M = cu.perspective_transform(thresh_copy, outer_approx)
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
    disp.show(cu.cv_to_maix(cv_img))

    # 8. 返回值（单位：cm）
    distance_cm = round(filtered_distance / 10, 3)
    side_cm = round(side_mm / 10, 3)

    # 经验补偿（原作者调参）
    if distance_cm >= 17.6:
        distance_cm = round(distance_cm * config.COMPENSATION_LONG_RANGE, 3)

    return distance_cm, side_cm, focal_length
