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

import cv2
import numpy as np

import config
import camera_utils as cu


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
def find_overlap_shape(disp, img, focal_length):
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
    cv_img = cu.maix_to_cv(img)
    result = cv_img.copy()

    # 1. A4 纸检测
    a4_contour, a4_thresh = cu.detect_a4_adaptive(cv_img)
    if a4_contour is None:
        cv2.putText(result, "No A4 detected", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        disp.show(cu.cv_to_maix(result))
        return 0.0, 0.0

    # 2. 计算距离
    a4_long_px = cu.get_a4_pixel_size(a4_contour)[0]
    a4_distance_mm = cu.calculate_distance(focal_length, a4_long_px)

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

    disp.show(cu.cv_to_maix(result))

    return round(a4_distance_mm / 10, 3), round(min_edge_mm / 10, 3)


# ============================================================
# 简化的分离正方形检测（发挥1）
# ============================================================
def find_separated_squares(disp, img, focal_length):
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
    cv_img = cu.maix_to_cv(img)
    result = cv_img.copy()

    # 1. A4 纸检测
    a4_contour, a4_thresh = cu.detect_a4_adaptive(cv_img)
    if a4_contour is None:
        cv2.putText(result, "No A4 detected", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        disp.show(cu.cv_to_maix(result))
        return 0.0, 0.0

    # 2. 距离
    a4_long_px = cu.get_a4_pixel_size(a4_contour)[0]
    a4_distance_mm = cu.calculate_distance(focal_length, a4_long_px)

    cv2.drawContours(result, [a4_contour], -1, (0, 255, 0), 4, cv2.LINE_AA)

    # 3. 透视校正
    try:
        warped, M = cu.perspective_transform(a4_thresh, a4_contour)
        warped_h = warped.shape[0]
        if warped_h == 0:
            raise ValueError
        scale_mm_per_px = config.A4_LONG_SIDE_MM / warped_h
    except Exception:
        disp.show(cu.cv_to_maix(result))
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

    disp.show(cu.cv_to_maix(result))

    a4_distance_cm = round(a4_distance_mm / 10, 3)
    # 经验补偿
    if a4_distance_cm >= 16.0:
        a4_distance_cm = round(a4_distance_cm * config.COMPENSATION_FINE_MAZY, 3)
    min_square_side_cm = round(min_square_side_mm / 10, 3)

    return a4_distance_cm, min_square_side_cm
