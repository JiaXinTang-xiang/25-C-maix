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

import cv2
import numpy as np

import config
import camera_utils as cu


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
def find_rotation_shape(disp, img, focal_length):
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
    cv_img = cu.maix_to_cv(img)
    result = cv_img.copy()

    # 1. HSV 法检测 A4
    outer_approx, mask = cu.detect_a4_hsv(cv_img)

    if outer_approx is None or focal_length is None:
        cv2.putText(result, "No A4 / No Calib", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        disp.show(cu.cv_to_maix(result))
        return 0.0, 0.0

    # 2. 距离（使用长边）
    rect = cv2.minAreaRect(outer_approx)
    pixel_long = max(rect[1])
    if pixel_long <= 10:
        disp.show(cu.cv_to_maix(result))
        return 0.0, 0.0

    a4_distance_mm = cu.calculate_distance(focal_length, pixel_long, config.A4_LONG_SIDE_MM)

    # 3. 透视变换到 mm 坐标
    try:
        warped, M = cu.perspective_transform(
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

    disp.show(cu.cv_to_maix(result))

    # 返回（统一单位: cm）
    distance_cm = round(a4_distance_mm / 10, 3)
    size_cm = round(shape_size_mm / 10, 3)

    return distance_cm, size_cm
