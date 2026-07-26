"""Standalone visual debugger for the MaixCam vision algorithms."""

import cv2
import numpy as np
from maix import app, camera, display, time

import config
import vision


def _make_binary(cv_img):
    """始终生成二值化图像，不管有没有检测到 A4"""
    gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    thresh = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        config.ADAPTIVE_THRESH_BLOCK,
        config.ADAPTIVE_THRESH_C,
    )
    kernel = np.ones(config.A4_MORPH_KERNEL, np.uint8)
    mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    return mask


def _show_a4_debug(disp, img):
    cv_img = vision.maix_to_cv(img)
    contour, _ = vision.detect_a4_adaptive(cv_img)

    if config.DEBUG_VIEW == "binary":
        # 独立生成二值化图——不管有没有检测到 A4，都能看
        binary = _make_binary(cv_img)
        view = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    else:
        view = cv_img.copy()
        if contour is not None:
            cv2.drawContours(view, [contour], -1, (0, 255, 0), 3)

    status = "A4: FOUND" if contour is not None else "A4: NOT FOUND"
    color = (0, 255, 0) if contour is not None else (0, 0, 255)
    cv2.putText(view, status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, color, 2)
    disp.show(vision.cv_to_maix(view))
    return contour is not None


def _calibrate(cam, disp):
    known_cm = config.CALIBRATION_DISTANCE_MM / 10.0
    print(f"[DEBUG] Place A4 at {known_cm:.0f} cm for calibration")

    while not app.need_exit():
        img = cam.read()
        cv_img = vision.maix_to_cv(img)
        contour, binary = vision.detect_a4_adaptive(cv_img)

        if contour is not None:
            focal = vision.calibrate_focal_length(cv_img)
            if focal is not None:
                print(f"[DEBUG] Calibration OK, focal={focal:.2f}px")
                return focal

        if config.DEBUG_VIEW == "binary":
            binary = _make_binary(cv_img)
            view = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        else:
            view = cv_img
        cv2.putText(view, f"CALIBRATE A4 AT {known_cm:.0f}CM", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        disp.show(vision.cv_to_maix(view))
        time.sleep(0.05)

    return None


def main():
    task = config.DEBUG_TASK.lower()
    valid_tasks = ("a4", "basic", "separated", "overlap", "rotation")
    if task not in valid_tasks:
        print(f"[DEBUG] Invalid task: {task}")
        print(f"[DEBUG] Choose one of: {', '.join(valid_tasks)}")
        return

    disp = display.Display()
    cam = camera.Camera(config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
    cam.skip_frames(config.CAMERA_SKIP_FRAMES)
    distance_filter = vision.DistanceFilter()

    print(f"[DEBUG] Task={task}, view={config.DEBUG_VIEW}")
    print("[DEBUG] UART is disabled")

    if task == "a4":
        frame_count = 0
        while not app.need_exit():
            found = _show_a4_debug(disp, cam.read())
            frame_count += 1
            if frame_count % max(1, config.DEBUG_PRINT_INTERVAL) == 0:
                print(f"[DEBUG/A4] found={found}")
            time.sleep(0.01)
        return

    focal_length = _calibrate(cam, disp)
    if focal_length is None:
        print("[DEBUG] Calibration cancelled or failed")
        return

    # The basic detector can show either its annotated result or warped binary.
    config.DEBUG_BINARY_VIEW = config.DEBUG_VIEW == "binary"
    config.RUN_MODE = task

    detectors = {
        "separated": vision.detect_separated_squares,
        "overlap": vision.detect_overlap_squares,
        "rotation": vision.detect_rotation_shape,
    }
    frame_count = 0

    while not app.need_exit():
        img = cam.read()
        if task == "basic":
            distance_cm, size_cm, focal_length = vision.detect_basic_shape(
                disp, img, focal_length, distance_filter)
        else:
            distance_cm, size_cm = detectors[task](disp, img, focal_length)

        frame_count += 1
        if frame_count % max(1, config.DEBUG_PRINT_INTERVAL) == 0:
            print(f"[DEBUG/{task}] D={distance_cm:.2f}cm, size={size_cm:.2f}cm")
        time.sleep(0.01)


if __name__ == "__main__":
    main()
