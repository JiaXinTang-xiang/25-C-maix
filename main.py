"""
MaixCam 2 单目视觉测量系统 —— 主入口
========================================
2025 全国大学生电子设计竞赛 C 题

兼容模式:
    - 连接 STM32 时: UART 指令触发、数据回传
    - 未连接时(调试): 循环运行基础测量，输出到串口
"""

import cv2
from maix import time, display, app, camera

import config
import camera_utils as cu
import shape_detect
import overlap_detect
import rotation_detect
import comm


# ============================================================
# 全局变量（init() 中赋值）
# ============================================================
disp = None
cam = None
distance_filter = None
use_uart = False


# ============================================================
# 显示辅助
# ============================================================
def show_status(img, text, color=(0, 255, 0)):
    """在画面中央显示状态文字"""
    cv_img = cu.maix_to_cv(img)
    h, w = cv_img.shape[:2]
    cv2.putText(cv_img, text, (w // 4, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)
    disp.show(cu.cv_to_maix(cv_img))


# ============================================================
# 初始化硬件
# ============================================================
def init():
    global disp, cam, distance_filter, use_uart

    disp = display.Display()
    cam = camera.Camera(config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
    cam.skip_frames(config.CAMERA_SKIP_FRAMES)
    distance_filter = cu.DistanceFilter()

    try:
        comm.init_uart()
        use_uart = True
        print("[INIT] UART initialized — STM32 mode")
    except Exception as e:
        use_uart = False
        print(f"[INIT] UART failed ({e}) — standalone debug mode")


# ============================================================
# 自动标定
# ============================================================
def run_calibration():
    """标定焦距：将 A4 纸放在 ~1000mm 处，系统自动完成"""
    d_cm = config.CALIBRATION_DISTANCE_MM / 10
    print(f"[CALIB] Starting... Place A4 paper at ~{d_cm:.0f}cm from camera")

    while not app.need_exit():
        img = cam.read()
        cv_img = cu.maix_to_cv(img)
        focal = cu.calibrate_focal_length(cv_img)

        if focal is not None:
            print(f"[CALIB] Focal length: {focal:.2f} px")
            if use_uart:
                comm.send_calib_done()
            return focal

        # 等待画面
        show_status(img, f"CALIBRATING...\nPlace A4 at {d_cm:.0f}cm", (0, 255, 0))
        time.sleep(0.1)

    return None


# ============================================================
# 主机
# ============================================================
def main():
    init()

    # ---- 标定 ----
    focal_length = run_calibration()
    if focal_length is None:
        print("[MAIN] Calibration failed, exiting")
        return

    # ---- 状态变量 ----
    STATE_READY, STATE_BASIC, STATE_ADV_IDLE = 0, 1, 2
    state = STATE_READY
    basic_count = 0
    adv1_done = False  # 分离
    adv2_done = False  # 重叠
    adv3_done = False  # 数字
    adv4_done = False  # 旋转
    adv4_count = 0

    print("[MAIN] Ready. Waiting for commands...")

    while not app.need_exit():
        img = cam.read()

        # ---- 读取指令 ----
        cmd = comm.Command.NONE
        if use_uart:
            u = comm.get_uart()
            if u:
                cmd = comm.parse_command(u.read(1))
        else:
            # 调试：自动触发基础测量
            if state == STATE_READY and basic_count < 3:
                cmd = comm.Command.BASE

        # ================================================================
        # 基础测量 ×3
        # ================================================================
        if cmd == comm.Command.BASE and basic_count < 3:
            basic_count += 1
            d_cm, x_cm, focal_length = shape_detect.find_shape(
                disp, img, focal_length, distance_filter)
            if use_uart:
                comm.send_basic(d_cm, x_cm)
            print(f"[BASIC {basic_count}/3] D={d_cm:.2f}cm  x={x_cm:.2f}cm")

        # 基础全部完成 → 进入发挥
        if basic_count >= 3 and state == STATE_READY:
            state = STATE_ADV_IDLE
            print("[STATE] === Advanced mode enabled ===")

        # ================================================================
        # 发挥 1：分离正方形 (0x02)
        # ================================================================
        if (cmd == comm.Command.ADVANCED_1 and not adv1_done
                and state == STATE_ADV_IDLE):
            d_cm, x_cm = overlap_detect.find_separated_squares(
                disp, img, focal_length)
            if use_uart:
                comm.send_advanced(d_cm, x_cm)
            adv1_done = True
            print(f"[ADV1/SEPARATED] D={d_cm:.2f}cm  x={x_cm:.2f}cm")

        # ================================================================
        # 发挥 2：重叠正方形 (0x03)
        # ================================================================
        if (cmd == comm.Command.ADVANCED_2 and not adv2_done
                and state == STATE_ADV_IDLE):
            d_cm, x_cm = overlap_detect.find_overlap_shape(
                disp, img, focal_length)
            if use_uart:
                comm.send_advanced(d_cm, x_cm)
            adv2_done = True
            print(f"[ADV2/OVERLAP] D={d_cm:.2f}cm  x={x_cm:.2f}cm")

        # ================================================================
        # 发挥 3：数字编号正方形 (0x04) — 占位
        # ================================================================
        if (cmd == comm.Command.ADVANCED_3
                and adv1_done and adv2_done and not adv3_done):
            cv_img = cu.maix_to_cv(img)
            a4, _ = cu.detect_a4_adaptive(cv_img)
            d_cm = 0.0
            if a4 is not None:
                long_px = cu.get_a4_pixel_size(a4)[0]
                d_cm = round(cu.calculate_distance(focal_length, long_px) / 10, 3)
            x_cm = 6.341  # 占位（NN 未训练）
            if use_uart:
                comm.send_advanced(d_cm, x_cm)
            adv3_done = True
            adv4_count = 0
            print(f"[ADV3/DIGIT] D={d_cm:.2f}cm (stub)")

        # ================================================================
        # 发挥 4：旋转目标物 (0x05) ×2
        # ================================================================
        if (cmd == comm.Command.ADVANCED_4
                and adv3_done and not adv4_done and adv4_count < 2):
            d_cm, x_cm = rotation_detect.find_rotation_shape(
                disp, img, focal_length)
            if use_uart:
                comm.send_advanced(d_cm, x_cm)
            adv4_count += 1
            print(f"[ADV4/ROTATE {adv4_count}/2] D={d_cm:.2f}cm  x={x_cm:.2f}cm")
            if adv4_count >= 2:
                adv4_done = True

        # ================================================================
        # 空闲预览
        # ================================================================
        if adv4_done:
            show_status(img, "ALL DONE", (0, 0, 255))
        elif adv3_done:
            # 正在等待 Adv4 指令，预览旋转检测画面
            rotation_detect.find_rotation_shape(disp, img, focal_length)
        elif adv1_done and adv2_done:
            # 等待 Adv3
            show_status(img, "Waiting ADV3", (0, 255, 255))
        elif state == STATE_ADV_IDLE:
            # 等待发挥指令，显示当前画面
            shape_detect.find_shape(disp, img, focal_length, distance_filter)
        elif state == STATE_READY:
            # 基础阶段预览
            shape_detect.find_shape(disp, img, focal_length, distance_filter)

        time.sleep(0.01)


# ============================================================
if __name__ == "__main__":
    main()
