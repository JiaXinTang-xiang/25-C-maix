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
import vision
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
    cv_img = vision.maix_to_cv(img)
    h, w = cv_img.shape[:2]
    cv2.putText(cv_img, text, (w // 4, h // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)
    disp.show(vision.cv_to_maix(cv_img))


# ============================================================
# 初始化硬件
# ============================================================
def init():
    global disp, cam, distance_filter, use_uart

    disp = display.Display()
    cam = camera.Camera(config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
    cam.skip_frames(config.CAMERA_SKIP_FRAMES)
    distance_filter = vision.DistanceFilter()

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
        cv_img = vision.maix_to_cv(img)
        focal = vision.calibrate_focal_length(cv_img)

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

    print("[MAIN] Ready. Waiting for commands...")

    ready_announce_count = 0
    current_cmd = comm.Command.NONE   # 锁存指令
    cmd_timeout = 0                   # 超时计数：连续无指令帧数

    while not app.need_exit():
        img = cam.read()

        # ---- 读取指令（收到新指令更新锁存+复位超时） ----
        if use_uart:
            u = comm.get_uart()
            if u:
                raw = u.read(1)
                new_cmd = comm.parse_command(raw)
                if new_cmd is not comm.Command.NONE:
                    current_cmd = new_cmd
                    cmd_timeout = 0    # 收到指令，清零超时
                else:
                    cmd_timeout += 1   # 空帧，超时累加
                    if cmd_timeout > 50:   # ~0.5s 没指令，回空闲
                        current_cmd = comm.Command.NONE
            else:
                current_cmd = comm.Command.NONE
        else:
            vision.detect_basic_shape(
                disp, img, focal_length, distance_filter)
            time.sleep(0.01)
            continue

        cmd = current_cmd   # 使用锁存的指令

        # ================================================================
        # 基础测量
        # ================================================================
        if cmd == comm.Command.BASE:
            d_cm, x_cm, focal_length = vision.detect_basic_shape(
                disp, img, focal_length, distance_filter)
            comm.send_basic(d_cm, x_cm)
            print(f"[BASIC] D={d_cm:.2f}cm  x={x_cm:.2f}cm")

        # ================================================================
        # 发挥 1：分离正方形 (0x02)
        # ================================================================
        elif cmd == comm.Command.ADVANCED_1:
            d_cm, x_cm = vision.detect_separated_squares(
                disp, img, focal_length)
            comm.send_advanced(d_cm, x_cm)
            print(f"[ADV1/SEPARATED] D={d_cm:.2f}cm  x={x_cm:.2f}cm")

        # ================================================================
        # 发挥 2：重叠正方形 (0x03)
        # ================================================================
        elif cmd == comm.Command.ADVANCED_2:
            d_cm, x_cm = vision.detect_overlap_squares(
                disp, img, focal_length)
            comm.send_advanced(d_cm, x_cm)
            print(f"[ADV2/OVERLAP] D={d_cm:.2f}cm  x={x_cm:.2f}cm")

        # ================================================================
        # 发挥 3：数字编号正方形 (0x04) — 占位
        # ================================================================
        elif cmd == comm.Command.ADVANCED_3:
            cv_img = vision.maix_to_cv(img)
            a4, _ = vision.detect_a4_adaptive(cv_img)
            d_cm = 0.0
            if a4 is not None:
                long_px = vision.get_a4_pixel_size(a4)[0]
                d_cm = round(vision.calculate_distance(focal_length, long_px) / 10, 3)
            x_cm = 6.341  # 占位（NN 未训练）
            comm.send_advanced(d_cm, x_cm)
            print(f"[ADV3/DIGIT] D={d_cm:.2f}cm (stub)")

        # ================================================================
        # 发挥 4：旋转目标物 (0x05)
        # ================================================================
        elif cmd == comm.Command.ADVANCED_4:
            d_cm, x_cm = vision.detect_rotation_shape(
                disp, img, focal_length)
            comm.send_advanced(d_cm, x_cm)
            print(f"[ADV4/ROTATE] D={d_cm:.2f}cm  x={x_cm:.2f}cm")

        else:
            # STM32可能晚于MaixCam启动，空闲时周期重发准备信号。
            ready_announce_count += 1
            if ready_announce_count >= 10:
                comm.send_calib_done()
                ready_announce_count = 0

            vision.detect_basic_shape(disp, img, focal_length, distance_filter)

        time.sleep(0.01)


# ============================================================
if __name__ == "__main__":
    if config.PROGRAM_MODE == "debug":
        import debug
        debug.main()
    else:
        main()
