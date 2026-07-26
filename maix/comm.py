"""
通信模块 —— UART 数据打包与指令解析

与 STM32 主控的通信协议：

上位机（STM32） → 下位机（MaixCam）: 单字节触发指令
下位机（MaixCam） → 上位机（STM32）: 8 字节数据包

数据包格式:
    [Header(2)] [Distance(2)] [Size(2)] [Tail(2)]

    其中 Distance 和 Size 为 uint16，值 = 实际值 × 100（保留 2 位小数）
    - 基础测量包尾: CC DD
    - 发挥测量包尾: EE FF
"""

from maix import pinmap, uart

import config


# ============================================================
# UART 初始化
# ============================================================
_uart = None


def init_uart(device=None, baudrate=None, rx_pin=None, tx_pin=None):
    """
    初始化 UART 通信

    Args:
        device: UART 设备路径
        baudrate: 波特率
        rx_pin: RX 引脚号
        tx_pin: TX 引脚号

    Returns:
        uart.UART 对象
    """
    global _uart

    dev = device or config.UART_DEVICE
    baud = baudrate or config.UART_BAUDRATE
    rx = rx_pin or config.UART_RX_PIN
    tx = tx_pin or config.UART_TX_PIN

    # 引脚功能映射
    pinmap.set_pin_function(rx, f"UART1_RX")
    pinmap.set_pin_function(tx, f"UART1_TX")

    _uart = uart.UART(dev, baud)
    return _uart


def get_uart():
    """获取已初始化的 UART 对象"""
    return _uart


# ============================================================
# 数据打包
# ============================================================
def _float_to_uint16(value, scale=100):
    """
    浮点数 → uint16

    Args:
        value: 浮点数值
        scale: 乘数（默认 100，保留 2 位小数）

    Returns:
        (high_byte, low_byte)
    """
    val = value if value is not None else 0
    integer = int(val * scale)
    integer = max(0, min(65535, integer))  # 钳位到 uint16 范围
    return (integer >> 8) & 0xFF, integer & 0xFF


def pack_basic(distance_cm, size_cm):
    """
    打包基础测量数据

    Args:
        distance_cm: 距离（cm）
        size_cm: 边长/直径（cm）

    Returns:
        bytes: 8 字节数据包
    """
    h, l = config.PACKET_HEADER
    dh, dl = _float_to_uint16(distance_cm)
    sh, sl = _float_to_uint16(size_cm)
    t1, t2 = config.PACKET_TAIL_BASIC
    return bytes([h, l, dh, dl, sh, sl, t1, t2])


def pack_advanced(distance_cm, size_cm):
    """
    打包发挥部分测量数据

    Args:
        distance_cm: 距离（cm）
        size_cm: 边长（cm）

    Returns:
        bytes: 8 字节数据包
    """
    h, l = config.PACKET_HEADER
    dh, dl = _float_to_uint16(distance_cm)
    sh, sl = _float_to_uint16(size_cm)
    t1, t2 = config.PACKET_TAIL_ADVANCED
    return bytes([h, l, dh, dl, sh, sl, t1, t2])


def pack_calib_done():
    """打包标定完成信号"""
    return config.CALIB_DONE_SIGNAL


# ============================================================
# 指令解析
# ============================================================
class Command:
    """STM32 发来的触发指令"""
    NONE = None
    BASE = 0x01
    ADVANCED_1 = 0x02    # 分离正方形
    ADVANCED_2 = 0x03    # 重叠正方形
    ADVANCED_3 = 0x04    # 数字编号正方形
    ADVANCED_4 = 0x05    # 旋转目标物


def parse_command(byte_data):
    """
    解析单字节指令

    Args:
        byte_data: bytes (单字节) 或 None

    Returns:
        Command 枚举值
    """
    if byte_data is None or len(byte_data) == 0:
        return Command.NONE

    cmd = byte_data[0]
    if cmd == Command.BASE:
        return Command.BASE
    elif cmd == Command.ADVANCED_1:
        return Command.ADVANCED_1
    elif cmd == Command.ADVANCED_2:
        return Command.ADVANCED_2
    elif cmd == Command.ADVANCED_3:
        return Command.ADVANCED_3
    elif cmd == Command.ADVANCED_4:
        return Command.ADVANCED_4

    return Command.NONE


# ============================================================
# 发送辅助函数
# ============================================================
def send_basic(distance_cm, size_cm):
    """发送基础测量结果"""
    if _uart is None:
        print(f"[COMM] BASIC: D={distance_cm}cm, x={size_cm}cm")
        return
    packet = pack_basic(distance_cm, size_cm)
    _uart.write(packet)


def send_advanced(distance_cm, size_cm):
    """发送发挥部分测量结果"""
    if _uart is None:
        print(f"[COMM] ADVANCED: D={distance_cm}cm, x={size_cm}cm")
        return
    packet = pack_advanced(distance_cm, size_cm)
    _uart.write(packet)


def send_calib_done():
    """发送标定完成信号"""
    if _uart is None:
        print("[COMM] Calibration done")
        return
    _uart.write(pack_calib_done())


# ============================================================
# 调试工具
# ============================================================
def print_hex(data: bytes, label=""):
    """打印十六进制数据（调试用）"""
    hex_str = " ".join(f"0x{b:02X}" for b in data)
    if label:
        print(f"[{label}] {hex_str}")
    else:
        print(hex_str)
