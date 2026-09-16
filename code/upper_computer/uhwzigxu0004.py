import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
import sys

# ==================== 配置区 ====================
CAMERA_ID = 0
BAUDRATE = 115200
SPEED_LEVEL = 3              # 手动控制默认速度档位（1~10）
AUTO_SPEED = 3               # 自动对准固定速度（强制3档，避免帧率低震荡）
DEAD_ZONE = 15               # 死区（像素）
AUTO_IDLE_TIME = 1.0         # 手动空闲多久后进入自动对准（秒）
HOMING_TIMEOUT = 10          # 调零最大等待时间（秒）

# 红色HSV阈值
LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([180, 255, 255])

# ==================== 串口功能 ====================
def find_ch340_port():
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in ports:
        if "CH340" in desc or "USB-SERIAL" in desc or "USB Serial" in desc:
            return port
    return None

def send_command(ser, cmd, data):
    if ser is None:
        return
    checksum = (0xAA + cmd + data) & 0xFF
    frame = bytes([0xAA, cmd, data, checksum])
    ser.write(frame)

def send_stop(ser):
    send_command(ser, 0x03, 0)

def send_forward(ser, speed):
    send_command(ser, 0x01, speed)

def send_reverse(ser, speed):
    send_command(ser, 0x02, speed)

def send_homing(ser):
    send_command(ser, 0x05, 0)

# ==================== 视觉识别 ====================
def detect_red_target(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    mask = cv2.bitwise_or(mask1, mask2)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < 100:
        return None
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

def draw_debug(frame, target, dx, mode, estop, speed):
    h, w = frame.shape[:2]
    cx = w // 2
    cv2.line(frame, (cx, 0), (cx, h), (255, 255, 255), 1)
    if target:
        tx, ty = target
        cv2.circle(frame, (tx, ty), 10, (0, 255, 0), 2)
        cv2.line(frame, (cx, ty), (tx, ty), (0, 255, 255), 1)
        cv2.putText(frame, f"dx: {dx}px", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    else:
        cv2.putText(frame, "No target!", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(frame, f"Manual Speed: {speed}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
    cv2.putText(frame, f"Auto Speed: {AUTO_SPEED} (fixed)", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.putText(frame, f"Mode: {mode}", (10, 110),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    if estop:
        cv2.putText(frame, "!!! E-STOP ACTIVE !!!", (w//2-150, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
    cv2.putText(frame, "W/S: move  Space:stop  E:emerg  Z:home  0-9:speed  Q:quit",
                (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

# ==================== 自动对准更新（强制速度 = AUTO_SPEED） ====================
last_dir_sent = 0   # 0=停止, 1=正转, -1=反转
last_stop_sent = False

def auto_align_update(ser, cap):
    global last_dir_sent, last_stop_sent

    ret, frame = cap.read()
    if not ret:
        return False

    target = detect_red_target(frame)
    if target is None:
        if not last_stop_sent:
            send_stop(ser)
            last_dir_sent = 0
            last_stop_sent = True
        draw_debug(frame, None, 0, "AutoAlign (no target)", False, SPEED_LEVEL)
        cv2.imshow("Visual Servo", frame)
        return False

    last_stop_sent = False

    h, w = frame.shape[:2]
    cx = w // 2
    tx, ty = target
    dx = tx - cx

    draw_debug(frame, target, dx, "AutoAlign", False, SPEED_LEVEL)
    cv2.imshow("Visual Servo", frame)

    if abs(dx) < DEAD_ZONE:
        if not last_stop_sent:
            send_stop(ser)
            last_dir_sent = 0
            last_stop_sent = True
        return True

    # 未居中：决定方向，使用固定 AUTO_SPEED
    if dx > 0:
        if last_dir_sent != -1:
            send_reverse(ser, AUTO_SPEED)   # 强制用 AUTO_SPEED
            last_dir_sent = -1
            last_stop_sent = False
    else:
        if last_dir_sent != 1:
            send_forward(ser, AUTO_SPEED)   # 强制用 AUTO_SPEED
            last_dir_sent = 1
            last_stop_sent = False

    return False

# ==================== 主函数 ====================
def main():
    global SPEED_LEVEL, last_dir_sent, last_stop_sent

    # 串口连接
    port = find_ch340_port()
    if port is None:
        port = input("未检测到CH340，请输入端口号 (如 COM3): ").strip()
    try:
        ser = serial.Serial(port, BAUDRATE, timeout=0.1)
        print(f"✅ 串口 {port} 连接成功！")
    except Exception as e:
        print(f"❌ 串口连接失败: {e}")
        ser = None

    # 摄像头
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("❌ 摄像头打开失败！")
        if ser and ser.is_open:
            ser.close()
        sys.exit(1)
    print("✅ 摄像头打开成功！")

    # 状态变量
    mode = "manual"
    estop = False
    last_manual_time = 0
    manual_stopped = True
    frame_count = 0

    print("\n" + "="*50)
    print("键盘控制: W正转  S反转  空格停止  E急停  Z调零  0-9调速  Q退出")
    print("自动对准速度固定为3档（防震荡）")
    print("松开手动按键后，程序将自动对准红色物体到画面中央")
    print("="*50 + "\n")

    while True:
        key = cv2.waitKey(20) & 0xFF
        frame_count += 1

        # ====== 急停处理 ======
        if key == ord('e') or key == ord('E'):
            estop = not estop
            if estop:
                send_stop(ser)
                last_dir_sent = 0
                last_stop_sent = True
                print("!!! 急停开启 !!!")
            else:
                print("急停解除")
                last_stop_sent = False
                mode = "manual"
                manual_stopped = True
                last_manual_time = time.time()
            continue

        if estop:
            ret, frame = cap.read()
            if ret:
                target = detect_red_target(frame)
                draw_debug(frame, target, 0, "ESTOP", True, SPEED_LEVEL)
                cv2.imshow("Visual Servo", frame)
            if key == ord('q') or key == ord('Q'):
                send_stop(ser)
                print("退出程序")
                break
            continue

        # ====== 按键处理 ======
        if key == ord('q') or key == ord('Q'):
            send_stop(ser)
            print("退出程序")
            break

        elif key == ord('w') or key == ord('W'):
            send_forward(ser, SPEED_LEVEL)
            last_dir_sent = 1
            last_stop_sent = False
            mode = "manual"
            last_manual_time = time.time()
            manual_stopped = False
            print(f"手动正转 (速度{SPEED_LEVEL})")

        elif key == ord('s') or key == ord('S'):
            send_reverse(ser, SPEED_LEVEL)
            last_dir_sent = -1
            last_stop_sent = False
            mode = "manual"
            last_manual_time = time.time()
            manual_stopped = False
            print(f"手动反转 (速度{SPEED_LEVEL})")

        elif key == 32:  # 空格
            send_stop(ser)
            last_dir_sent = 0
            last_stop_sent = True
            mode = "manual"
            last_manual_time = time.time()
            manual_stopped = True
            print("手动停止")

        elif key == ord('z') or key == ord('Z'):
            print("调零开始...")
            send_homing(ser)
            mode = "manual"
            last_manual_time = time.time()
            manual_stopped = False
            time.sleep(HOMING_TIMEOUT)
            send_stop(ser)
            last_dir_sent = 0
            last_stop_sent = False
            manual_stopped = True
            mode = "manual"
            print("调零结束（等待超时），请检查限位是否到位")
            last_manual_time = time.time()

        # 速度档位（仅影响手动）
        elif ord('1') <= key <= ord('9'):
            SPEED_LEVEL = key - ord('0')
            print(f"手动速度档位 -> {SPEED_LEVEL}")
        elif key == ord('0'):
            SPEED_LEVEL = 10
            print(f"手动速度档位 -> {SPEED_LEVEL}")

        # ====== 模式切换 ======
        if mode == "manual":
            if (time.time() - last_manual_time > AUTO_IDLE_TIME) and manual_stopped:
                mode = "auto"
                print("进入自动对准模式（速度锁定3档）")
                last_dir_sent = 0
                last_stop_sent = False
        else:
            if frame_count % 5 == 0:
                centered = auto_align_update(ser, cap)
                if centered:
                    pass

        # ====== 画面显示（手动模式） ======
        if mode == "manual" and not estop:
            ret, frame = cap.read()
            if ret:
                target = detect_red_target(frame)
                draw_debug(frame, target, 0, "Manual", False, SPEED_LEVEL)
                cv2.imshow("Visual Servo", frame)

        time.sleep(0.02)

    cap.release()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()
    print("程序已退出。")

if __name__ == "__main__":
    main()