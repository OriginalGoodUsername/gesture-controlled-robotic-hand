import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions
import serial
import serial.tools.list_ports
import time
import math
import ctypes
from pathlib import Path

# Windows arrow-key state for manual servo adjustment.
_user32 = ctypes.windll.user32
VK_LEFT, VK_UP, VK_RIGHT, VK_DOWN = 0x25, 0x26, 0x27, 0x28
_vk_prev    = {k: False for k in (VK_LEFT, VK_UP, VK_RIGHT, VK_DOWN)}
_last_lr_t  = 0.0
LR_REPEAT_S = 0.08  # seconds between repeats while left/right held

def _vk_down(vk):
    return bool(_user32.GetAsyncKeyState(vk) & 0x8000)

# MediaPipe landmark detector.
options = HandLandmarkerOptions(
    base_options=python.BaseOptions(model_asset_path=str(Path(__file__).resolve().parents[1] / 'hand_landmarker.task')),
    num_hands=1
)
landmarker = HandLandmarker.create_from_options(options)

# Serial communication with Arduino
arduino = None
available_ports = [port.device for port in serial.tools.list_ports.comports()]
print("Available COM ports:", available_ports)

try:
    arduino = serial.Serial('COM3', 9600)
    time.sleep(2)
    print("Connected to Arduino on COM3")
except serial.SerialException as e:
    print(f"Warning: Could not connect to Arduino on COM3: {e}")
    print(f"Available ports: {available_ports}")
    print("Running in DEMO mode without servo control...\n")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17)
]

# Calibrated positions from serial monitor testing
# order: [wrist, pinky, ring, middle, index, thumb]
OPEN_ANGLES   = [90, 0,  0,  180, 180, 90]
CLOSED_ANGLES = [90, 180, 180, 0,  0,   0]

manual_mode_servo = False
selected_servo = 0
manual_angles = list(OPEN_ANGLES)
servo_names = ["Wrist", "Pinky", "Ring", "Middle", "Index", "Thumb"]

# Smoothing: EMA keeps angles stable when detection is noisy
SMOOTH      = 0.15   # 0 = frozen, 1 = raw — lower = smoother/slower
MAX_STEP    = 12     # maximum commanded angle change per frame
SNAP_THRESH = 10      # snap to 0 or 180 when within this many degrees (removes endpoint jitter)
HOLD_FRAMES = 15     # frames to hold last position when hand disappears

smoothed_angles    = list(OPEN_ANGLES)
last_valid_angles  = list(OPEN_ANGLES)
last_sent_angles   = list(OPEN_ANGLES)
frames_no_hand     = 0

LM_SMOOTH  = 0.55   # EMA for drawn landmark positions — lower = steadier, higher = closer to hand
smoothed_lm = None  # list of [x, y] updated each frame


def draw_hand_mesh(image, lm_xy):
    """lm_xy: list of [x, y] in normalised 0-1 coords (pre-smoothed)."""
    h, w = image.shape[:2]
    for s, e in HAND_CONNECTIONS:
        cv2.line(image,
                 (int(lm_xy[s][0] * w), int(lm_xy[s][1] * h)),
                 (int(lm_xy[e][0] * w), int(lm_xy[e][1] * h)),
                 (0, 255, 0), 2)
    for x, y in lm_xy:
        cv2.circle(image, (int(x * w), int(y * h)), 5, (0, 0, 255), -1)


def get_finger_angles(hand_landmarks):
    """
    Returns [wrist, pinky, ring, middle, index, thumb] in servo angle space.

    Finger curl uses tip-to-wrist distance relative to knuckle-to-wrist
    distance, scaled between the calibrated open and closed values.
    Thumb position uses its distance from the index knuckle relative to palm
    size, plus a cross-product check for crossing the knuckle line.
    """

    def dist(a, b):
        return ((a.x - b.x)**2 + (a.y - b.y)**2) ** 0.5

    wrist = hand_landmarks[0]

    def finger_curl(tip_idx, mcp_idx):
        tip_d = dist(hand_landmarks[tip_idx], wrist)
        mcp_d = dist(hand_landmarks[mcp_idx], wrist)
        if mcp_d < 0.01:
            return 0.0
        ratio = tip_d / mcp_d
        raw = max(0.0, min(1.0, (2.5 - ratio) / 2.0))
        # Map calibrated raw values 0.39 (open) and 0.83 (closed) to 0..1.
        return max(0.0, min(1.0, (raw - 0.39) / 0.44))

    def thumb_abduction():
        wrist_lm  = hand_landmarks[0]
        index_mcp = hand_landmarks[5]
        pinky_mcp = hand_landmarks[17]
        thumb_tip = hand_landmarks[4]
        mid_mcp   = hand_landmarks[9]

        palm_size = dist(wrist_lm, mid_mcp)
        if palm_size < 0.001:
            return 0.0

        # Detect thumb crossing behind the palm via knuckle-line cross product.
        # The wrist is always on the "outside" (thumb side) of the knuckle line.
        # If the thumb tip crosses to the opposite side, it is behind the palm = closed.
        kx = pinky_mcp.x - index_mcp.x
        ky = pinky_mcp.y - index_mcp.y
        thumb_sign = kx * (thumb_tip.y - index_mcp.y) - ky * (thumb_tip.x - index_mcp.x)
        wrist_sign = kx * (wrist_lm.y  - index_mcp.y) - ky * (wrist_lm.x  - index_mcp.x)
        if (thumb_sign > 0) != (wrist_sign > 0):
            return 1.0  # thumb crossed into palm → fully closed

        # Side-to-side abduction by distance
        ratio = dist(thumb_tip, index_mcp) / palm_size
        # open (curl=0) at ratio ~0.46, closed (curl=1) at ratio ~0.36
        return max(0.0, min(1.0, (0.46 - ratio) / 0.10))

    def lerp(curl_val, ch):
        return int(OPEN_ANGLES[ch] + curl_val * (CLOSED_ANGLES[ch] - OPEN_ANGLES[ch]))

    wrist_angle = max(0, min(180, int(90 - (wrist.x - 0.5) * 180)))

    return [
        wrist_angle,
        lerp(finger_curl(20, 17), 1),  # pinky
        lerp(finger_curl(16, 13), 2),  # ring
        lerp(finger_curl(12,  9), 3),  # middle
        lerp(finger_curl( 8,  5), 4),  # index
        lerp(thumb_abduction(),    5),  # thumb
    ]


def send_servo_angles(angles):
    """
    Send angles directly to Arduino. Angles are already in servo space:
    [wrist, pinky, ring, middle, index, thumb] → channels [0,1,2,3,4,5]
    """
    if arduino is None:
        return
    data = ','.join(map(str, angles)) + '\n'
    arduino.write(data.encode())


def draw_ui(image, angles):
    h, w = image.shape[:2]
    cv2.rectangle(image, (w-400, 0), (w, 210), (0, 0, 0), -1)

    mode_text = "MANUAL Mode" if manual_mode_servo else "AUTO Mode"
    mode_color = (0, 255, 255) if manual_mode_servo else (0, 255, 0)
    cv2.putText(image, mode_text, (w-390, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)

    cv2.putText(image, "Servo Angles:", (w-390, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    for i, label in enumerate(servo_names):
        color = (0, 255, 255) if (manual_mode_servo and selected_servo == i) else (255, 255, 255)
        cv2.putText(image, f"{i} {label}: {angles[i]}", (w-390, 90 + i*18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    cv2.putText(image, "M: Manual/Auto | Q: Quit",
                (10, h-40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.putText(image, "0-5: Select servo | Arrows: left/right=-/+5  up/down=prev/next",
                (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.putText(image, "by Vraj Patel", (w-390, 205),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)


while cap.isOpened():
    success, image = cap.read()
    if not success:
        break

    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

    result = landmarker.detect(mp_image)
    if result.hand_landmarks:
        frames_no_hand = 0
        for hand_landmarks in result.hand_landmarks:
            # Smooth landmark positions for a steady mesh
            if smoothed_lm is None:
                smoothed_lm = [[lm.x, lm.y] for lm in hand_landmarks]
            else:
                for i, lm in enumerate(hand_landmarks):
                    smoothed_lm[i][0] = LM_SMOOTH * lm.x + (1 - LM_SMOOTH) * smoothed_lm[i][0]
                    smoothed_lm[i][1] = LM_SMOOTH * lm.y + (1 - LM_SMOOTH) * smoothed_lm[i][1]
            draw_hand_mesh(image, smoothed_lm)
            raw = list(get_finger_angles(hand_landmarks))
            for i in range(6):
                target = SMOOTH * raw[i] + (1 - SMOOTH) * smoothed_angles[i]
                step   = target - smoothed_angles[i]
                if abs(step) > MAX_STEP:
                    step = MAX_STEP if step > 0 else -MAX_STEP
                val = smoothed_angles[i] + step
                if val <= SNAP_THRESH:
                    val = 0
                elif val >= 180 - SNAP_THRESH:
                    val = 180
                smoothed_angles[i] = int(val)
            last_valid_angles[:] = smoothed_angles
            break
        current_angles = smoothed_angles.copy()
    else:
        frames_no_hand += 1
        if frames_no_hand <= HOLD_FRAMES:
            current_angles = last_valid_angles.copy()
            if smoothed_lm is not None:
                draw_hand_mesh(image, smoothed_lm)  # hold last mesh while briefly lost
        else:
            # Return toward the open pose after the tracking hold expires.
            for i in range(6):
                smoothed_angles[i] = int(0.1 * OPEN_ANGLES[i] + 0.9 * smoothed_angles[i])
            current_angles = smoothed_angles.copy()

    if manual_mode_servo:
        current_angles = manual_angles.copy()

    if any(abs(current_angles[i] - last_sent_angles[i]) > 1 for i in range(6)):
        send_servo_angles(current_angles)
        last_sent_angles[:] = current_angles
    draw_ui(image, current_angles)

    cv2.imshow('Hand Tracking', image)

    key = cv2.waitKey(5)
    char = key & 0xFF if key != -1 else 0

    if char == ord('q') or char == ord('Q'):
        break
    elif char == ord('m') or char == ord('M'):
        manual_mode_servo = not manual_mode_servo
    elif ord('0') <= char <= ord('5'):
        selected_servo = int(chr(char))

    # Arrow keys via GetAsyncKeyState (polls Windows key state directly)
    _now = time.time()
    for _vk in (VK_UP, VK_DOWN, VK_LEFT, VK_RIGHT):
        _cur = _vk_down(_vk)
        _rising = _cur and not _vk_prev[_vk]
        _vk_prev[_vk] = _cur
        if _vk == VK_UP and _rising:
            selected_servo = (selected_servo - 1) % 6
        elif _vk == VK_DOWN and _rising:
            selected_servo = (selected_servo + 1) % 6
        elif _vk in (VK_LEFT, VK_RIGHT):
            if _rising or (_cur and _now - _last_lr_t >= LR_REPEAT_S):
                if manual_mode_servo:
                    if _vk == VK_LEFT:
                        manual_angles[selected_servo] = max(0, manual_angles[selected_servo] - 5)
                    else:
                        manual_angles[selected_servo] = min(180, manual_angles[selected_servo] + 5)
                _last_lr_t = _now

cap.release()
cv2.destroyAllWindows()
landmarker.close()