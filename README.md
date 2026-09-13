# Gesture-Controlled Robotic Hand

Webcam hand tracking drives five finger servos and a wrist servo through Python, serial, and an Arduino/PCA9685 controller.

## How it works

MediaPipe supplies the hand landmarks. Python maps finger-curl and thumb-position measurements to calibrated servo angles, smooths the commands, and sends six comma-separated angles to the Arduino. The Arduino maps each angle to its channel's PWM limits.

The controller includes a manual adjustment mode, a per-frame movement limit, and a short hold of the last pose when tracking disappears before returning toward the open pose. These are control rules in the source, not measurements of tracking accuracy or latency.

## Setup

1. Use Windows: the arrow-key controls use `GetAsyncKeyState` through `ctypes`.
2. Install dependencies: `python -m pip install -r requirements.txt`.
3. Download the Hand Landmarker model from the [official MediaPipe guide](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker/index#models) and save it as `hand_landmarker.task` in the repository root. The model is provided by Google and is not bundled here.
4. Install the Adafruit PWM Servo Driver library in Arduino IDE, then upload `firmware/hand_control/hand_control.ino` to the connected Arduino.
5. Set the serial port in `src/main.py` and `src/servo_test.py` to match the Arduino; both currently use `COM3` at 9600 baud. The camera index is `0`.
6. Run `python src/main.py` from the repository root.

If the serial connection fails, the camera display can run without servo output. The camera and model are still required.

## Controls and wiring

`M` toggles automatic/manual mode; `0` through `5` select a servo; left/right arrows adjust its angle; up/down arrows change selection; `Q` exits.

| PCA9685 channel | Actuator |
| --- | --- |
| 0 | Wrist |
| 1 | Pinky |
| 2 | Ring |
| 3 | Middle |
| 4 | Index |
| 5 | Thumb |

`OPEN_ANGLES`, `CLOSED_ANGLES`, and the firmware pulse limits are specific to this build. Check the linkage travel and power wiring before running the sweep utility. The thumb has a separate pulse range. The firmware retains the last commanded pose if serial input stops; it does not implement a serial timeout.

## Files and validation

`src/servo_test.py` alternates open and closed poses to check the serial/hardware path. `tools/i2c_scan/` scans for the PCA9685. The main sketch was selected over the nested experimental `test` sketch and patch file.

Python syntax and comment-only equivalence were checked during publication. The one functional packaging change replaces a machine-specific model path with a path relative to this repository. Camera, serial, and servo operation were not re-tested during publication. Photos and a demonstration can be added under `media/`.
