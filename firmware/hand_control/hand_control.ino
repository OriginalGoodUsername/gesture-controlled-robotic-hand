#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50

// Servo channels (PCA9685 channel number = physical servo)
#define WRIST_SERVO  0
#define PINKY_SERVO  1
#define RING_SERVO   2
#define MIDDLE_SERVO 3
#define INDEX_SERVO  4
#define THUMB_SERVO  5

// Per-channel pulse limits (4096 ticks at 50 Hz = 20 ms period)
// ch 0-4: calibrated MG996R pulse counts
// ch 5:   micro   → tighter range to prevent stall/overheat
const uint16_t SERVO_MIN[6] = {80, 80, 80, 80, 80, 175};
const uint16_t SERVO_MAX[6] = {520, 520, 520, 520, 520, 480};

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int angles[6];
    int index = 0;
    char* ptr = strtok((char*)data.c_str(), ",");
    while (ptr != NULL && index < 6) {
      angles[index++] = atoi(ptr);
      ptr = strtok(NULL, ",");
    }

    if (index == 6) {
      setServoAngle(WRIST_SERVO,  angles[0]);
      setServoAngle(PINKY_SERVO,  angles[1]);
      setServoAngle(RING_SERVO,   angles[2]);
      setServoAngle(MIDDLE_SERVO, angles[3]);
      setServoAngle(INDEX_SERVO,  angles[4]);
      setServoAngle(THUMB_SERVO,  angles[5]);
    }
  }
}

void setServoAngle(uint8_t servo, int angle) {
  angle = constrain(angle, 0, 180);
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN[servo], SERVO_MAX[servo]);
  pwm.setPWM(servo, 0, pulse);
}