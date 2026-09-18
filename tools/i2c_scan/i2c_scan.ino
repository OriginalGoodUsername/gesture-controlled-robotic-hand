/*
  I2C address scanner. Open Serial Monitor at 9600 baud.
  The hand firmware expects the PCA9685 at 0x40. If its address differs,
  check the address jumpers and update the driver address to match.
  No response: check SDA, SCL, power, and ground.
*/
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  Wire.begin();
  Serial.println("\nI2C scanner ready.");
}

void loop() {
  byte count = 0;
  Serial.println("Scanning...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("  No I2C devices found.");
  Serial.println("done.\n");
  delay(3000);
}
