// Reads distance from a VL53L0X with no library install.
// The sensor code is in the VL53L0X_Direct.h tab of this sketch.
#include <Wire.h>
#include "VL53L0X_Direct.h"
VL53L0X_Direct sensor;
void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(21, 22);           
  Wire.beginTransmission(0x29);
  if (Wire.endTransmission() != 0) {
    Serial.println("Nothing at I2C address 0x29 - check wiring and power");
    while (true) delay(1000);
  }
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Sensor answered but setup failed - unplug and retry");
    while (true) delay(1000);
  }
  sensor.startContinuous();
  Serial.println("VL53L0X ready");
}
void loop() {
  uint16_t mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || mm > 8000) {
    Serial.println("out of range");
  } else {
    Serial.print(mm);
    Serial.println(" mm");
  }
}
