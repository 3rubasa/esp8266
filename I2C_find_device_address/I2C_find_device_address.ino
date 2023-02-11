#include <Wire.h>

#define SERIAL_BAUD_RATE 115200

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setDebugOutput(true);

  while (!Serial) {

  }
}

void loop() {
  Serial.println("I2C scanning...");
  byte count = 0;

  Wire.begin();

  for (byte i=0; i <127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found address: 0x");
      Serial.println(i, HEX);
      count++;
      delay(1);
    }
  }

  Serial.print("Done, found ");
  Serial.print(count, DEC);
  Serial.println(" devices");

  delay(2000);
}
