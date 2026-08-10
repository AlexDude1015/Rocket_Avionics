#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_CS 2

Adafruit_BMP280 bmp(BMP_CS);
float P_ground = 0;

void setup() {
  Serial.begin(9600);
  if (!bmp.begin()) {
    Serial.println("BMP280 not found — check wiring");
    while (1);
  }

  // Capture ground reference — do this once, at startup, while stationary
  for (int i = 0; i < 20; i++) {
    P_ground += bmp.readPressure() / 100.0;  // Pa -> hPa
    delay(100);
  }
  P_ground /= 20;

  Serial.print("Ground pressure: ");
  
  Serial.println(P_ground);
}

void loop() {
  float altitude = bmp.readAltitude(P_ground);
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" m");
  delay(1000);
}