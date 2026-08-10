#include <SPI.h>
#include <SD.h>

// Change this to match the CS pin you wired to the SD adapter
const int chipSelect = 4;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect (needed for some boards)
  }

  Serial.println("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("Card initialized.");

  // Try writing a test file
  File testFile = SD.open("test.txt", FILE_WRITE);
  if (testFile) {
    testFile.println("Hello, SD card!");
    testFile.close();
    Serial.println("Wrote to test.txt");
  } else {
    Serial.println("Error opening test.txt for writing");
  }

  // Read it back
  testFile = SD.open("test.txt");
  if (testFile) {
    Serial.println("Reading test.txt:");
    while (testFile.available()) {
      Serial.write(testFile.read());
    }
    testFile.close();
  } else {
    Serial.println("Error opening test.txt for reading");
  }
}

void loop() {
  // nothing to do here
}