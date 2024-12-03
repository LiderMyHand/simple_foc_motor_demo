#include <Arduino.h>

#include <SimpleFOC.h>

// Define the SPI pins for the encoder
#define ENCODER_CS 15  // Chip Select pin
#define ENCODER_CLK 18 // Clock pin
#define ENCODER_MISO 23 // MISO pin
#define ENCODER_MOSI 19 // MOSI pin

// MagneticSensorSPI object for the AS5147 encoder
MagneticSensorSPI sensor(AS5147_SPI, ENCODER_CS);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial) {}

  // Initialize SPI with custom pin mapping
  SPI.begin(ENCODER_CLK, ENCODER_MISO, ENCODER_MOSI, ENCODER_CS);

  // Initialize the encoder
  sensor.init();

  // Check if the encoder was successfully initialized
  if (sensor.getAngle() != 0) {
    Serial.println("Encoder initialized successfully!");
  } else {
    Serial.println("Failed to initialize encoder!");
  }
}

void loop() {
  // Update the encoder to get the latest data
  sensor.update();

  // Read and print the encoder angle
  float angle = sensor.getAngle();
  Serial.print("Angle: ");
  Serial.println(angle);

  // Small delay for readability
  delay(100);
}
