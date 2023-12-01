#include <Arduino.h>
#include <Wire.h>

// Pin definitions
#define LED_1 4
#define LED_2 5
#define LED_3 6

#define TSL257_PIN A0

// Sensor addresses
#define HIH6120address 0x27

// Sensor data types
struct HIH6120data {
  float humidity;
  float temperature;
};


// Variables
int LightSensorValue;

// Function declarations
HIH6120data readHIH6120() {

  // Intantiate the data structure
  HIH6120data HumTempData;
  uint8_t sensorBytes[4];

  // Send the command to read the sensor (Wake up the sensor)
  Wire.beginTransmission(HIH6120address);
  Wire.write(0x00);
  Wire.endTransmission();

  // // Wait for the sensor to respond
  delay(100);

  // Request 4 bytes of data from the sensor
  Wire.requestFrom(HIH6120address, 4);

  // Read the data from the sensor
  for (int i = 0; i < 4; i++) {
    sensorBytes[i] = Wire.read();
  }

  // Convert the data to the correct format (most significant byte first)
  // For humidity, add the conversion factor
  HumTempData.humidity = (((sensorBytes[0] << 8) | sensorBytes[1]))/100.0;
  HumTempData.temperature = ((sensorBytes[2] << 8) | sensorBytes[3])/1000.0;

  // Return the status byte
  return  HumTempData;
}

void setup() {
  // Set LED pins as outputs
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);

  // Set TSL257 pin as input
  pinMode(TSL257_PIN, INPUT);

  // Set Serial baud rate
  Serial.begin(9600);

  // Start the I2C bus and enable pull-up resistors
  Wire.begin();
}

void loop() {

  // Read the HIH6120 sensor
  HIH6120data HumTempData = readHIH6120();

  // Print the data to the serial monitor
  Serial.print("Humidity: ");
  Serial.print(HumTempData.humidity);
  Serial.print("%, Temperature: ");
  Serial.print(HumTempData.temperature);
  Serial.println("C");


  // Wait 100ms
  delay(1000);
}