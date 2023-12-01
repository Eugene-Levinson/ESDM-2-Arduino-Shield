#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

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
Adafruit_MMA8451 mma = Adafruit_MMA8451();

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

  mma.begin();
  mma.setRange(MMA8451_RANGE_8_G);
}

void loop() {

  // Read light sensor value
  LightSensorValue = analogRead(TSL257_PIN);

  // Read the HIH6120 sensor
  HIH6120data HumTempData = readHIH6120();

  // Read the accelerometer
  sensors_event_t event;
  mma.getEvent(&event);
  uint8_t orientation = mma.getOrientation();

  // Set LED states
  // If humidity is greater than 60%, turn on LED 1
  if (HumTempData.humidity > 60) {
    digitalWrite(LED_1, HIGH);
  } else {
    digitalWrite(LED_1, LOW);
  }

  // If temperature is greater than 25C, turn on LED 2
  if (HumTempData.temperature > 25) {
    digitalWrite(LED_2, HIGH);
  } else {
    digitalWrite(LED_2, LOW);
  }

  // If its facing up, turn on LED 3 (if z acceleration is between 8 and 11)
  if (event.acceleration.z > 9.7) {
    digitalWrite(LED_3, HIGH);
  } else {
    digitalWrite(LED_3, LOW);
  }


  // Print the data to the serial monitor
  Serial.print("Humidity: ");
  Serial.print(HumTempData.humidity);
  Serial.print("%, Temperature: ");
  Serial.print(HumTempData.temperature);
  Serial.print("C");
  Serial.print(", Light: ");
  Serial.print(LightSensorValue);
  Serial.print(", X: ");
  Serial.print(event.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(event.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(event.acceleration.z);
  Serial.print(", Orientation: ");
  Serial.println(orientation);
  

  // Wait 100ms
  delay(1000);
}