#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// Create an instance of the BME680
Adafruit_BME680 bme;

// Adjust this for your altitude (in meters) for accurate pressure readings
#define SEA_LEVEL_PRESSURE_HPA (1013.25)

void setup() {
  Serial.begin(9600);  // Initialize serial for debugging
  Serial.println("BME680 test");

  // Initialize I2C communication with the BME680
  if (!bme.begin(0x76)) { // 0x76 is the default I2C address; use 0x77 if necessary
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Configure the sensor settings
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
}

void loop() {
  // Trigger a measurement
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading");
    return;
  }

  // Print the sensor readings
  Serial.print("Temperature: ");
  Serial.print(bme.temperature);
  Serial.println(" °C");

  Serial.print("Humidity: ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Pressure: ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Gas Resistance: ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" kOhms");

  Serial.print("Approx. Altitude: ");
  Serial.print(bme.readAltitude(SEA_LEVEL_PRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000); // Delay between readings
}