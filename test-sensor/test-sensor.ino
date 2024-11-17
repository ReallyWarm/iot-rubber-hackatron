#include "LowPower.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
 
#define BAT_PIN A0
#define MOI_PIN A1
#define S51_RE 11
#define S51_DE 10
#define S51_RO 5
#define S51_DI 6
#define SWITCH_SENSOR 7
#define SEALEVELPRESSURE_HPA (1013.25)

bool debug = true;
 
const byte npkQuery[]= {0x01, 0x03, 0x00, 0x04, 0x00, 0x03, 0x44, 0x0a};
const byte phQuery[]= {0x01, 0x03, 0x00, 0x03, 0x00, 0x01, 0x74, 0x0a};
const byte cdQuery[]= {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xca};

uint16_t battery = 0, moisture = 0, conductivity = 0, nitrogen = 0, phosphorus = 0, potassium = 0;
float pHValue = 0.0, temperature = 0.0, humidity = 0.0, pressure = 0.0;
SoftwareSerial mod(S51_RO, S51_DI);
// Adafruit_BME680 bme(&Wire);

uint8_t deviceID = 1;

void setup() {
  Serial.begin(9600);
  resetSensor();
  Serial.println("Starting...");

  if (debug) {
    String message = 
      "{\"command\":\"DATA\",\"device\":"+String(deviceID)+",\"batt\":"+String(battery)+
      ",\"data\":{\"tmp\":"+String(26.5)+",\"hmd\":"+String(55)+",\"prs\":"+String(69)+
      ",\"moi\":"+String(95)+",\"pH\":"+String(7.1)+",\"cnd\":"+String(360)+
      ",\"ntg\":"+String(56)+",\"php\":"+String(124)+",\"pts\":"+String(89)+"}}";
    Serial.println("Sending STR: " + message);
  }

  delay(1000);
}

void loop() {
  uint8_t getSensorSec = 9, sendDataSec = 6;
  moisture = 0, conductivity = 0, nitrogen = 0, phosphorus = 0, potassium = 0;
  pHValue = 0.0, temperature = 0.0, humidity = 0.0, pressure = 0.0;
  uint16_t moi = 0, cnd = 0, ntg = 0, php = 0, pts = 0;
  float ph = 0.0, tmp = 0.0, hmd = 0.0, prs = 0.0;
  for (int i = 0; i < 5; i++) {
    if (debug) Serial.print("Iteration: ");
    if (debug) Serial.println(i+1);
    startSwitch();
    startSensor();
    delay(5000);
    getMoisture(&moi);
    delay(200);
    getConduct(&cnd);
    delay(200);
    getNPK(&ntg, &php, &pts);
    delay(200);
    getPH(&ph);
    delay(200);
    // getBME(&tmp, &hmd, &prs);
    // delay(200);
    moisture += moi;
    conductivity += cnd;
    nitrogen += ntg;
    phosphorus += php;
    potassium += pts;
    pHValue += ph;
    temperature += tmp;
    humidity += hmd;
    pressure += prs;
    resetSensor();
    endSwitch();
    if (debug) Serial.println("Going short sleep");
    delay(1000);
    longSleep((15-getSensorSec) * 0.9);
  }
  startSwitch();
  getBattery();
  delay(200);
  endSwitch();

  String message = 
      "{\"command\":\"DATA\",\"device\":"+String(deviceID)+",\"batt\":"+String(battery)+
      ",\"data\":{\"tmp\":"+String(temperature / 5.0)+",\"hmd\":"+String(humidity / 5.0)+",\"prs\":"+String(pressure / 5.0)+
      ",\"moi\":"+String(moisture / 5.0)+",\"pH\":"+String(pHValue / 5.0)+",\"cnd\":"+String(conductivity / 5.0)+
      ",\"ntg\":"+String(nitrogen / 5.0)+",\"php\":"+String(phosphorus / 5.0)+",\"pts\":"+String(potassium / 5.0)+"}}";
  Serial.println("Sending STR: " + message);
  resetSensor();
  if (debug) Serial.println("Going long sleep");
  delay(3000);
  longSleep((30-sendDataSec) * 0.9);
}

void startSwitch() {
  pinMode(SWITCH_SENSOR, OUTPUT);
  digitalWrite(SWITCH_SENSOR, HIGH);
  delay(1000);
}

void endSwitch() {
  digitalWrite(SWITCH_SENSOR, LOW);
  pinMode(SWITCH_SENSOR, INPUT);
  delay(1000);
}

void startSensor() {
  Wire.begin();
  mod.begin(4800);
  pinMode(S51_RE, OUTPUT);
  pinMode(S51_DE, OUTPUT);
  // bool bmeFound = bme.begin();
  // if (!bmeFound && debug) Serial.println("Could not find a valid BME680 sensor, check wiring!");
  // bme.setTemperatureOversampling(BME680_OS_8X);
  // bme.setHumidityOversampling(BME680_OS_2X);
  // bme.setPressureOversampling(BME680_OS_4X);
  // bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  // bme.setGasHeater(320, 150);
  while (mod.available()) { 
    mod.read(); 
  }
}

void resetSensor() {
  Wire.end();
  mod.end();
  pinMode(S51_RE, INPUT);
  pinMode(S51_DE, INPUT);
}

void getBattery() {
  battery = analogRead(BAT_PIN);
  if (debug) {
    Serial.print("Battery: ");
    Serial.println(battery);
  }
}

void getMoisture(uint16_t* moisture) {
  *moisture = analogRead(MOI_PIN);
  if (debug) {
    Serial.print("Moisture: ");
    Serial.println(*moisture);
  }
}

void getNPK(uint16_t* nitrogen, uint16_t* phosphorus, uint16_t* potassium) {
  byte npkResp[11] = { 0 };
  digitalWrite(S51_DE,HIGH);
  digitalWrite(S51_RE,HIGH);
  mod.write(npkQuery, sizeof(npkQuery));
  digitalWrite(S51_DE,LOW);
  digitalWrite(S51_RE,LOW);
  delay(200);

  if (mod.available() >= sizeof(npkResp)) { 
    mod.readBytes(npkResp, sizeof(npkResp)); 

    if (debug) {
      for (int i = 0; i < sizeof(npkResp); i++) {
        Serial.print(npkResp[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    *nitrogen = (npkResp[3] << 8) | npkResp[4];
    *phosphorus = (npkResp[5] << 8) | npkResp[6];
    *potassium = (npkResp[7] << 8) | npkResp[8];

    if (debug) {
      Serial.print("Nitrogen: ");
      Serial.println(*nitrogen);
      Serial.print("Phosphorus: ");
      Serial.println(*phosphorus);
      Serial.print("Potassium: ");
      Serial.println(*potassium);
    }
  }
}

void getPH(float* pH) {
  byte phResp[7] = { 0 };
  digitalWrite(S51_DE,HIGH);
  digitalWrite(S51_RE,HIGH);
  mod.write(phQuery, sizeof(phQuery));
  digitalWrite(S51_DE,LOW);
  digitalWrite(S51_RE,LOW);
  delay(200);

  if (mod.available() >= sizeof(phResp)) { 
    mod.readBytes(phResp, sizeof(phResp)); 

    if (debug) {
      for (int i = 0; i < sizeof(phResp); i++) {
        Serial.print(phResp[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    uint16_t phValue = (phResp[3] << 8) | phResp[4];
    *pH = phValue / 10.0;

    if (debug) {
      Serial.print("pH: ");
      Serial.println(*pH);
    }
  }
}

void getConduct(uint16_t* conductivity) {
  byte cdResp[7] = { 0 };
  digitalWrite(S51_DE,HIGH);
  digitalWrite(S51_RE,HIGH);
  mod.write(cdQuery, sizeof(cdQuery));
  digitalWrite(S51_DE,LOW);
  digitalWrite(S51_RE,LOW);
  delay(200);

  if (mod.available() >= sizeof(cdResp)) { 
    mod.readBytes(cdResp, sizeof(cdResp)); 

    if (debug) {
      for (int i = 0; i < sizeof(cdResp); i++) {
        Serial.print(cdResp[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }

    *conductivity = (cdResp[3] << 8) | cdResp[4];

    if (debug) {
      Serial.print("Conductivity: ");
      Serial.println(*conductivity);
    }
  }
}

void getBME(float* temperature, float* humidity, float* pressure) {
  bme.performReading();
  *temperature = bme.temperature;
  *humidity = bme.humidity;
  *pressure = bme.pressure / 100.0;

  if (debug) {
    Serial.print("Temperature: ");
    Serial.print(*temperature);
    Serial.println(" *C");
    Serial.print("Humidity: ");
    Serial.print(*humidity);
    Serial.println(" %");
    Serial.print("Pressure: ");
    Serial.print(*pressure);
    Serial.println(" hPa");
  }
}

void longSleep( uint32_t sleepInSeconds )
{
  if ( sleepInSeconds & 0x01 ) LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  if ( sleepInSeconds & 0x02 ) LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  if ( sleepInSeconds & 0x04 ) LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);

  while ( sleepInSeconds & 0xFFFFFFF8 ) {
    sleepInSeconds = sleepInSeconds - 8;
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}
