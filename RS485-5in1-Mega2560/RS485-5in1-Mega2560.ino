#include <Wire.h>
 
#define RE 11
#define DE 10
 
const byte npkQuery[]= {0x01, 0x03, 0x00, 0x04, 0x00, 0x03, 0x44, 0x0a};
const byte phQuery[]= {0x01, 0x03, 0x00, 0x03, 0x00, 0x01, 0x74, 0x0a};
const byte cdQuery[]= {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xca};

byte values[11];
 
void setup() {
  Serial.begin(9600);
  Serial1.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  Serial.println("Start");
  delay(1000);
}
 
void loop() {
  all();
  delay(1000);
  pNPK();
  delay(1000);
  pPH();
  delay(1000);
  pCD();
  delay(1000);
}

void sendQuery(const byte* query, size_t querySize) {
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  Serial1.write(query, querySize);
  Serial1.flush();
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
}

void receiveSoilSensor(byte* resp, size_t respSize) {
  unsigned long start = millis();
  size_t index = 0;

  while ((millis() - start >= 0) && (millis() - start < 1000) && (index < respSize)) {
    if (Serial1.available()) {
      resp[index++] = Serial1.read();
    }
  }
}

void printResponseByte(byte* resp, size_t respSize) {
  for (int i = 0; i < respSize; i++) {
    Serial.print(resp[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void pNPK() {
  Serial.println("npk");
  byte npkResp[11] = { 0 };
  sendQuery(npkQuery, sizeof(npkQuery));

  Serial.println("get npk");
  receiveSoilSensor(npkResp, sizeof(npkResp)); 
  if (npkResp[0] > 0) { 
    printResponseByte(npkResp, sizeof(npkResp));

    uint16_t nitrogen = (npkResp[3] << 8) | npkResp[4];
    uint16_t phosphorus = (npkResp[5] << 8) | npkResp[6];
    uint16_t potassium = (npkResp[7] << 8) | npkResp[8];

    Serial.print("Nitrogen: ");
    Serial.println(nitrogen);
    Serial.print("Phosphorus: ");
    Serial.println(phosphorus);
    Serial.print("Potassium: ");
    Serial.println(potassium);
    }
}

void pPH() {
  Serial.println("ph");
  byte phResp[7] = { 0 };
  sendQuery(phQuery, sizeof(phQuery));

  Serial.println("get ph");
  receiveSoilSensor(phResp, sizeof(phResp)); 
  if (phResp[0] > 0) { 
    printResponseByte(phQuery, sizeof(phResp));

    uint16_t phValue = (phResp[3] << 8) | phResp[4];
    float pH = phValue / 10.0;

    Serial.print("pH: ");
    Serial.println(pH);
    }
}

void pCD() {
  Serial.println("cd");
  byte cdResp[7] = { 0 };
  sendQuery(cdQuery, sizeof(cdQuery));

  Serial.println("get cd");
  receiveSoilSensor(cdResp, sizeof(cdResp)); 
  if (cdResp[0] > 0) { 
    printResponseByte(cdResp, sizeof(cdResp));

    uint16_t cdValue = (cdResp[3] << 8) | cdResp[4];

    Serial.print("Conductivity: ");
    Serial.println(cdValue);
    }
}
void all() {
  const byte allQuery[]= {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
  Serial.println("all");
  byte allResp[19] = { 0 };
  sendQuery(allQuery, sizeof(allQuery));

  Serial.println("get all");
  receiveSoilSensor(allResp, sizeof(allResp)); 
  if (allResp[0] > 0) { 
    printResponseByte(allResp, sizeof(allResp));
  }
}