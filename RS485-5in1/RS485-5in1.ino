#include <SoftwareSerial.h>
#include <Wire.h>
 
#define RE 11
#define DE 10
#define RO 5
#define DI 6
 
const byte npkQuery[]= {0x01, 0x03, 0x00, 0x04, 0x00, 0x03, 0x44, 0x0a};
const byte phQuery[]= {0x01, 0x03, 0x00, 0x03, 0x00, 0x01, 0x74, 0x0a};
const byte cdQuery[]= {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xca};

byte values[11];
SoftwareSerial mod(RO,DI);
 
void setup() {
  Serial.begin(9600);
  mod.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  Serial.println("Start");
  delay(1000);
}
 
void loop() {
  Serial.println("t1");
  pNPK();
  delay(1000);
  Serial.println("t2");
  pPH();
  delay(1000);
  pCD();
  delay(1000);
}

void pNPK() {
  Serial.println("npk");
  byte npkResp[11] = { 0 };
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  mod.write(npkQuery, sizeof(npkQuery));
  digitalWrite(DE,LOW);
  digitalWrite(RE,LOW);
  delay(100);

  Serial.println("get npk");
  if (mod.available() >= sizeof(npkResp)) { 
    mod.readBytes(npkResp, sizeof(npkResp)); 

    for (int i = 0; i < sizeof(npkResp); i++) {
      Serial.print(npkResp[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

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
  byte phResp[7] = { 0 };
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  mod.write(phQuery, sizeof(phQuery));
  digitalWrite(DE,LOW);
  digitalWrite(RE,LOW);
  delay(100);

  if (mod.available() >= sizeof(phResp)) { 
    mod.readBytes(phResp, sizeof(phResp)); 

    for (int i = 0; i < sizeof(phResp); i++) {
      Serial.print(phResp[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    uint16_t phValue = (phResp[3] << 8) | phResp[4];
    float pH = phValue / 10.0;

    Serial.print("pH: ");
    Serial.println(pH);
    }
}

void pCD() {
  byte cdResp[7] = { 0 };
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  mod.write(cdQuery, sizeof(cdQuery));
  digitalWrite(DE,LOW);
  digitalWrite(RE,LOW);
  delay(100);

  if (mod.available() >= sizeof(cdResp)) { 
    mod.readBytes(cdResp, sizeof(cdResp)); 

    for (int i = 0; i < sizeof(cdResp); i++) {
      Serial.print(cdResp[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    uint16_t cdValue = (cdResp[3] << 8) | cdResp[4];

    Serial.print("Conductivity: ");
    Serial.println(cdValue);
    }
}
