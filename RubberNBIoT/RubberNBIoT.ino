#include "LowPower.h"
#include "AIS_NB_BC95.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define BAT_PIN A0
#define MOI_PIN A1
#define S51_RE 11
#define S51_DE 10
#define S51_RO 5
#define S51_DI 6
#define SWITCH_SENSOR 7
#define SEALEVELPRESSURE_HPA (1013.25)

#define TO_SLEEP_MASK 0b10000000  // Bit 7 (MSB) for toSleep
#define DEBUG_MASK    0b01000000  // Bit 6 for debug
#define STATE_MASK    0b00110000  // Bits 4-5 for state
#define STATE_SHIFT   4           // Right-shift to extract state bits

// State values
#define STATE_DEEP_SLEEP 0b00  // 00 for Deep Sleep
#define STATE_SAMPLING   0b01  // 01 for Sampling
#define STATE_CALI_SLEEP 0b10  // 10 for Calibrate

const char deviceID = 'a';

#define samplingCycle 9 // 9
#define samplingTimes 5 // 5
uint16_t battery = 0;
SoftwareSerial mod(S51_RO, S51_DI);

const float offsetPercent = 0.89; // offset of exceeded sleep time
const uint32_t secInHour = 3600;
const uint32_t secInMinute = 60;

#define wakeTime 24 // wake up time 24
#define sleepTime 3 // start sleep time 3
#define sleepHour 6 // max hour to sleep
#define timeoutSleep 3 // hour to sleep when timeout
uint32_t tdSTime = 0; // total deep sleep
uint32_t rmSTime = (uint32_t)secInHour * 21; // total remaining
uint32_t diSTime = 0; // this deep sleep instance

AIS_NB_BC95 AISnb;
#define RESET_NB_PIN 4
#define address "34.2.30.58"
#define port "9999"
#define cmdTime "{\"c\":\"t\",\"d\":\"%c\"}"
#define cmdDataStart "{\"c\":\"d\",\"d\":\"%c%05u%05u%05u%05u%05u%05u%05u%05u%05u%05u\"}"
#define DEBUG_FLAG ((flag & DEBUG_MASK) != 0) // Use a unique name instead of "debug"
#define TO_SLEEP_FLAG ((flag & TO_SLEEP_MASK) != 0) // Use unique names for all macros
const uint16_t millisTimeout = 30000;
const uint8_t maxTimeout = 10;

uint16_t moisture = 0, conductivity = 0, nitrogen = 0, phosphorus = 0, potassium = 0, pHValue = 0, temperature = 0, humidity = 0, pressure = 0;
char message[75];

uint8_t flag = 0b01100000; // to sleep, debug , state (D, S, C) * 2, toSample, res, res, res

void(* resetFunc) (void) = 0;


void setup() {
  Serial.begin(9600);
  Serial.println(F("Setup..."));
  resetSensor();
  AISnb.debug = DEBUG_FLAG;
  AISnb.setupDevice(port);
  AISnb.pingIP(address);
  if (DEBUG_FLAG) showNBinfo();
  EEPROM.get(0, flag);
  Serial.println(F("Starting..."));
  delay(1000);
}

void loop() {
  // Extract the current state from the flag
  uint8_t state = (flag & STATE_MASK) >> STATE_SHIFT;

  if (!(flag & TO_SLEEP_MASK)) {
    flag = (flag & ~STATE_MASK) | (STATE_SAMPLING << STATE_SHIFT);
  }

  switch (state) {
    case STATE_CALI_SLEEP:
      if (flag & DEBUG_MASK) Serial.println(F("Calibrate..."));
      calibrateTime();
      if (EEPROM.read(0) != flag) {
        EEPROM.put(0, flag);
      }
      if (tdSTime >= ((uint32_t)secInHour * 21)) {
        // Transition to Sampling
        flag = (flag & ~STATE_MASK) | (STATE_SAMPLING << STATE_SHIFT);
        if (EEPROM.read(0) != flag) {
          EEPROM.put(0, flag);
        }
        resetTimeCount();
      }
      if (((flag & STATE_MASK) >> STATE_SHIFT) == STATE_DEEP_SLEEP) {
        break;
      }
      resetFunc();
      break;

    case STATE_DEEP_SLEEP:
      if (flag & DEBUG_MASK) Serial.println(F("Deep Sleep..."));
      deepSleep();
      // Transition to Calibrate Sleep
      flag = (flag & ~STATE_MASK) | (STATE_CALI_SLEEP << STATE_SHIFT);
      if (EEPROM.read(0) != flag) {
        EEPROM.put(0, flag);
      }
      resetFunc();
      break;

    case STATE_SAMPLING:
      if (flag & DEBUG_MASK) Serial.println(F("Sampling..."));
      sampling();
      // Transition to Calibrate Sleep
      flag = (flag & ~STATE_MASK) | (STATE_CALI_SLEEP << STATE_SHIFT);
      break;
  }
}

void calibrateTime() {
  UDPReceive dataResp;
  dataResp.data = "";
  char asciiResp[100] = {0};
  uint32_t cnt = 0;
  uint8_t timeout = 0;
  UDPSend udp;
  delay(500);
  pingRESP pingR = AISnb.pingIP(address);
  if (!pingR.status) {
    resetConnection(true);
  }
  snprintf(
    message, sizeof(message),
    cmdTime,
    deviceID
  );
  udp = sendNBmsg(message, true);

  while ((dataResp.data == "") || (strncmp("343030", dataResp.data.c_str(), 3) == 0)) {
    if (timeout >= maxTimeout) {
      Serial.println(F("Timeout!"));
      if (DEBUG_FLAG) AISnb.pingIP(address);
      uint16_t totalTimeout = (millisTimeout / 1000) * maxTimeout;
      if (rmSTime > totalTimeout) {
        rmSTime -= totalTimeout;
        tdSTime += totalTimeout;
      }
      diSTime = (rmSTime > (timeoutSleep * secInHour) ) ? (timeoutSleep * secInHour) : rmSTime;
      if (DEBUG_FLAG) showTimeInfo();
      flag |= (1 << 7);
      return ;
    }
    if (cnt >= millisTimeout) {
      Serial.println(F("Resending from timeout..."));
      snprintf(
        message, sizeof(message),
        cmdTime,
        deviceID
      );
      udp = sendNBmsg(message, false);
      cnt = 0;
      timeout++;
    }

    dataResp = AISnb.waitResponse();
    delay(1);
    cnt++;
  }

  hexToAscii(dataResp.data, asciiResp);
  dataResp.data.remove(0);
  // Serial.println("# Receive : " + dataAscii);
  uint8_t timeArray[2] = {12, 0};
  getTimeFromJson(asciiResp, timeArray);
  delay(500);

  uint8_t hour = timeArray[0];
  uint8_t min = timeArray[1];
  if (DEBUG_FLAG) showServerTime(hour, min);
  if ( (hour < wakeTime) && (hour >= sleepTime) ) {
    uint8_t remainHour = wakeTime - hour - 1;
    uint8_t remainMin = 60 - min;
    rmSTime = (remainHour * secInHour) + (remainMin * secInMinute);
    diSTime = (rmSTime > (sleepHour * secInHour) ) ? (sleepHour * secInHour) : rmSTime;
    if (DEBUG_FLAG) showTimeInfo();
    delay(1000);
    flag = (flag & ~STATE_MASK) | (STATE_DEEP_SLEEP << STATE_SHIFT);
    flag |= (1 << 7);
    return ;
  }
  flag &= ~(1 << 7);
  return ;
}

void deepSleep() {
  delay(500); // wait for stuff to done
  longSleep(diSTime * offsetPercent);
  tdSTime += diSTime;
  rmSTime -= diSTime;
  if (DEBUG_FLAG) Serial.println(F("Wake up!"));
  if (DEBUG_FLAG) showTimeInfo();
}

void sampling() {
  uint8_t getSensorSec = 7, sendDataSec = 14;
  // uint16_t moi = 0, cnd = 0, ntg = 0, php = 0, pts = 0;
  // float ph = 0.0, tmp = 0.0, hmd = 0.0, prs = 0.0;
  uint32_t delayCounter = 0;
  UDPSend udp;
  pingRESP pingR;

  for (int cycle = 0; cycle < samplingCycle; cycle++) {
    moisture = 0, conductivity = 0, nitrogen = 0, phosphorus = 0, potassium = 0;
    pHValue = 0, temperature = 0, humidity = 0, pressure = 0;
    delayCounter = 0;

    if (DEBUG_FLAG) Serial.print(F("Iteration: "));
    if (DEBUG_FLAG) Serial.println(cycle + 1);

    // sample every 1 minute 5 times
    for (int i = 0; i < samplingTimes; i++) {
      startSwitch();
      startSensor();
      delay(3000);
      getMoisture();
      delay(200);
      getConduct();
      delay(200);
      getNPK();
      delay(200);
      getPH();
      delay(200);
      resetSensor();
      endSwitch();
      if (DEBUG_FLAG) Serial.print(F("Going minute "));
      if (DEBUG_FLAG) Serial.print(i + 1);
      if (DEBUG_FLAG) Serial.println(F(" sleep"));
      delay(1000);
      // longSleep((secInMinute - getSensorSec - 45) * offsetPercent); // in testing -45
      longSleep((secInMinute - getSensorSec) * offsetPercent); 
    }
    startSwitch();
    getBattery();
    delay(500);
    endSwitch();
    resetSensor();

    pingR = AISnb.pingIP(address);
    if (!pingR.status) {
      resetConnection(true);
    }

    Serial.print(F("Sending STR: "));
    snprintf(
        message, sizeof(message),
        cmdDataStart,
        deviceID, moisture / 5,conductivity / 5, nitrogen / 5, phosphorus / 5, battery, potassium / 5, pHValue / 5, temperature / 5, humidity / 5, pressure / 5
    );
    Serial.println(message);
    udp = sendNBmsg(message, true);
    delay(3000);

    // Delay 15 minutes
    if (DEBUG_FLAG) Serial.println(F("15 minutes delay starting..."));
    delay(2000);
    // longSleep(((secInMinute - 40) - sendDataSec) * offsetPercent); // in testing -40
    longSleep(((secInMinute * 15) - sendDataSec) * offsetPercent);
  }
  Serial.println(F("Offsetting..."));
  delay(1000);
  longSleep((secInMinute * 5) * offsetPercent);
  flag = (flag & ~STATE_MASK) | (STATE_CALI_SLEEP << STATE_SHIFT);
  flag |= (1 << 7);
  EEPROM.put(0, flag);
  resetFunc();
  delay(100);
}

UDPSend sendNBmsg(char * data, bool resend) {
  UDPSend udp;
  uint8_t maxRetry = 5;
  uint8_t retry = 0;
  udp.status = false;
  udp = AISnb.sendUDPmsgStr(address, port, data);
  if (!udp.status || udp.status == NULL) {
    resetConnection(true);
    if (resend) {
      Serial.println(F("Resending msg..."));
      AISnb.sendUDPmsgStr(address, port, data);
      while (!udp.status && retry < maxRetry) {
        Serial.println(F("Retrying..."));
        resetConnection(true);
        AISnb.sendUDPmsgStr(address, port, data);
        retry++;
      }
    }
  }
  return udp;
}

void resetConnection(bool force) {
  if (!AISnb.getNBConnect() || force) {
    Serial.println(F("Resetting NB"));
    AISnb.rebootModule();
    delay(10000);
    if (DEBUG_FLAG) AISnb.pingIP(address);
  }
}

void resetTimeCount() {
  tdSTime = 0;
  rmSTime = (uint32_t)secInHour*21;
  diSTime = 0;
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

void hexToAscii(const String& hexStr, char* asciiArr) {
  uint8_t length = (uint8_t) hexStr.length();
  
  for (int i = 0; i < length; i += 2) {
    char highNibble = hexStr[i];
    char lowNibble = hexStr[i + 1];
    
    byte byteVal = (hexCharToByte(highNibble) << 4) | hexCharToByte(lowNibble);
    asciiArr[i / 2] = byteVal;
  }
  asciiArr[length / 2] = '\0';
  Serial.println(asciiArr);

}

byte hexCharToByte(char hexChar) {
  if (hexChar >= '0' && hexChar <= '9') {
    return hexChar - '0'; 
  } 
  else if (hexChar >= 'A' && hexChar <= 'F') {
    return hexChar - 'A' + 10;
  }
  else if (hexChar >= 'a' && hexChar <= 'f') {
    return hexChar - 'a' + 10;
  }
  return 0;
}

void getTimeFromJson(const char* jsonStr, uint8_t* timeArr) {
  const char* timeStart = strstr(jsonStr, "\"ts\": \"");
  if (timeStart != NULL) {
    timeStart += 7;
    const char* timeEnd = strchr(timeStart, '\"');
    
    if (timeEnd != NULL) {
      char timeStr[6]; 
      strncpy(timeStr, timeStart, timeEnd - timeStart);
      timeStr[timeEnd - timeStart] = '\0'; 
      sscanf(timeStr, "%d:%d", &timeArr[0], &timeArr[1]);
    }
  }
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
  mod.begin(4800);
  pinMode(S51_RE, OUTPUT);
  pinMode(S51_DE, OUTPUT);
  while (mod.available()) { 
    mod.read(); 
  }
}

void resetSensor() {
  mod.end();
  digitalWrite(S51_RE, LOW);
  digitalWrite(S51_DE, LOW);
  pinMode(S51_RE, INPUT);
  pinMode(S51_DE, INPUT);
}

void getBattery() {
  battery = analogRead(BAT_PIN);
  if (DEBUG_FLAG) {
    Serial.print(F("Battery: "));
    Serial.println(battery);
  }
}

void getMoisture() {
  moisture += analogRead(MOI_PIN);
  if (DEBUG_FLAG) {
    Serial.print(F("Moisture: "));
    Serial.println(moisture);
  }
}

void getNPK() {
  byte npkQuery[]= {0x01, 0x03, 0x00, 0x04, 0x00, 0x03, 0x44, 0x0a};
  byte npkResp[11] = { 0 };
  digitalWrite(S51_DE,HIGH);
  digitalWrite(S51_RE,HIGH);
  mod.write(npkQuery, sizeof(npkQuery));
  digitalWrite(S51_DE,LOW);
  digitalWrite(S51_RE,LOW);
  delay(200);

  if (mod.available() >= sizeof(npkResp)) { 
    mod.readBytes(npkResp, sizeof(npkResp)); 

    if (DEBUG_FLAG) {
      for (int i = 0; i < sizeof(npkResp); i++) {
        Serial.print(npkResp[i], HEX);
        Serial.print(F(" "));
      }
      Serial.println();
    }

    nitrogen += (npkResp[3] << 8) | npkResp[4];
    phosphorus += (npkResp[5] << 8) | npkResp[6];
    potassium += (npkResp[7] << 8) | npkResp[8];

    if (DEBUG_FLAG) {
      Serial.print(F("Nitrogen: "));
      Serial.println((npkResp[3] << 8) | npkResp[4]);
      Serial.print(F("Phosphorus: "));
      Serial.println((npkResp[5] << 8) | npkResp[6]);
      Serial.print(F("Potassium: "));
      Serial.println((npkResp[7] << 8) | npkResp[8]);
    }
  }
}

void getPH() {
  byte phQuery[]= {0x01, 0x03, 0x00, 0x03, 0x00, 0x01, 0x74, 0x0a};
  byte phResp[7] = { 0 };
  digitalWrite(S51_DE,HIGH);
  digitalWrite(S51_RE,HIGH);
  mod.write(phQuery, sizeof(phQuery));
  digitalWrite(S51_DE,LOW);
  digitalWrite(S51_RE,LOW);
  delay(200);

  if (mod.available() >= sizeof(phResp)) { 
    mod.readBytes(phResp, sizeof(phResp)); 

    if (DEBUG_FLAG) {
      for (int i = 0; i < sizeof(phResp); i++) {
        Serial.print(phResp[i], HEX);
        Serial.print(F(" "));
      }
      Serial.println();
    }

    uint16_t pH = (phResp[3] << 8) | phResp[4];
    pHValue += pH * 10.0;

    if (DEBUG_FLAG) {
      Serial.print(F("pH: "));
      Serial.println(pH / 10.0);
    }
  }
}

void getConduct() {
  byte cdQuery[]= {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xca};
  byte cdResp[7] = { 0 };
  digitalWrite(S51_DE,HIGH);
  digitalWrite(S51_RE,HIGH);
  mod.write(cdQuery, sizeof(cdQuery));
  digitalWrite(S51_DE,LOW);
  digitalWrite(S51_RE,LOW);
  delay(200);

  if (mod.available() >= sizeof(cdResp)) { 
    mod.readBytes(cdResp, sizeof(cdResp)); 

    if (DEBUG_FLAG) {
      for (int i = 0; i < sizeof(cdResp); i++) {
        Serial.print(cdResp[i], HEX);
        Serial.print(F(" "));
      }
      Serial.println();
    }

    conductivity += (cdResp[3] << 8) | cdResp[4];

    if (DEBUG_FLAG) {
      Serial.print(F("Conductivity: "));
      Serial.println((cdResp[3] << 8) | cdResp[4]);
    }
  }
}

void showNBinfo() {
  AISnb.getDeviceIP();
  AISnb.getSignal();
}

void showServerTime(uint8_t hour, uint8_t min) {
  Serial.print(F("Time : "));
  Serial.print(hour);
  Serial.print(F(":"));
  Serial.println(min);
}

void showTimeInfo() {
  Serial.print(F("Total Deep Sleep  : "));
  Serial.println(tdSTime);
  Serial.print(F("Remain Time Sleep : "));
  Serial.println(rmSTime);
  Serial.print(F("This Sleep Time   : "));
  Serial.println(diSTime);
}