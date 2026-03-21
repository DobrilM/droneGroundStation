#include <SPI.h>
#include <RH_RF95.h>
#

#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  3

#define RF95_FREQ 433.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct message {

  uint32_t packetCounter; //IMPLEMENT!!!!

  int16_t temp;
  int16_t alt; 
  int32_t pressure;

  uint8_t fix;
  uint8_t numSat;
  uint32_t rawLat;
  uint32_t rawLong;
  int16_t gpsAlt;

  uint16_t battVolt;

  int16_t accX;
  int16_t accY;
  int16_t accZ;


  uint8_t navMode;
  uint8_t navState;
  uint8_t navError;

  uint8_t state;

  uint8_t geozoneStatus;
};

message recievedMessage;
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa RX Test!");

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}

const char* stateToCArr(int value) {
    switch (value) {
        case 0:    return "STANDBY";
        case 1:  return "ASCENDING";
        case 2: return "DESCENDING";
        case 3: return "WAITFORFIX";
        case 4:       return "RTWP";
        case 5:   return "FAILSAFE";
        default:         return "ERROR";
    }
}

const char* navModeToCArr(int value) {
    switch (value) {
        case 0: return "NONE";
        case 1: return "HOLD";
        case 2: return "RTH";
        case 3: return "NAV";
        case 15: return "EMERG";
        default: return "ERROR";
    }
}
const char* navStateToCArr(int value) {
    switch (value) {
        case 0: return "NONE";
        case 1: return "RTH_START";
        case 2: return "RTH_ENROUTE";
        case 3: return "HOLD_INFINITE";
        case 4: return "HOLD_TIMED";
        case 5: return "WP_ENROUTE";
        case 6: return "PROCESS_NEXT";
        case 7: return "DO_JUMP";
        case 8: return "LAND_START";
        case 9: return "LAND_IN_PROGRESS";
        case 10: return "LANDED";
        case 11: return "LAND_SETTLE";
        case 12: return "LAND_START_DESCENT";
        default: return "ERROR";
    }
}

const char* navErrorToCArr(int value) {
    switch (value) {
        case 0: return "NONE";
        case 1: return "TOOFAR";
        case 2: return "SPOILED_GPS";
        case 3: return "WP_CRC";
        case 4: return "FINISH";
        case 5: return "TIMEWAIT";
        case 6: return "INVALID_JUMP";
        case 7: return "INVALID_DATA";
        case 8: return "WAIT_FOR_RTH_ALT";
        case 9: return "GPS_FIX_LOST";
        case 10: return "DISARMED";
        case 11: return "LANDING";
        default:return "ERROR";
    }
}


void printTelemetryPacket(const uint8_t from, const uint8_t to, const int32_t rssi, message pkt) {
  Serial.print("# ");
  Serial.print(from, HEX);
  Serial.print(';');
  Serial.print(to, HEX);
  Serial.print(';');
  Serial.print(rssi);
  Serial.print(';');

  uint32_t packetCounter = pkt.packetCounter;
  float temp = pkt.temp/100.0;
  float alt = pkt.alt/100.0;
  float pressure = pkt.pressure/100.0;

  float accX = pkt.accX/100.0;
  float accY = pkt.accY/100.0;
  float accZ = pkt.accZ/100.0;

  uint8_t fix = pkt.fix;
  uint8_t numSat = pkt.numSat;
  uint32_t rawLat = pkt.rawLat;
  uint32_t rawLong = pkt.rawLong; 
  float latitude = rawLat / 1e7f;
  float longitude = rawLong/ 1e7f;
  int16_t altGPS = pkt.gpsAlt;

  uint16_t battVolt = pkt.battVolt;
  uint8_t navMode = pkt.navMode;
  uint8_t navState = pkt.navState;
  uint8_t navError = pkt.navError;
  uint8_t state = pkt.state;
  uint8_t geozoneState = pkt.geozoneStatus;

  //message creation
  static char message[220];
  sprintf(message, 
  "packetCounter=%i;temp=%.2f;pressure=%.2f;alt=%.2f;accX=%.2f;accY=%.2f;accZ=%.2f;fix=%u;numSat=%u;rawLat=%u;rawLong=%u;latitude=%.7f;longitude=%.7f;altGPS=%i;battVolt=%u;geozoneState=%u;",
  packetCounter, 
  temp, pressure, alt, 
  accX, accY, accZ, 
  fix, numSat, rawLat, rawLong, latitude, longitude, altGPS, 
  battVolt, 
  navMode, navState, navError, 
  geozoneState, 
  state);

  Serial.println(message);

  Serial.print("current mode = ");
  Serial.print(stateToCArr(state));
  Serial.print(";");
  Serial.print("current FC mode =");
  Serial.print(navModeToCArr(navMode));
  Serial.print(";");
  Serial.print("current FC state = ");
  Serial.print(navStateToCArr(navState));
  Serial.print(";");
  Serial.print("FC error = ");
  Serial.println(navErrorToCArr(navError));
}

void loop() {
  if (rf95.available()) {
    
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      const uint8_t from = rf95.headerFrom();
      const uint8_t to = rf95.headerTo();
      const int32_t rssi = rf95.lastRssi();
      memcpy(&recievedMessage, buf, sizeof(recievedMessage));
      printTelemetryPacket(from, to, rssi, recievedMessage);
    } else {
      Serial.println("Receive failed");
    }
  }
}
