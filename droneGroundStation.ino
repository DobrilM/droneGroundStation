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

void print_telemetry_packet_geiger(const uint8_t from, const uint8_t to, const int32_t rssi, message pkt) {
  Serial.print("# ");
  Serial.print(from, HEX);
  Serial.print(';');
  Serial.print(to, HEX);
  Serial.print(';');
  Serial.print(rssi);
  Serial.print(';');

  static char message[220];
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

  // Build ASCII line to print
  sprintf(message, "%u;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%u;%u;%u;%u;%.7f;%.7f;%i;%u;%u;%u;%u;%u;%u", 
  packetCounter, temp, pressure, alt, accX, accY, accZ, fix, numSat, rawLat, rawLong, latitude, longitude, altGPS, battVolt, navMode, navState, navError, geozoneState, state
  );

  // Send to USB / Serial port
    Serial.println(message);
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
      print_telemetry_packet_geiger(from, to, rssi, recievedMessage);
    } else {
      Serial.println("Receive failed");
    }
  }
}
