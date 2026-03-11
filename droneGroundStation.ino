#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  3

#define RF95_FREQ 433.4

RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct message {

  int16_t temp;
  int16_t alt; 
  int32_t pressure;

  uint8_t fix;
  uint8_t numSat;
  uint32_t rawLat;
  uint32_t rawLong;
  int16_t gpsAlt;

  uint16_t battVolt;

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
  float temp = pkt.temp/100.0;
  float alt = pkt.alt/100.0;
  float pressure = pkt.pressure/100.0;
  int fix = pkt.fix;
  int numSat =pkt.numSat;
  float latitude = pkt.rawLat/1e7f;
  float longitude = pkt.rawLong/1e7f;
  float altGPS = pkt.gpsAlt/100.0;
  float battVolt = pkt.battVolt/100.0;
  int navMode =pkt.navMode;
    int navState =pkt.navState;
      int navError =pkt.navError;
        int state =pkt.state;
          int geozoneStatus =pkt.geozoneStatus;
  // Build ASCII line to print
  sprintf(message, "temp = %.2f *C; pressure = %.2f; alt = %.2f; fix = %i; numSat = %i; lat = %.7f; long = %.7f; altGPS = %.2f; battvolt = %.2f; navMode = %i; navState = %i; navError = %i; state = %i; geozoneState = %i;",
  temp,
  pressure,
  alt,
  fix,
  numSat,
  latitude,
  longitude,
  altGPS,
  battVolt,
  navMode,
  navState,
  navError,
  state,
  geozoneStatus
  );
  /*float accX = pkt.accX/100.0;
  float accY = pkt.accY/100.0;
  float accZ = pkt.accZ/100.0;
  float accTot = pkt.accTot/100.0;
  float accMax = pkt.accMax/100.0;
  sprintf(message, "accX = %.2f; accY = %.2f; accZ = %.2f; accTot = %.2f; accMax = %.2f", accX, accY, accZ, accTot, accMax);*/

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