#include <SPI.h>
#include <RH_RF95.h>
#

#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  3

#define RF95_FREQ 433.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct message {
  int16_t temp;
  int16_t alt; 
  int32_t pressure;
  uint8_t fix;
  uint8_t numSat;
  uint32_t latitude;
  uint32_t longitude;
  int16_t altGPS;
  uint16_t battVolt;
  uint8_t navStat;
  uint8_t status;
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
  int numSat = pkt.numSat;
  float latitude = pkt.latitude / 1e7f;
  float longitude = pkt.longitude / 1e7f;
  float altGPS = pkt.altGPS / 100.0;
  int battVolt = pkt.battVolt;
  int navStat = pkt.navStat;
  int status = pkt.status;

  // Build ASCII line to print
  sprintf(message, "%i; count = %i;temp = %.2f *C; pressure = %.2f; alt = %.2f; latitude = %.7f; longitude = %.7f; altGps = %.2f; battVolt = %i; navStat = %i; status = %i",
  temp,
  pressure,
  alt,
  fix,
  numSat,
  latitude,
  longitude,
  altGPS,
  battVolt,
  navStat,
  status
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
