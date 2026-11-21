// COVERAGE-AI bot firmware (ESP32-S3 + L298N), single router. Needs ESP32 Arduino core 3.x.
// Connects to the Wi-Fi AP you want to map, streams its RSSI over UDP,
// and drives the motors from UDP commands "M,<left>,<right>" (-255..255).
#include <WiFi.h>
#include <WiFiUdp.h>

const char* SSID = "Airtel-MyWiFi-PQR-C100-3D6E";          // the AP being mapped
const char* PASS = "60838253";
IPAddress PC(192, 168, 1, 107);          // laptop IPv4 on that network
const int TX_PORT = 5005, RX_PORT = 5006;
const int SEND_MS = 100;                 // 10 samples/s

const int ENA = 4, IN1 = 5, IN2 = 6, ENB = 7, IN3 = 15, IN4 = 16;

WiFiUDP udp;
uint32_t seq = 0, lastTx = 0, lastCmd = 0;

void drive(int l, int r) {
  digitalWrite(IN1, l > 0); digitalWrite(IN2, l < 0); ledcWrite(ENA, abs(l));
  digitalWrite(IN3, r > 0); digitalWrite(IN4, r < 0); ledcWrite(ENB, abs(r));
}

void setup() {
  Serial.begin(115200);
  int pins[] = {IN1, IN2, IN3, IN4};
  for (int p : pins) pinMode(p, OUTPUT);
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);
  drive(0, 0);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.printf("\nConnected %s ch%d BSSID %s\n", WiFi.localIP().toString().c_str(),
                WiFi.channel(), WiFi.BSSIDstr().c_str());
  udp.begin(RX_PORT);
}

void loop() {
  int n = udp.parsePacket();
  if (n) {
    char b[32]; int len = udp.read(b, 31); b[len] = 0;
    int l, r;
    if (sscanf(b, "M,%d,%d", &l, &r) == 2) {
      drive(constrain(l, -255, 255), constrain(r, -255, 255));
      lastCmd = millis();
    }
  }
  if (millis() - lastCmd > 500) drive(0, 0);   // failsafe stop

  if (millis() - lastTx >= SEND_MS) {
    lastTx = millis();
    if (WiFi.status() == WL_CONNECTED) {
      char buf[128];
      snprintf(buf, sizeof buf, "{\"seq\":%lu,\"rssi\":%d,\"ch\":%d,\"bssid\":\"%s\"}",
               (unsigned long)seq++, WiFi.RSSI(), WiFi.channel(), WiFi.BSSIDstr().c_str());
      udp.beginPacket(PC, TX_PORT); udp.print(buf); udp.endPacket();
    } else WiFi.reconnect();
  }
}
