// Wi-Fi check: lists the networks the R4 can see, then tries to join the hotspot.
#include <WiFiS3.h>

const char* WIFI_SSID = "iPhone";
const char* WIFI_PASS = "sahilquazi9311";

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);
  Serial.println("\n=== Wi-Fi check ===");
  Serial.print("Wi-Fi module firmware: ");
  Serial.println(WiFi.firmwareVersion());
}

void loop() {
  Serial.println("\nScanning...");
  int n = WiFi.scanNetworks();
  bool found = false;
  for (int i = 0; i < n; i++) {
    Serial.print("  ["); Serial.print(WiFi.SSID(i)); Serial.print("]  signal ");
    Serial.print(WiFi.RSSI(i)); Serial.println(" dBm");
    if (String(WiFi.SSID(i)) == WIFI_SSID) found = true;
  }
  Serial.print(n); Serial.println(" networks seen.");
  Serial.println(found ? "Hotspot FOUND." : "Hotspot NOT found. Keep the Personal Hotspot screen open, Maximize Compatibility on.");

  if (found) {
    Serial.print("Joining...");
    int status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
    status = WiFi.status();
    Serial.print("\nStatus code: "); Serial.println(status);   // 3 = connected, 4 = failed (usually wrong password), 6 = disconnected
    if (status == WL_CONNECTED) {
      Serial.print("CONNECTED, IP: "); Serial.println(WiFi.localIP());
      while (true) delay(1000);
    }
    WiFi.disconnect();
  }
  delay(5000);
}
