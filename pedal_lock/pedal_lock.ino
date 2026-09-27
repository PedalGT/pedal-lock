/*
  Pedal lock firmware — Arduino Uno R4 WiFi + PN532 NFC shield
  Unlock / network portion. Motor movement goes in the three MOTOR functions below.

  What it does
  - Connects to Wi-Fi and polls the server every second for UNLOCK / LOCK commands
    (sent when a rider unlocks or ends a ride in the app).
  - Acts as an NFC tag the iPhone can read (target mode). The phone reads
    "PEDAL:<CODE>" straight off the lock, then asks the server to start the ride.
    No sticker, no printed code.
  - Also still reads keycards / NFC fobs (initiator mode). A tap is sent to the
    server, which decides: start a ride (UNLOCK), end a ride (LOCK), link a card
    (LINKED), or deny.
  - Calls motorUnlock() / motorLock() (your code) and shows an icon on the LED matrix.

  The PN532 cannot read and be read at the same time, so loop() alternates:
  a short window pretending to be a tag, a short window looking for cards, then
  the server poll. See TAG_WINDOW_MS / CARD_WINDOW_MS below.

  Libraries
  - Seeed Studio PN532 library. The Adafruit one cannot do target mode at all,
    so this sketch does NOT use it.
      https://github.com/Seeed-Studio/PN532  ->  Code > Download ZIP
      Arduino IDE > Sketch > Include Library > Add .ZIP Library
      (adds PN532, PN532_I2C, PN532_SPI, PN532_HSU and NDEF folders)
  - WiFiS3 and Arduino_LED_Matrix come with the "Arduino UNO R4 Boards" package.

  Wiring
  - PN532 shield stacked on the R4, I2C mode (both DIP switches: SEL0 = 1, SEL1 = 0).
    This library polls I2C and does not use the IRQ line, but the shield still
    occupies D2 and D3 physically, so don't use those for your motor.
  - Servo on D9 (signal), 5V and GND. If the servo has its own battery, connect
    that battery's negative to R4 GND (common ground). A servo under load can
    brown out the R4 off USB alone.
  - Power the R4 from USB while testing.

  Reading the emulated tag needs an iPhone 7 or newer with the Pedal app, and the
  app must be built with the "Near Field Communication Tag Reading" capability
  (paid Apple Developer account). Typing the 6-character code always works instead.
*/

#include <WiFiS3.h>
#include <Wire.h>
#include <Servo.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <emulatetag.h>
#include "Arduino_LED_Matrix.h"

// ---------------- CONFIG: change these ----------------
const char* WIFI_SSID   = "YOUR_HOTSPOT_NAME";
const char* WIFI_PASS   = "YOUR_HOTSPOT_PASSWORD";
const char* SERVER_HOST = "192.168.1.50";   // your laptop's IP on the same Wi-Fi (not "localhost")
const int   SERVER_PORT = 3000;
const char* LOCK_ID     = "lock_xxxxxxxxxxxx"; // from the app: My bikes > Lock setup for Arduino
const char* DEVICE_KEY  = "xxxxxxxxxxxxxxxxxxxxxxxx";
const char* BIKE_CODE   = "XXXXXX";            // the 6-character code, same screen

const unsigned long POLL_MS = 1000;

// How long each loop spends pretending to be a tag vs. looking for keycards.
// The iPhone polls every ~100 ms while its scan sheet is up, so a 350 ms window
// is caught quickly. Raise TAG_WINDOW_MS if phone taps feel unreliable; raise
// CARD_WINDOW_MS if keycards do.
const uint16_t TAG_WINDOW_MS  = 350;
const uint16_t CARD_WINDOW_MS = 150;

// Servo latch. D9 is free: the shield holds D2 and D3, so keep off those.
const int SERVO_PIN      = 9;
const int LOCKED_ANGLE   = 0;      // where the latch sits closed
const int UNLOCKED_ANGLE = 90;     // where the latch sits open
const int SERVO_TRAVEL_MS = 600;   // time to let the horn actually get there
// -------------------------------------------------------

#define PN532_RESET 3   // the shield ties the PN532's reset line to D3

PN532_I2C pn532i2c(Wire);
PN532     nfc(pn532i2c);        // initiator mode: reading keycards
EmulateTag tag(pn532i2c);       // target mode: being read by a phone
ArduinoLEDMatrix matrix;
Servo latch;

// NFCID1 the phone sees. Any 3 bytes; the PN532 prepends 0x08 for a random UID.
uint8_t tagUid[3] = { 0x0F, 0xED, 0xA1 };

// The NDEF file the phone reads: a 2-byte length, then one NDEF text record.
uint8_t ndefFile[64];
uint16_t ndefFileLength = 0;

bool isLocked = true;
unsigned long lastPoll = 0;
String lastUid = "";
unsigned long lastUidAt = 0;

// ---------- LED matrix icons (8 rows x 12 cols) ----------
uint8_t ICON_LOCK[8][12] = {
  {0,0,0,0,1,1,1,1,0,0,0,0},
  {0,0,0,1,0,0,0,0,1,0,0,0},
  {0,0,0,1,0,0,0,0,1,0,0,0},
  {0,0,1,1,1,1,1,1,1,1,0,0},
  {0,0,1,1,1,1,1,1,1,1,0,0},
  {0,0,1,1,1,0,0,1,1,1,0,0},
  {0,0,1,1,1,1,1,1,1,1,0,0},
  {0,0,1,1,1,1,1,1,1,1,0,0}
};
uint8_t ICON_OPEN[8][12] = {
  {0,0,0,0,0,0,0,1,1,1,1,0},
  {0,0,0,0,0,0,1,0,0,0,0,1},
  {0,0,0,0,0,0,1,0,0,0,0,0},
  {0,0,1,1,1,1,1,1,1,1,0,0},
  {0,0,1,1,1,1,1,1,1,1,0,0},
  {0,0,1,1,1,0,0,1,1,1,0,0},
  {0,0,1,1,1,1,1,1,1,1,0,0},
  {0,0,1,1,1,1,1,1,1,1,0,0}
};
uint8_t ICON_CHECK[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,1,1,0},
  {0,0,0,0,0,0,0,0,1,1,0,0},
  {0,0,0,0,0,0,0,1,1,0,0,0},
  {0,1,1,0,0,0,1,1,0,0,0,0},
  {0,0,1,1,0,1,1,0,0,0,0,0},
  {0,0,0,1,1,1,0,0,0,0,0,0},
  {0,0,0,0,1,0,0,0,0,0,0,0}
};
uint8_t ICON_X[8][12] = {
  {0,0,1,1,0,0,0,0,1,1,0,0},
  {0,0,0,1,1,0,0,1,1,0,0,0},
  {0,0,0,0,1,1,1,1,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,1,1,1,1,0,0,0,0},
  {0,0,0,1,1,0,0,1,1,0,0,0},
  {0,0,1,1,0,0,0,0,1,1,0,0}
};
uint8_t ICON_WIFI[8][12] = {
  {0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,1,1,1,1,1,1,0,0,0},
  {0,0,1,0,0,0,0,0,0,1,0,0},
  {0,0,0,0,1,1,1,1,0,0,0,0},
  {0,0,0,1,0,0,0,0,1,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,1,1,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0}
};

void showIcon(uint8_t icon[8][12]) { matrix.renderBitmap(icon, 8, 12); }
void showStateIcon() { if (isLocked) showIcon(ICON_LOCK); else showIcon(ICON_OPEN); }

// ---------- the NDEF message the phone reads ----------
// One NDEF Text record holding "PEDAL:<CODE>". A Text record is deliberate: a
// URI record makes iOS pop a Safari banner whenever a phone drifts past the
// lock, while a Text record stays silent until the app is actually scanning.
// The app parses either (see NFCScanner.swift).
void buildNdefFile() {
  String text = String("PEDAL:") + BIKE_CODE;

  uint8_t payloadLength = 1 + 2 + text.length();   // status byte + "en" + text
  uint8_t i = 2;                                   // leave room for the 2-byte NLEN

  ndefFile[i++] = 0xD1;                 // MB | ME | SR | TNF = NFC Forum well-known
  ndefFile[i++] = 0x01;                 // type length: "T"
  ndefFile[i++] = payloadLength;
  ndefFile[i++] = 'T';                  // record type: Text
  ndefFile[i++] = 0x02;                 // UTF-8, language code is 2 bytes
  ndefFile[i++] = 'e';
  ndefFile[i++] = 'n';
  for (uint16_t n = 0; n < text.length(); n++) ndefFile[i++] = text[n];

  uint16_t messageLength = i - 2;
  ndefFile[0] = (messageLength >> 8) & 0xFF;
  ndefFile[1] = messageLength & 0xFF;
  ndefFileLength = i;

  Serial.println("Emulating tag: " + text);
}

// =====================================================================
// ===================  MOTOR CODE GOES HERE  ==========================
// Put your motor pins / #includes at the top of the file, then fill in
// these three functions. Everything else in this sketch calls only these.
// =====================================================================

void motorSetup() {
  // Runs once at startup.
  latch.attach(SERVO_PIN);
  latch.write(LOCKED_ANGLE);
  delay(SERVO_TRAVEL_MS);
}

void motorUnlock() {
  // Move the mechanism to the UNLOCKED position.
  latch.write(UNLOCKED_ANGLE);
  delay(SERVO_TRAVEL_MS);
}

void motorLock() {
  // Move the mechanism to the LOCKED position.
  latch.write(LOCKED_ANGLE);
  delay(SERVO_TRAVEL_MS);
}

// =====================================================================

void setLocked(bool locked) {
  if (locked) motorLock(); else motorUnlock();
  isLocked = locked;
  showStateIcon();
  Serial.println(locked ? "Locked" : "Unlocked");
}

// ---------- Wi-Fi ----------
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  showIcon(ICON_WIFI);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 10 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
  }
  Serial.print("\nConnected, IP: ");
  Serial.println(WiFi.localIP());
  showStateIcon();
}

// ---------- tiny HTTP client (plain-text responses) ----------
String httpRequest(const char* method, String path, String body) {
  WiFiClient client;
  client.setTimeout(3000);
  if (!client.connect(SERVER_HOST, SERVER_PORT)) return "ERR:connect";

  client.print(String(method) + " " + path + " HTTP/1.1\r\n");
  client.print(String("Host: ") + SERVER_HOST + "\r\n");
  client.print(String("X-Device-Key: ") + DEVICE_KEY + "\r\n");
  client.print("Connection: close\r\n");
  if (body.length()) {
    client.print("Content-Type: application/x-www-form-urlencoded\r\n");
    client.print("Content-Length: " + String(body.length()) + "\r\n");
  }
  client.print("\r\n");
  if (body.length()) client.print(body);

  // Read whole response, then take what's after the headers
  String resp = "";
  unsigned long start = millis();
  while ((client.connected() || client.available()) && millis() - start < 4000) {
    while (client.available()) resp += (char)client.read();
  }
  client.stop();
  int split = resp.indexOf("\r\n\r\n");
  if (split < 0) return "ERR:bad_response";
  String out = resp.substring(split + 4);
  out.trim();
  return out;
}

void reportEvent(const char* event) {
  httpRequest("POST", "/api/device/event", String("lock=") + LOCK_ID + "&event=" + event);
}

void handleCommand(String cmd) {
  if (cmd == "UNLOCK") { setLocked(false); reportEvent("unlocked"); }
  else if (cmd == "LOCK") { setLocked(true); reportEvent("locked"); }
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(1500);
  matrix.begin();
  showIcon(ICON_WIFI);
  motorSetup();

  buildNdefFile();

  // The shield wires the PN532's reset line to D3. Adafruit's library pulsed it;
  // the Seeed library never touches it, so do it here or the chip can sit held
  // in reset and report "not found".
  pinMode(PN532_RESET, OUTPUT);
  digitalWrite(PN532_RESET, LOW);
  delay(10);
  digitalWrite(PN532_RESET, HIGH);
  delay(50);

  // Retry rather than halt: a shield that is merely seated badly shouldn't
  // brick the lock until someone power-cycles it.
  nfc.begin();
  uint32_t version = 0;
  while (!(version = nfc.getFirmwareVersion())) {
    Serial.println("PN532 not found. Check the shield is seated and set to I2C.");
    showIcon(ICON_X);
    delay(2000);
  }
  Serial.print("PN532 firmware ");
  Serial.print((version >> 16) & 0xFF); Serial.print('.'); Serial.println((version >> 8) & 0xFF);
  nfc.SAMConfig();

  // Target mode: what the phone sees when it taps the lock.
  tag.setUid(tagUid);
  tag.setNdefFile(ndefFile + 2, ndefFileLength - 2);   // library adds the length header itself

  ensureWiFi();
  setLocked(true);
  reportEvent("locked");
  Serial.println("Ready. Tap a phone or a card, or unlock from the app.");
}

// Returns the tapped card's UID as uppercase hex, or "" if nothing was there.
String readCardUid() {
  uint8_t uid[7];
  uint8_t uidLength = 0;
  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, CARD_WINDOW_MS)) return "";

  String uidHex = "";
  for (uint8_t i = 0; i < uidLength; i++) {
    if (uid[i] < 0x10) uidHex += "0";
    uidHex += String(uid[i], HEX);
  }
  uidHex.toUpperCase();
  return uidHex;
}

void handleTapReply(String reply) {
  Serial.println("Server: " + reply);
  if (reply == "UNLOCK" || reply == "LOCK") {
    showIcon(ICON_CHECK); delay(500);
    handleCommand(reply);
  } else if (reply == "LINKED") {
    showIcon(ICON_CHECK); delay(1200); showStateIcon();
  } else {
    showIcon(ICON_X); delay(1200); showStateIcon();   // DENY:low_balance, DENY:unknown_card, errors
  }
}

void loop() {
  ensureWiFi();

  // 1) Be a tag for a moment, so a phone held against the lock can read the
  //    bike code off it. The phone does the rest through the app and the server;
  //    the UNLOCK arrives on the next poll below, exactly as an app unlock does.
  if (tag.emulate(TAG_WINDOW_MS)) {
    Serial.println("A phone read the lock.");
    showIcon(ICON_CHECK);
    delay(400);
    showStateIcon();
  }
  // emulate() leaves the chip in target mode; put it back to reader mode.
  nfc.SAMConfig();

  // 2) Look for a keycard or fob.
  String uidHex = readCardUid();
  if (uidHex.length()) {
    // Ignore the same card held on the reader for 3 seconds
    if (uidHex != lastUid || millis() - lastUidAt > 3000) {
      lastUid = uidHex;
      lastUidAt = millis();
      Serial.println("Card: " + uidHex);
      handleTapReply(httpRequest("POST", "/api/device/tap",
                                 String("lock=") + LOCK_ID + "&uid=" + uidHex));
    }
  }

  // 3) Poll the server for app-triggered commands
  if (millis() - lastPoll > POLL_MS) {
    lastPoll = millis();
    String cmd = httpRequest("GET", String("/api/device/poll?lock=") + LOCK_ID, "");
    if (cmd == "UNLOCK" || cmd == "LOCK") {
      Serial.println("Command: " + cmd);
      handleCommand(cmd);
    } else if (cmd.startsWith("ERR") || cmd.startsWith("DENY")) {
      Serial.println("Poll problem: " + cmd);
    }
  }
}
