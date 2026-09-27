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
// Wi-Fi networks, tried in order. If one joins but can't reach the server,
// the lock moves to the next.
// iPhone hotspot names usually have a curly apostrophe, so both spellings are here.
struct WiFiNetwork { const char* ssid; const char* pass; };
const WiFiNetwork WIFI_NETWORKS[] = {
  { "Nishchai\xE2\x80\x99" "s iPhone", "testtest" },   // Nishchai’s iPhone (curly ’)
  { "Nishchai's iPhone",              "testtest" },
};
const int WIFI_NETWORK_COUNT = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);
const int SERVER_FAILS_BEFORE_SWITCH = 4;   // failed server connections in a row
const char* SERVER_HOST = "pedalgt-esb6dmewe0bnafbz.westus-01.azurewebsites.net";   // Pedal on Azure (plain HTTP, port 80)
const int   SERVER_PORT = 80;
const char* LOCK_ID     = "lock_0126d1a6cd1d";  // Test Bike
const char* DEVICE_KEY  = "8ea23acdca03e61d1d39db61";
const char* BIKE_CODE   = "NM69B2";

const unsigned long POLL_MS = 1000;

// How long each loop spends pretending to be a tag vs. looking for keycards.
// The iPhone polls every ~100 ms while its scan sheet is up, so a 350 ms window
// is caught quickly. Raise TAG_WINDOW_MS if phone taps feel unreliable; raise
// CARD_WINDOW_MS if keycards do.
// CARD_READING = false: the lock only acts as a tag. iPhones treat a PN532 in
// reader mode as a payment terminal and pop Apple Pay, and every card window is
// time the phone can't read the lock. Turn on only to use keycards / fobs.
const bool     CARD_READING   = true;
// Tag emulation (the phone reading the PN532) is off: the lock is a card reader
// only. Phones start rides from the app or a ride link instead.
const bool     TAG_EMULATION  = false;

// PLACEHOLDER: the one card this lock accepts. Any other card is rejected here,
// without asking the server. Replace with the real card's UID (tap it and read
// "Card: ..." in the monitor). Must match DEMO_CARD_UID in server/server.js.
const char*    ALLOWED_CARD_UID = "04842E726B1C90";
const uint16_t TAG_WINDOW_MS  = CARD_READING ? 350 : 1000;
const uint16_t CARD_WINDOW_MS = 300;
// How many times the PN532 itself looks for a card per search. Its default
// (0xFF) means "forever": it kept searching after we'd stopped waiting, so it
// was nearly always mid-command, and an upload/reset left it stuck.
const uint8_t  CARD_SEARCH_RETRIES = 0x05;

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
// Resting screen: padlock while locked, check mark while unlocked (ride in progress).
void showStateIcon() { if (isLocked) showIcon(ICON_LOCK); else showIcon(ICON_CHECK); }

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
// Never blocks for long: NFC keeps working while Wi-Fi is down. One join
// attempt at most every WIFI_RETRY_MS.
const unsigned long WIFI_RETRY_MS = 15000;
unsigned long lastWiFiTry = 0;
bool wifiWasUp = false;
int wifiIndex = 0;          // which of WIFI_NETWORKS we're on
int serverFailsInARow = 0;  // counted in httpRequest()

void nextNetwork() {
  wifiIndex = (wifiIndex + 1) % WIFI_NETWORK_COUNT;
  lastWiFiTry = 0;          // try the next one right away
}

bool wifiUp() { return WiFi.status() == WL_CONNECTED; }

void ensureWiFi() {
  if (wifiUp() && serverFailsInARow >= SERVER_FAILS_BEFORE_SWITCH) {
    Serial.print("Wi-Fi: "); Serial.print(WIFI_NETWORKS[wifiIndex].ssid);
    Serial.println(" is connected but can't reach the server. Trying the next network.");
    serverFailsInARow = 0;
    WiFi.disconnect();
    wifiWasUp = false;
    nextNetwork();
    delay(300);
  }
  if (wifiUp()) {
    if (!wifiWasUp) {
      wifiWasUp = true;
      Serial.print("Connected, IP: ");
      Serial.println(WiFi.localIP());
      showStateIcon();
      // Tell the server where the latch really is. If a LOCK or UNLOCK was lost
      // while we were offline, the server sees the mismatch and sends it again.
      reportEvent(isLocked ? "locked" : "unlocked");
    }
    return;
  }
  if (wifiWasUp) { wifiWasUp = false; Serial.println("Wi-Fi dropped."); }
  if (lastWiFiTry && millis() - lastWiFiTry < WIFI_RETRY_MS) return;
  lastWiFiTry = millis();
  showIcon(ICON_WIFI);

  const WiFiNetwork& net = WIFI_NETWORKS[wifiIndex];
  Serial.print("Wi-Fi: joining "); Serial.println(net.ssid);
  WiFi.begin(net.ssid, net.pass);
  for (int i = 0; i < 10 && !wifiUp(); i++) delay(500);
  if (!wifiUp()) {
    Serial.print("Wi-Fi: join failed, status "); Serial.print(WiFi.status());
    Serial.println(" (for a hotspot, keep the phone on the Personal Hotspot screen)");
    showStateIcon();
    nextNetwork();
  }
}

// ---------- tiny HTTP client (plain-text responses) ----------
String httpRequest(const char* method, String path, String body) {
  if (!wifiUp()) return "ERR:no_wifi";
  WiFiClient client;
  client.setTimeout(3000);
  if (!client.connect(SERVER_HOST, SERVER_PORT)) { serverFailsInARow++; return "ERR:connect"; }
  serverFailsInARow = 0;

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

  // Read until we have the headers plus Content-Length bytes of body. Don't trust
  // client.connected() alone: over a phone hotspot it can go false before the
  // reply has arrived, which used to show up as ERR:bad_response.
  String resp = "";
  unsigned long start = millis();
  int bodyLen = -1, headerEnd = -1;
  while (millis() - start < 8000) {
    while (client.available()) resp += (char)client.read();
    if (headerEnd < 0) {
      headerEnd = resp.indexOf("\r\n\r\n");
      if (headerEnd >= 0) {
        String head = resp.substring(0, headerEnd);
        head.toLowerCase();
        int cl = head.indexOf("content-length:");
        if (cl >= 0) bodyLen = head.substring(cl + 15, head.indexOf('\r', cl)).toInt();
      }
    }
    if (headerEnd >= 0 && bodyLen >= 0 && (int)resp.length() >= headerEnd + 4 + bodyLen) break;
    if (headerEnd >= 0 && bodyLen < 0 && !client.connected() && !client.available()) break;
    delay(5);
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

// I2C bus recovery. If the R4 resets (e.g. after an upload) in the middle of an
// I2C transfer, the PN532 can be left holding SDA low and ignore everything
// until it loses power. Clocking SCL until it lets go, then sending a STOP,
// frees the bus without unplugging.
void recoverI2CBus() {
  Wire.end();   // hand the pins back from the I2C peripheral before bit-banging them
  pinMode(WIRE_SDA_PIN, INPUT_PULLUP);
  pinMode(WIRE_SCL_PIN, OUTPUT);
  for (int i = 0; i < 18 && digitalRead(WIRE_SDA_PIN) == LOW; i++) {
    digitalWrite(WIRE_SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(WIRE_SCL_PIN, HIGH); delayMicroseconds(10);
  }
  // STOP: SDA low -> high while SCL is high
  pinMode(WIRE_SDA_PIN, OUTPUT);
  digitalWrite(WIRE_SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(WIRE_SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(WIRE_SDA_PIN, HIGH); delayMicroseconds(10);
  pinMode(WIRE_SDA_PIN, INPUT);
  pinMode(WIRE_SCL_PIN, INPUT);
  Wire.begin();  // give the pins back to the I2C peripheral (a plain begin() may not)
}

// One line of facts for a failed attempt: are the I2C lines idle-high, and does
// anything answer at the PN532's address?
void printI2CDiagnostics() {
  Wire.end();
  pinMode(WIRE_SDA_PIN, INPUT);
  pinMode(WIRE_SCL_PIN, INPUT);
  int sda = digitalRead(WIRE_SDA_PIN), scl = digitalRead(WIRE_SCL_PIN);
  Wire.begin();
  Wire.beginTransmission(0x24);
  int probe = Wire.endTransmission();   // 0 = chip acknowledged its address
  int status = -1;
  if (Wire.requestFrom(0x24, 1) == 1) status = Wire.read();   // bit 0 = chip has a reply ready
  Serial.print("  I2C check: SDA="); Serial.print(sda ? "high" : "LOW (stuck)");
  Serial.print(" SCL="); Serial.print(scl ? "high" : "LOW (stuck)");
  Serial.print(" address 0x24: "); Serial.print(probe == 0 ? "answers" : "no answer");
  Serial.print(" (code "); Serial.print(probe); Serial.print(")");
  Serial.print(" status byte: "); Serial.println(status);
}

// The PN532 keeps running whatever command it had when the R4 was reset (after
// an upload that's usually "wait for a phone"), and ignores new commands until
// it finishes. An ACK frame from the host cancels the current command
// (PN532 user manual, 6.2.1.3), so send one before talking to it.
void sendPn532Cancel(unsigned int settleMs) {
  const uint8_t ack[] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00 };
  Wire.beginTransmission(0x24);   // PN532 I2C address
  Wire.write(ack, sizeof(ack));
  Wire.endTransmission();
  delay(settleMs);
}

void abortPn532Command() {
  Wire.begin();
  sendPn532Cancel(50);
  Wire.requestFrom(0x24, 32);     // read out any stale reply it was holding
  while (Wire.available()) Wire.read();
}

// Clears whatever the PN532 is stuck on (cancel, then reconfigure) so the tag
// can be armed again. Used after every phone session and whenever arming fails.
void recoverTag(const char* why) {
  abortPn532Command();
  nfc.SAMConfig();
  nfc.setPassiveActivationRetries(CARD_SEARCH_RETRIES);
  Serial.print("Tag reset ("); Serial.print(why); Serial.println(")");
}

// The Wi-Fi hardware (MAC) address, for registering the lock on GTother
// (GT's device registration page), so it can join campus Wi-Fi on its own.
void printMacAddress() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  // Library versions disagree on the byte order, so print both. The real one
  // starts with an Espressif prefix (the R4's Wi-Fi chip); registering both is fine.
  Serial.print("Wi-Fi MAC address (register on GTother): ");
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) Serial.print('0');
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(':');
  }
  Serial.print("   or reversed: ");
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 0x10) Serial.print('0');
    Serial.print(mac[i], HEX);
    if (i) Serial.print(':');
  }
  Serial.println();
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
  // An upload resets the R4 but not the PN532, which can be left mid-command
  // and deaf. So every attempt pulses its reset line (held longer than the
  // datasheet minimum) and wakes it again before asking for the firmware.
  pinMode(PN532_RESET, OUTPUT);
  uint32_t version = 0;
  for (int attempt = 1; !version; attempt++) {
    digitalWrite(PN532_RESET, LOW);
    delay(100);
    digitalWrite(PN532_RESET, HIGH);
    delay(400);
    recoverI2CBus();
    abortPn532Command();
    nfc.begin();
    version = nfc.getFirmwareVersion();
    if (!version) {
      Serial.print("PN532 not found (attempt "); Serial.print(attempt);
      Serial.println("). If this repeats, unplug USB for 5 s; check the shield is seated and set to I2C.");
      printI2CDiagnostics();
      showIcon(ICON_X);
      delay(1500);
    }
  }
  Serial.print("PN532 firmware ");
  Serial.print((version >> 16) & 0xFF); Serial.print('.'); Serial.println((version >> 8) & 0xFF);
  nfc.SAMConfig();
  nfc.setPassiveActivationRetries(CARD_SEARCH_RETRIES);

  // Target mode: what the phone sees when it taps the lock.
  tag.setUid(tagUid);
  // setNdefFile() adds the 2-byte length itself, so hand it the message only.
  // Passing the whole buffer doubled the length header and iOS rejected the tag.
  tag.setNdefFile(ndefFile + 2, ndefFileLength - 2);
  tag.setTagWriteable(false);   // phones can read the code but not overwrite it

  setLocked(true);
  printMacAddress();
  ensureWiFi();
  reportEvent("locked");
  Serial.println("Ready. Tap a phone or a card, or unlock from the app.");
}

// Returns the tapped card's UID as uppercase hex, or "" if nothing was there.
String readCardUid() {
  uint8_t uid[7];
  uint8_t uidLength = 0;
  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, CARD_WINDOW_MS)) {
    sendPn532Cancel(2);   // make sure the chip isn't still searching: keep it idle between searches
    return "";
  }

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
  } else if (reply == "DENY:end_in_app") {
    Serial.println("Ride in progress. End it in the app (with the bike photo), not by tapping.");
    showIcon(ICON_X); delay(800); showStateIcon();
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
  // Without card reading the PN532 stays armed as a tag the whole time and we
  // only check in on it; with card reading it has to be re-armed each window.
  bool phoneServed = !TAG_EMULATION ? false
                   : CARD_READING ? tag.emulate(TAG_WINDOW_MS) : tag.poll(10);
  if (TAG_EMULATION && !CARD_READING && tag.lastPollResult == -10) {
    tag.lastPollResult = 0;
    recoverTag("chip stopped answering");
  }
  if (phoneServed) {
    Serial.print("A phone read the lock. Exchange:");
    for (uint8_t i = 0; i < emuTraceLen; i++) {
      Serial.print(" [");
      for (uint8_t j = 0; j < 6; j++) {
        if (emuTrace[i][j] < 0x10) Serial.print('0');
        Serial.print(emuTrace[i][j], HEX);
        if (j == 3) Serial.print(" ->");
        if (j < 5) Serial.print(' ');
      }
      Serial.print("]");
    }
    Serial.print("  end="); Serial.println(emuEndStatus);
    if (emuEndStatus != 0) recoverTag("phone session ended with an error");
    showIcon(ICON_CHECK);
    delay(400);
    showStateIcon();
  }
  // Heartbeat so the monitor shows whether the tag is actually armed.
  static unsigned long lastBeat = 0;
  if (TAG_EMULATION && millis() - lastBeat > 5000) {
    lastBeat = millis();
    Serial.print("Tag status: armed="); Serial.print(tag.isArmed() ? "yes" : "NO");
    Serial.print(" lastPoll="); Serial.print(tag.lastPollResult);
    Serial.print(" armCount="); Serial.println(tag.armCount);
  }

  // 2) Look for a keycard or fob (only if CARD_READING is on).
  String uidHex = "";
  if (CARD_READING) {
    if (TAG_EMULATION) nfc.SAMConfig();   // emulate() leaves the chip in target mode; back to reader mode
    uidHex = readCardUid();
  }
  if (uidHex.length()) {
    // Ignore the same card held on the reader for 3 seconds
    if (uidHex != lastUid || millis() - lastUidAt > 3000) {
      lastUid = uidHex;
      lastUidAt = millis();
      Serial.println("Card: " + uidHex);
      if (uidHex != ALLOWED_CARD_UID) {
        Serial.println("Invalid card. This lock only accepts " + String(ALLOWED_CARD_UID) + ".");
        showIcon(ICON_X); delay(1200); showStateIcon();
      } else {
        handleTapReply(httpRequest("POST", "/api/device/tap",
                                   String("lock=") + LOCK_ID + "&uid=" + uidHex));
      }
    }
  }

  // 3) Poll the server for app-triggered commands
  if (wifiUp() && millis() - lastPoll > POLL_MS) {
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
