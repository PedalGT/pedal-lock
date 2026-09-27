/*
  Keycard reader: tap a card on the PN532 shield and it prints the card's ID.

  Hardware: Arduino UNO R4 WiFi + PN532 NFC shield, I2C mode (SEL0 = ON, SEL1 = OFF).
  Library:  Seeed Studio PN532 (already in ~/Arduino/libraries/PN532).
  Output:   Serial Monitor at 115200.

  Reads 13.56 MHz cards (ISO 14443A: MIFARE Classic / Ultralight / DESFire,
  NTAG, most campus and hotel cards). It cannot read 125 kHz proximity cards
  (e.g. HID Prox): those need a different reader.
*/

#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>

#define PN532_RESET 3        // the shield ties the PN532 reset line to D3

PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);

String lastUid = "";
unsigned long lastSeenAt = 0;

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

// After an upload the PN532 may still be running an old command and ignore us.
// An ACK frame cancels it (PN532 user manual 6.2.1.3).
void sendPn532Cancel(unsigned int settleMs) {
  const uint8_t ack[] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00 };
  Wire.beginTransmission(0x24);
  Wire.write(ack, sizeof(ack));
  Wire.endTransmission();
  delay(settleMs);
}

void abortPn532Command() {
  Wire.begin();
  sendPn532Cancel(50);
  Wire.requestFrom(0x24, 32);     // read out any stale reply
  while (Wire.available()) Wire.read();
}

String toHex(const uint8_t* bytes, uint8_t len, const char* sep) {
  String out = "";
  for (uint8_t i = 0; i < len; i++) {
    if (i && sep[0]) out += sep;
    if (bytes[i] < 0x10) out += "0";
    out += String(bytes[i], HEX);
  }
  out.toUpperCase();
  return out;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nKeycard reader");

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
      Serial.println("). If this repeats, unplug USB for 10 s and check the shield is seated, I2C mode.");
      printI2CDiagnostics();
      delay(1500);
    }
  }
  Serial.print("PN532 found, firmware ");
  Serial.print((version >> 16) & 0xFF); Serial.print('.'); Serial.println((version >> 8) & 0xFF);

  nfc.setPassiveActivationRetries(0x05);   // not 0xFF ("forever"): keeps the chip idle between searches
  nfc.SAMConfig();
  Serial.println("Ready. Tap a card.");
}

void loop() {
  uint8_t uid[10];
  uint8_t uidLength = 0;

  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 500)) {
    sendPn532Cancel(2);
  } else {
    String id = toHex(uid, uidLength, "");
    // Don't repeat while the same card sits on the reader.
    if (id != lastUid || millis() - lastSeenAt > 3000) {
      unsigned long decimal = 0;   // first 4 bytes, little-endian: how many door systems print it
      for (int i = min((int)uidLength, 4) - 1; i >= 0; i--) decimal = (decimal << 8) | uid[i];

      Serial.println("==============================");
      Serial.print("Card ID:   "); Serial.println(id);
      Serial.print("Bytes:     "); Serial.println(toHex(uid, uidLength, " "));
      Serial.print("Length:    "); Serial.print(uidLength); Serial.println(" bytes");
      Serial.print("Decimal:   "); Serial.println(decimal);
      Serial.println(uidLength == 4 && uid[0] == 0x08
        ? "Note:      starts with 08, a random ID (phones, some cards). It changes every tap."
        : "Tap again: the ID should be the same every time.");
      Serial.println("==============================");
    }
    lastUid = id;
    lastSeenAt = millis();
  }
  delay(100);
}
