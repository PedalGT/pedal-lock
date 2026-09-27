/*
  Pedal — STEP 1: hardware check. No Wi-Fi, no server, no accounts.

  Proves three things before anything else is plugged in:
    1. the PN532 shield is found and talking over I2C
    2. the servo on D9 actually swings when told to
    3. the PN532 can pretend to be an NFC tag that your iPhone reads

  Watch the Serial Monitor at 115200.

  Two ways to trigger the "unlock" swing:
    - tap any NFC card or fob on the shield  -> prints the card's UID, swings
    - hold an iPhone against the shield       -> phone reads "PEDAL:TEST01", swings
      (the phone half needs the Pedal app built with the NFC capability, which
       needs a paid Apple account. The card half works with no account at all.)

  Library: Seeed Studio PN532, NOT Adafruit.
    https://github.com/Seeed-Studio/PN532  ->  Code > Download ZIP
    Arduino IDE > Sketch > Include Library > Add .ZIP Library
  Adafruit's library cannot do target mode, which is the whole point of step 1.

  Wiring, as you have it now:
    PN532 shield stacked on the R4, I2C mode. IRQ = D2, RESET = D3.
    Servo signal = D9, servo red = 5V, servo brown/black = GND.
*/

#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <emulatetag.h>
#include <Servo.h>

// ---------------- CONFIG ----------------
const int  SERVO_PIN      = 9;
const int  LOCKED_ANGLE   = 0;     // where the latch sits closed
const int  UNLOCKED_ANGLE = 90;    // where the latch sits open
const char* TEST_CODE     = "TEST01";   // what the phone will read off the lock

const uint16_t TAG_WINDOW_MS  = 350;    // time spent pretending to be a tag
const uint16_t CARD_WINDOW_MS = 150;    // time spent looking for a card
// ----------------------------------------

#define PN532_RESET 3   // the shield ties the PN532's reset line to D3

PN532_I2C  pn532i2c(Wire);
PN532      nfc(pn532i2c);     // initiator mode: reading cards
EmulateTag tag(pn532i2c);     // target mode: being read by a phone
Servo      latch;

uint8_t tagUid[3] = { 0x0F, 0xED, 0xA1 };
uint8_t ndefFile[64];
uint16_t ndefFileLength = 0;

String lastUid = "";
unsigned long lastUidAt = 0;

// ---------- servo ----------
void motorSetup() {
  latch.attach(SERVO_PIN);
  latch.write(LOCKED_ANGLE);
  delay(500);
}

void motorUnlock() {
  latch.write(UNLOCKED_ANGLE);
  delay(600);          // give the horn time to actually get there
}

void motorLock() {
  latch.write(LOCKED_ANGLE);
  delay(600);
}

// ---------- the NDEF message the phone reads ----------
// One NDEF Text record holding "PEDAL:<CODE>".
void buildNdefFile(const char* code) {
  String text = String("PEDAL:") + code;

  uint8_t payloadLength = 1 + 2 + text.length();   // status byte + "en" + text
  uint8_t i = 2;                                   // room for the 2-byte NLEN

  ndefFile[i++] = 0xD1;      // MB | ME | SR | TNF = NFC Forum well-known
  ndefFile[i++] = 0x01;      // type length: "T"
  ndefFile[i++] = payloadLength;
  ndefFile[i++] = 'T';       // record type: Text
  ndefFile[i++] = 0x02;      // UTF-8, 2-byte language code
  ndefFile[i++] = 'e';
  ndefFile[i++] = 'n';
  for (uint16_t n = 0; n < text.length(); n++) ndefFile[i++] = text[n];

  uint16_t messageLength = i - 2;
  ndefFile[0] = (messageLength >> 8) & 0xFF;
  ndefFile[1] = messageLength & 0xFF;
  ndefFileLength = i;

  Serial.println("Emulating tag: " + text);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nPedal step 1: servo + PN532 check");

  motorSetup();
  Serial.println("Servo attached on D9 and moved to the locked position.");

  buildNdefFile(TEST_CODE);

  // The shield wires the PN532's reset line to D3. Adafruit's library pulses it;
  // the Seeed library never touches it, so do it here or the chip may sit held
  // in reset and report "not found".
  pinMode(PN532_RESET, OUTPUT);
  digitalWrite(PN532_RESET, LOW);
  delay(10);
  digitalWrite(PN532_RESET, HIGH);
  delay(50);

  nfc.begin();
  uint32_t version = 0;
  while (!(version = nfc.getFirmwareVersion())) {
    Serial.println("PN532 not found. Check the shield is seated and set to I2C (SEL0 = 1, SEL1 = 0).");
    delay(2000);
  }
  Serial.print("Found PN532, firmware ");
  Serial.print((version >> 16) & 0xFF); Serial.print('.');
  Serial.println((version >> 8) & 0xFF);

  nfc.SAMConfig();
  tag.setUid(tagUid);
  tag.setNdefFile(ndefFile + 2, ndefFileLength - 2);   // library adds the length header itself

  Serial.println("Ready. Tap a card, or hold an iPhone against the shield.");
}

void swing(const char* why) {
  Serial.print(why);
  Serial.println(" -> unlocking");
  motorUnlock();
  delay(1500);
  motorLock();
  Serial.println("Locked again. Waiting.");
}

void loop() {
  // 1) Be a tag for a moment, so a phone can read us.
  if (tag.emulate(TAG_WINDOW_MS)) {
    swing("A phone read the lock");
  }
  nfc.SAMConfig();     // emulate() leaves the chip in target mode; go back

  // 2) Look for a card or fob.
  uint8_t uid[7];
  uint8_t uidLength = 0;
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, CARD_WINDOW_MS)) {
    String uidHex = "";
    for (uint8_t i = 0; i < uidLength; i++) {
      if (uid[i] < 0x10) uidHex += "0";
      uidHex += String(uid[i], HEX);
    }
    uidHex.toUpperCase();

    // Ignore the same card sitting on the reader
    if (uidHex != lastUid || millis() - lastUidAt > 3000) {
      lastUid = uidHex;
      lastUidAt = millis();
      Serial.println("====================");
      Serial.println("Card UID: " + uidHex);
      Serial.println("====================");
      swing("Card tapped");
    }
  }
}
