/*
  Servo calibration: find the lock's LOCKED and UNLOCKED angles.

  Servo signal on D9 (same as the lock). Open the Serial Monitor at 115200 and set
  the line ending (bottom right) to "Newline". Then type:

    35        move to 35 degrees (0-180)
    +  /  -   nudge by 1 degree
    ++ / --   nudge by 5 degrees
    l         remember the current angle as LOCKED
    u         remember the current angle as UNLOCKED
    t         test: swing between LOCKED and UNLOCKED 3 times
    ?         show where it is and what's saved

  When both positions work, send the two numbers so they go into pedal_lock_test.
*/
#include <Servo.h>

const int SERVO_PIN = 9;
Servo servo;
int current = 90;
int lockedAngle = -1, unlockedAngle = -1;

// Move in small steps so the latch doesn't slam while you're testing.
void moveTo(int target) {
  target = constrain(target, 0, 180);
  int step = target > current ? 1 : -1;
  while (current != target) { current += step; servo.write(current); delay(8); }
  Serial.print("At "); Serial.print(current); Serial.println(" degrees");
}

void status() {
  Serial.print("Now: "); Serial.print(current);
  Serial.print("   LOCKED: "); lockedAngle < 0 ? Serial.print("not set") : Serial.print(lockedAngle);
  Serial.print("   UNLOCKED: "); unlockedAngle < 0 ? Serial.println("not set") : Serial.println(unlockedAngle);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  servo.attach(SERVO_PIN);
  servo.write(current);
  Serial.println("\nServo calibration. Type an angle (0-180), + / - to nudge, l = save LOCKED, u = save UNLOCKED, t = test, ? = status.");
  status();
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "+") moveTo(current + 1);
  else if (cmd == "-") moveTo(current - 1);
  else if (cmd == "++") moveTo(current + 5);
  else if (cmd == "--") moveTo(current - 5);
  else if (cmd == "l") { lockedAngle = current; Serial.print("LOCKED saved: "); Serial.println(lockedAngle); }
  else if (cmd == "u") { unlockedAngle = current; Serial.print("UNLOCKED saved: "); Serial.println(unlockedAngle); }
  else if (cmd == "?") status();
  else if (cmd == "t") {
    if (lockedAngle < 0 || unlockedAngle < 0) { Serial.println("Save both first: move to each position and type l or u."); return; }
    for (int i = 0; i < 3; i++) {
      Serial.println("Unlocking..."); moveTo(unlockedAngle); delay(1200);
      Serial.println("Locking...");   moveTo(lockedAngle);   delay(1200);
    }
    Serial.print("Done. Put these in the lock sketch: LOCKED_ANGLE = "); Serial.print(lockedAngle);
    Serial.print(", UNLOCKED_ANGLE = "); Serial.println(unlockedAngle);
  }
  else if (isDigit(cmd[0])) moveTo(cmd.toInt());
  else Serial.println("Try a number (0-180), + / -, l, u, t or ?");
}
