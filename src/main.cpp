#include <Arduino.h>

/* Aaron here - I'm defining the front as the short edge that the arduino is closest to.
 *
 * A and C pins (left-hand motors) are defined by the two pairs of jumpers (each pair has a red wire and black wire) on the bottom board.
 * They link the arduino's digital pins to motor pins (near the screw terminals that the motor power wires come from) 
 *                                                    (they're confusingly labelled 6,7,4,5)
 * I've plugged them into 12,11,3,2 respectively.
 * 
 * B and D pins (right-hand motors) are defined by the little 1x2 pin connectors that tie adjacent pins together on the top board.
 * They basically let you make a binary choice between two pre-defined arduino digital pins for each pin. 
 * I have them set to 6,7,4,5.
 * (careful, these connectors are fragile. You can grab the inner and outer edges of one side with some tweasers.)
 * 
 * When fiddling with this, as always, avoid pins used for built-in purposes. 
 */
#define A_DIR 12    // A : front left
#define A_PWM 11
#define B_DIR  7    // B : front right
#define B_PWM  6
#define C_DIR  2    // C : back left
#define C_PWM  3
#define D_DIR  4    // D : back right
#define D_PWM  5


void setup() {
  Serial.begin(9600);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(A_DIR, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(B_DIR, OUTPUT);
  pinMode(B_PWM, OUTPUT);
  pinMode(C_DIR, OUTPUT);
  pinMode(C_PWM, OUTPUT);
  pinMode(D_DIR, OUTPUT);
  pinMode(D_PWM, OUTPUT);
}

/// right-hand and left-hand motors go in opposite directions
void setMotorDir(int dirPin, bool forwards) {
  if (dirPin == A_DIR || dirPin == C_DIR) {
    digitalWrite(dirPin, forwards);
  }
  else if (dirPin == B_DIR || dirPin == D_DIR) {
    digitalWrite(dirPin, !forwards);
  }
  // else ??? that's not a motor dir pin
}

void stopDriving() {
  digitalWrite(A_PWM, 0);
  digitalWrite(B_PWM, 0);
  digitalWrite(C_PWM, 0);
  digitalWrite(D_PWM, 0);
}

/// neg power for backwards, pos power for forwards
void moveY(int power) {
  if (power == 0) return;

  bool forwards = power > 0;
  setMotorDir(A_DIR, forwards);
  setMotorDir(B_DIR, forwards);
  setMotorDir(C_DIR, forwards);
  setMotorDir(D_DIR, forwards);

  power = abs(power);
  analogWrite(A_PWM, power);
  analogWrite(B_PWM, power);
  analogWrite(C_PWM, power);
  analogWrite(D_PWM, power);
}

/// neg power for left, pos power for right
void moveX(int power) {
  if (power == 0) return;

  setMotorDir(A_DIR, power > 0);
  setMotorDir(B_DIR, power < 0);
  setMotorDir(C_DIR, power < 0);
  setMotorDir(D_DIR, power > 0);

  power = abs(power);
  analogWrite(A_PWM, power);
  analogWrite(B_PWM, power);
  analogWrite(C_PWM, power);
  analogWrite(D_PWM, power);
}

int p = 50;;
int t = 500;

void loop() {
  moveY(p);
  delay(t);
  stopDriving();
  delay(1000);

  moveX(p);
  delay(t);
  stopDriving();
  delay(1000);

  moveY(-p);
  delay(t);
  stopDriving();
  delay(1000);

  moveX(-p);
  delay(t);
  stopDriving();
  delay(1000);
}
