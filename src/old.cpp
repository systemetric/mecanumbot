//#include <Arduino.h>

/* Aaron here - I'm defining the front as the short edge that the arduino is closest to.
 *
 * A and C pins (left-hand motors) are defined by the two pairs of jumpers (each pair has a red wire and black wire) on the bottom board.
 * They link the arduino's digital pins to motor pins (near the screw terminals that the motor power wires come from) 
 *                                                    (they're confusingly labelled 6,7,4,5)
 * 
 * B and D pins (right-hand motors) are defined by the little 1x2 pin connectors that tie adjacent pins together on the top board.
 * They basically let you make a binary choice between two pre-defined arduino digital pins for each pin. 
 * (careful, these connectors are fragile. You can grab the inner and outer edges of one side with some tweasers.)
 * 
 * When fiddling with this, as always, avoid pins used for built-in purposes. 
 * 
 *    front
 * A ------ B
 * | ard    |
 * | uino   |
 * |        |   right
 * |    batt|
 * |     ery|
 * C -----  D
 * 
 */
/*
#define A_DIR  7    // A : front left
#define A_PWM  6
#define B_DIR  8    // B : front right
#define B_PWM  9
#define C_DIR  4    // C : back left
#define C_PWM  5
#define D_DIR 11    // D : back right
#define D_PWM 10

int dirPins[4] = {A_DIR, B_DIR, C_DIR, D_DIR};
int pwmPins[4] = {A_PWM, B_PWM, C_PWM, D_PWM};

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

void doMotorTest(int p, int t) {
  for (int i = 0; i < sizeof(dirPins)/sizeof(dirPins[0]); i++) {
    int dir = dirPins[i];
    int pwm = pwmPins[i];
    setMotorDir(dir, true);
    analogWrite(pwm, p);
    delay(t);
    stopDriving();
    delay(t/5);
    setMotorDir(dir, false);
    analogWrite(pwm, p);
    delay(t);
    stopDriving();
    delay(t/5);
  }
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

int p = 100;
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
*/