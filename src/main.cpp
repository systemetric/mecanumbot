#include <Arduino.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include <Omni4WD.h>


#define A_DIR        2
#define A_PWM        3
#define A_ENCODER_A 12
#define A_ENCODER_B 13

#define B_DIR        4
#define B_PWM        5
#define B_ENCODER_A A0
#define B_ENCODER_B A1

#define C_DIR       11
#define C_PWM       10
#define C_ENCODER_A A4
#define C_ENCODER_B A5

#define D_DIR        8
#define D_PWM        9
#define D_ENCODER_A  6
#define D_ENCODER_B  7

#define BAUDRATE 9600
#define WHEEL_CIRCUMFERENCE 188
#define WHEELSPAN 225


// interrupts
irqISR(irq1, isr1);
irqISR(irq2, isr2);
irqISR(irq3, isr3);
irqISR(irq4, isr4);
// MotorWheel objects represent a motor with a wheel attached, so
// wheelA.setSpeedMMPS(mmps) can use the circumference of the wheel (and gear ratio, etc.) to calculate the speed in rpm
MotorWheel wheelA(A_PWM, A_DIR, A_ENCODER_A, A_ENCODER_B, &irq1);
MotorWheel wheelB(B_PWM, B_DIR, B_ENCODER_A, B_ENCODER_B, &irq2);
MotorWheel wheelC(C_PWM, C_DIR, C_ENCODER_A, C_ENCODER_B, &irq3);
MotorWheel wheelD(D_PWM, D_DIR, D_ENCODER_A, D_ENCODER_B, &irq4);

// object to represent the whole 4 wheel driving setup
Omni4WD car(&wheelA, &wheelB, &wheelC, &wheelD, WHEELSPAN);


void setup() {
  Serial.begin(BAUDRATE);

  // set wheel circumferences
  wheelA.setCirMM(WHEEL_CIRCUMFERENCE);
  wheelB.setCirMM(WHEEL_CIRCUMFERENCE);
  wheelC.setCirMM(WHEEL_CIRCUMFERENCE);
  wheelD.setCirMM(WHEEL_CIRCUMFERENCE);

  // init the PID settings
  const float p = 0.3;
  const float i = 0.1;
  const float d = 0.0;
  const int t = 100;
  car.PIDEnable(p, i, d, t);
}

int i = 0;
unsigned long nextTime = 0;
unsigned short actionTime = 100;
const unsigned short circumferenceMS = 10000;

float rad = 0;
float radPerChange = M_PI * 2 / (circumferenceMS / actionTime);
int mmps = 40;


void loop() {
  car.PIDRegulate();    // do PID update every loop call to adjust motor speeds

  if (millis() > nextTime) {
    i++;
    nextTime = millis() + actionTime;
    Serial.print("next time: ");
    Serial.print(nextTime);

    rad += radPerChange;
    while (rad > M_PI * 2) {
      rad -= M_PI * 2;
    }

    Serial.print("    ");
    Serial.println(rad);
    car.setCarMove(mmps, rad);
  }
}
