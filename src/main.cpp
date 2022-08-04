#include <Arduino.h>
#include <PID_Beta6.h>
#include <MotorWheel.h>
#include <Omni4WD.h>


#define WHEEL_COUNT 4

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


irqISR(irq1, isr1);
irqISR(irq2, isr2);
irqISR(irq3, isr3);
irqISR(irq4, isr4);
MotorWheel wheelA(A_PWM, A_DIR, A_ENCODER_A, A_ENCODER_B, &irq1);
MotorWheel wheelB(B_PWM, B_DIR, B_ENCODER_A, B_ENCODER_B, &irq2);
MotorWheel wheelC(C_PWM, C_DIR, C_ENCODER_A, C_ENCODER_B, &irq3);
MotorWheel wheelD(D_PWM, D_DIR, D_ENCODER_A, D_ENCODER_B, &irq4);

MotorWheel* wheels[WHEEL_COUNT] = {&wheelA, &wheelB, &wheelC, &wheelD};


void setup() {
  Serial.begin(9600);

  wheelA.setCirMM(188);
  wheelB.setCirMM(188);
  wheelC.setCirMM(188);
  wheelD.setCirMM(188);

  float p = 0.3;
  float i = 0.1;
  float d = 0.0;
  int t = 100;

  wheelA.PIDEnable(p, i, d, t);
  wheelB.PIDEnable(p, i, d, t);
  wheelC.PIDEnable(p, i, d, t);
  wheelD.PIDEnable(p, i, d, t);

  wheelA.setSpeedMMPS(30);
  wheelB.setSpeedMMPS(-20);
  wheelC.setSpeedMMPS(20);
  wheelD.setSpeedMMPS(-20);
}

bool a = true;
int i = 0;
int nextTime = 0;

void loop() {
  wheelA.PIDRegulate();
  wheelB.PIDRegulate();
  wheelC.PIDRegulate();
  wheelD.PIDRegulate();

  if (millis() > nextTime) {
    int target = a ? -600 : 600;
    a = !a;
    nextTime = millis() + 4000;
    Serial.print(nextTime);

    Serial.println();
    Serial.println();
    Serial.println(target);
    Serial.println(target);
    Serial.println(target);
    Serial.println(target);


    for (int j = 0; j < sizeof(wheels)/sizeof(wheels[0]); j++) {
      Serial.print(j);
      Serial.print("   ");
      Serial.println((*wheels[j]).getSpeedRPM());
      (*wheels[j]).setSpeedRPM(target);
    }
  }
}
