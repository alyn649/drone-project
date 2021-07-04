#include <Arduino.h>
#include "DShot.h"
#include <TinyMPU6050.h>

#define base 300
#define maxT 600

// Object for imu comms
MPU6050 imu (Wire);

// Objects for each motor
volatile DShot motorA;
volatile DShot motorB;
volatile DShot motorC;
volatile DShot motorD;

bool armed = false;
bool killed = false;

volatile int throttle = 0;

volatile double gx, gy, gz;
volatile double ax, ay, az;

double kp1 = 2; // angle contorller

double kp = 0.5;
double ki = 0.015;
double kd = 0;

volatile double axErr, ayErr, azErr;

volatile double gxRef, gyRef, gzRef;
volatile double gxErr, gyErr, gzErr;
volatile double gxErrT, gyErrT, gzErrT;

volatile double effX, effY, effZ;

volatile int effA, effB, effC, effD;

unsigned long startTime;
unsigned long t;

//ISR(TIMER1_COMPA_vect) {
//
//}

int sat(int val, int low, int up) {
  if(val < low) return low;
  if(val > up) return up;
  return val;
}

void setup() {
  Serial.begin(115200);
  
  // Attach each motor
  motorA.attach(6);
  motorB.attach(5);
  motorC.attach(4);
  motorD.attach(3);

  pinMode(13, OUTPUT);

  TCCR1B |= (1 << CS11) | (1 << CS10) | (1 << WGM12); // Prescaler of 64

  //initilise imu
  imu.Initialize();

  imu.RegisterWrite(MPU6050_CONFIG, 0x03); // configure low pass filter

  delay(5000);

  imu.Calibrate();

  // Begin control loop
//  TCNT1 = 0;
//  OCR1A = 1250; // results in 5.00ms delay
//  TIMSK1 |= (1 << OCIE1A);
//  sei();

  startTime = millis();
}

void loop() {
    // Throttle logic

    t = millis() - startTime;

    if(t < 2000) {
      armed = false;
    } else if(t < 10000) {
      armed = true;
      if(throttle < base) {
        throttle++;
      }
    } else {
      if(throttle > 0) {
        throttle--;
      } else {
        armed = false;
      }
    }
    
    // Control system
    imu.Execute();
    // get imu values
    gx = imu.GetGyroX();
    gy = imu.GetGyroY();
    gz = imu.GetGyroZ();

    ax = imu.GetAngX();
    ay = imu.GetAngY();
    az = imu.GetAngZ();

    // calc angle error with 0 ref
    axErr = 0 - ax;
    ayErr = 0 - ay;
    azErr = 0 - az;

    // calc ref value for rate controller
    gxRef = (axErr * kp1);
    gyRef = (ayErr * kp1);
    gzRef = (azErr * kp1);

    // compute error with angle fed ref
    gxErr = gxRef - gx;
    gyErr = gyRef - gy;
    gzErr = gzRef - gz;

    // intergrate error
    gxErrT += gxErr;
    gyErrT += gyErr;
    gzErrT += gzErr;

    // calculate final effort
    effX = (gxErr * kp) + (gxErrT * ki);
    effY = (gyErr * kp) + (gyErrT * ki);
    effZ = (gzErr * kp) + (gzErrT * ki);

    if(ax > 15 || ax < -15) killed = true;
    if(ay > 15 || ay < -15) killed = true;

    // apply to motors
    if(armed && !killed) {
      effA = sat(throttle - effX - effY - effZ, 48, maxT);
      effB = sat(throttle + effX - effY + effZ, 48, maxT);
      effC = sat(throttle - effX + effY + effZ, 48, maxT);
      effD = sat(throttle + effX + effY - effZ, 48, maxT);
    } else {
      effA = 48;
      effB = 48;
      effC = 48;
      effD = 48;
    }
    
    motorA.setThrottle(effA);
    motorB.setThrottle(effB);
    motorC.setThrottle(effC);
    motorD.setThrottle(effD);

    digitalWrite(13, LOW);
    
    DShot::callSendData(); // bit bang the motor commands
}
