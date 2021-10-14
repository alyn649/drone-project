#include <Arduino.h>
#include "DShot.h"
#include <TinyMPU6050.h>

#define base 700
#define maxT 1000

// uncomment for debugging output DO NOT FLY ENABLED
//#define debug 1

// Object for imu comms
MPU6050 imu (Wire);

// Objects for each motor
volatile DShot motorA;
volatile DShot motorB;
volatile DShot motorC;
volatile DShot motorD;

bool armed = false;
bool killed = false;

int throttle = 0;
int throttleLast = 0;

// controller params

// angle contorller
double kp1 = 2;

// rate gyro controller
double kp = 0.5;
double ki = 0.015;
double kd = 0;

// false satuation cap to reduce windup
int satEff = 1300;

// controller internal variables 

volatile double gx, gy, gz;
volatile double ax, ay, az;

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
  
  #ifdef debug
    Serial.begin(115200);
    Serial.println("DEBUG_MODE");
  #endif
  
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

  delay(2000);

  #ifdef debug
    Serial.print("calibrating...");
  #endif

  imu.Calibrate();

  #ifdef debug
    Serial.println("done");
  #endif

  // Begin control loop
//  TCNT1 = 0;
//  OCR1A = 1250; // results in 5.00ms delay
//  TIMSK1 |= (1 << OCIE1A);
//  sei();

  startTime = millis();
}

void loop() {
    t = millis() - startTime;

    #ifdef debug
      Serial.print("t (ms) = ");
      Serial.print(t);
    #endif

    // Throttle logic

    // time based throttle logic 
    /*
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
    */

    // analog input based logic
    armed = true;
    throttle = map(analogRead(A0), 0, 1023, 0, base);

    // Anti throttle spike logic
    if(abs(throttle - throttleLast) > 30) killed = true;
    throttleLast = throttle;

    #ifdef debug
      Serial.print(", throttle = ");
      Serial.print(throttle);
      Serial.print("/2000");
    #endif
    
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
    gxErrT = sat(gxErrT + gxErr, -satEff, satEff);
    gyErrT = sat(gyErrT + gyErr, -satEff, satEff);
    gzErrT = sat(gzErrT + gzErr, -satEff, satEff);

    // calculate final effort
    effX = (gxErr * kp) + (gxErrT * ki);
    effY = (gyErr * kp) + (gyErrT * ki);
    effZ = (gzErr * kp) + (gzErrT * ki);

    if(ax > 15 || ax < -15) killed = true;
    if(ay > 15 || ay < -15) killed = true;

    // KILL YAW
    effZ = 0;

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
    
    #ifdef debug
      Serial.print(", armed = ");
      Serial.print(armed);
      Serial.print(", killed = ");
      Serial.print(killed);
      
      Serial.println("");
    #endif
    
    DShot::callSendData(); // bit bang the motor commands
}
