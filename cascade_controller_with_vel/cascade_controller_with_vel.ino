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

// velocity controller
double kp3 = 2;
double ki3 = 0;
double kd3 = 0;

// acceleration controller
double kp2 = 15;
double ki2 = 0;
double kd2 = 0;

// angle contorller
double kp1 = 2;
double ki1 = 0;
double kd1 = 0;

// rate gyro controller
double kp = 0.5;
double ki = 0.015;
double kd = 0;

// false satuation cap to reduce windup
int satEff = 1300;

// controller internal variables 

volatile double vx, vy, vz; // velocity read
volatile double ax, ay, az; // accleration read

volatile double tx, ty, tz; // tilt (angle) read
volatile double gx, gy, gz; // gyro read

volatile double vxErr, vyErr, vzErr;

volatile double axRef, ayRef, azRef;
volatile double axErr, ayErr, azErr;

volatile double txRef, tyRef, tzRef;
volatile double txErr, tyErr, tzErr;

volatile double gxRef, gyRef, gzRef;
volatile double gxErr, gyErr, gzErr;
volatile double gxErrT, gyErrT, gzErrT; 

volatile double effX, effY, effZ;

volatile int effA, effB, effC, effD;

unsigned long startTime;
unsigned long t;
unsigned long tLast = 0;
double tCycle;

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

  digitalWrite(13, HIGH);
  
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

  digitalWrite(13, LOW);
  startTime = millis();
}

void loop() {

    // Timing control
    t = millis() - startTime;
    tCycle = (double)(t - tLast) / 1000.0;
    tLast = t;
    
    #ifdef debug
      Serial.print("t (ms) = ");
      Serial.print(t);
    #endif

    // Throttle logic

    // time based throttle logic 
    /*
    if(t < 1000) {
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

    tx = imu.GetAngX();
    ty = imu.GetAngY();
    tz = imu.GetAngZ();

    ax = imu.GetAccX();
    ay = imu.GetAccY();
    az = imu.GetAccZ();

    // intergrate velocity
    vx += (ax * tCycle);
    vy += (ax * tCycle);
    vz += (az * tCycle);

    // VELOCITY CONTROLLER
    vxErr = 0 - vx;
    vyErr = 0 - vy;
    vzErr = 0 - vz;

    axRef = (vxErr * kp3);
    ayRef = (vyErr * kp3);
    azRef = (vzErr * kp3);

    // ACCELERATION CONTROLLER
    /* notes
     *  positive roll in x => positive y motion
     *  positive roll in y => negative x motion
     */

    axErr = axRef - ax;
    ayErr = ayRef - ay;
    azErr = azRef - az;

    txRef = (ayErr * kp2);
    tyRef = -(axErr * kp2);
    
    tzRef = 0; // no vertical control yet.

    // TILT CONTROLLER
    // calc angle error with 0 ref
    txErr = txRef - tx;
    tyErr = tyRef - ty;
    tzErr = tzRef - tz;

    // calc ref value for rate controller
    gxRef = (txErr * kp1);
    gyRef = (tyErr * kp1);
    gzRef = (tzErr * kp1);

    // RATE CONTROLLER
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

    // kill on over 15 degree roll. (safety)
    if(tx > 15 || tx < -15) killed = true;
    if(ty > 15 || ty < -15) killed = true;

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
      digitalWrite(13, HIGH);
    }

    // push to motor control objects
    motorA.setThrottle(effA);
    motorB.setThrottle(effB);
    motorC.setThrottle(effC);
    motorD.setThrottle(effD);
    
    #ifdef debug
      Serial.print(", armed = ");
      Serial.print(armed);
      Serial.print(", killed = ");
      Serial.print(killed);
      
      Serial.println("");
    #endif

    // bit bang the motor commands
    DShot::callSendData(); 
}
