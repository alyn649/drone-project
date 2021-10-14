#include <Arduino.h>
#include "DShot.h"
#include <TinyMPU6050.h>

#define base 700
#define maxT 1000

// uncomment for debugging output DO NOT FLY ENABLED
#define debug 1

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
double kp3 = 0;
double ki3 = 0;
double kd3 = 0;

// acceleration controller
double kp2 = 0;
double ki2 = 0;
double kd2 = 0;

// angle contorller
double kp1 = 3;
double ki1 = 0.2;
double kd1 = 0.4;

// rate gyro controller
double kp = 1.1;
double ki = 0.012;
double kd = 0;

// false satuation cap to reduce windup
int satEff = 1000;

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
volatile double txErrT, tyErrT, tzErrT;
volatile double txErrL, tyErrL, tzErrL;

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

  Serial.begin(115200);
  

  // Attach each motor
  motorA.attach(6);
  motorB.attach(5);
  motorC.attach(4);
  motorD.attach(3);

  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);
  
  //initilise imu
  imu.Initialize();

  imu.RegisterWrite(MPU6050_CONFIG, 0x05); // configure low pass filter

  #ifdef debug
    //Serial.print("calibrating...");
  #endif

  // calibrate each time or get from calibrate script
  //imu.Calibrate();
  imu.SetGyroOffsets(-201.753204, 139.824401, 67.86100);

  #ifdef debug
    //Serial.println("done");
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
      //Serial.print(" t_c:");
      //Serial.print(tCycle, 8);
    #endif

    // Throttle logic

    // time based throttle logic 
    
//    if(t < 1000) {
//      armed = false;
//    } else if(t < 10000) {
//      armed = true;
//      if(throttle < base) {
//        throttle++;
//      }
//    } else {
//      if(throttle > 0) {
//        throttle--;
//      } else {
//        armed = false;
//      }
//    }
    

    // analog input based logic
    armed = true;
    throttle = map(analogRead(A0), 0, 1023, 0, base);

    #ifdef debug
      throttle = 600;
    #endif

    // Anti throttle spike logic
    if(abs(throttle - throttleLast) > 40){
      killed = true;
//      Serial.println("Throttle Spike: ");
//      Serial.print("Last: ");
//      Serial.println(throttleLast);
//      Serial.print("Current: ");
//      Serial.println(throttle);
    }
    throttleLast = throttle;

    #ifdef debug
//      Serial.print(", throttle = ");
//      Serial.print(throttle);
//      Serial.print("/2000");
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
    vy += (ay * tCycle);
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

    // intergrate error ( only in the air )
    if(throttle > 200) {
      txErrT = sat(txErrT + (txErr * tCycle), -satEff, satEff);
      tyErrT = sat(tyErrT + (tyErr * tCycle), -satEff, satEff);
      tzErrT = sat(tzErrT + (tzErr * tCycle), -satEff, satEff);
    }

    // calc ref value for rate controller
    gxRef = (txErr * kp1) + (txErrT * ki1) + ((txErr - txErrL) * kd1);
    gyRef = (tyErr * kp1) + (txErrT * ki1) + ((tyErr - tyErrL) * kd1);
    gzRef = (tzErr * kp1) + (txErrT * ki1) + ((tzErr - tzErrL) * kd1);

    //store last error used for derivitive
    txErrL = txErr;
    tyErrL = tyErr;
    tzErrL = tzErr;

    // RATE CONTROLLER
    // compute error with angle fed ref
    gxErr = gxRef - gx;
    gyErr = gyRef - gy;
    gzErr = gzRef - gz;

    // intergrate error ( only in the air )
    if(throttle > 200) {
      gxErrT = sat(gxErrT + (gxErr * tCycle), -satEff, satEff);
      gyErrT = sat(gyErrT + (gyErr * tCycle), -satEff, satEff);
      gzErrT = sat(gzErrT + (gzErr * tCycle), -satEff, satEff);
    }
    
    // calculate final effort
    effX = (gxErr * kp) + (gxErrT * ki);
    effY = (gyErr * kp) + (gyErrT * ki);
    effZ = (gzErr * kp) + (gzErrT * ki);

    #ifdef debug
      Serial.print(vx, 4);
      Serial.print(",");
      Serial.print(vy, 4);
      Serial.print(",");
      Serial.print(ax, 4);
      Serial.print(",");
      Serial.print(ay, 4);
      Serial.print(",");
      Serial.print(tx, 4);
      Serial.print(",");
      Serial.print(ty, 4);
      Serial.print(",");
      Serial.print(gx, 4);
      Serial.print(",");
      Serial.print(gy, 4);
    #endif

    // kill on over 15 degree roll. (safety)
    if(tx > 15 || tx < -15 || ty > 15 || ty < -15){
      killed = true;
      //Serial.println("Over 15 degree roll detected");
    }

    // KILL YAW
    //effZ = 0;

    #ifdef debug
      killed = false;
    #endif

    // apply to motors
    if(armed && !killed) {
      effA = sat(throttle - effX - effY + effZ, 48, maxT);
      effB = sat(throttle + effX - effY - effZ, 48, maxT);
      effC = sat(throttle - effX + effY - effZ, 48, maxT);
      effD = sat(throttle + effX + effY + effZ, 48, maxT);
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
      Serial.print(",");
      Serial.print(effA);
      Serial.print(",");
      Serial.print(effB);
      Serial.print(",");
      Serial.print(effC);
      Serial.print(",");
      Serial.print(effD);
    #endif
    
    #ifdef debug
      Serial.println("");
    #endif

    // bit bang the motor commands
    DShot::callSendData(); 

    delay(1);
}
