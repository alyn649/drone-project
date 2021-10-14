#include <MPU9250_WE.h>
#include <Wire.h>
#include "DShot.h"
#include <util/delay.h>

#define EFFORT_SATURATION 1200

MPU9250_WE imu = MPU9250_WE(0x68);

DShot motorA;
DShot motorB;
DShot motorC;
DShot motorD;

int effortA;
int effortB;
int effortC;
int effortD;

int baseThrottle = 0;

uint16_t lastCycleTicks;

// Overload operators for xyzFloat to make control portion of program readable
xyzFloat operator+(xyzFloat a, xyzFloat b) {
  xyzFloat r;
  r.x = a.x + b.x;
  r.y = a.y + b.y;
  r.z = a.z + b.z;
  return r;
}

xyzFloat operator-(xyzFloat a, xyzFloat b) {
  xyzFloat r;
  r.x = a.x - b.x;
  r.y = a.y - b.y;
  r.z = a.z - b.z;
  return r;
}

xyzFloat operator-(xyzFloat a) {
  xyzFloat r;
  r.x = -a.x;
  r.y = -a.y;
  r.z = -a.z;
  return r;
}

xyzFloat operator*(xyzFloat a, float b) {
  xyzFloat r;
  r.x = a.x * b;
  r.y = a.y * b;
  r.z = a.z * b;
  return r;
}

xyzFloat operator*(xyzFloat a, double b) {
  xyzFloat r;
  r.x = a.x * b;
  r.y = a.y * b;
  r.z = a.z * b;
  return r;
}

xyzFloat operator/(xyzFloat a, double b) {
  xyzFloat r;
  r.x = a.x / b;
  r.y = a.y / b;
  r.z = a.z / b;
  return r;
}

xyzFloat operator*(xyzFloat a, xyzFloat b) {
  xyzFloat r;
  r.x = a.x * b.x;
  r.y = a.y * b.y;
  r.z = a.z * b.z;
  return r;
}

xyzFloat operator<(xyzFloat a, xyzFloat b) {
  xyzFloat r;
  r.x = a.x < b.x;
  r.y = a.y < b.y;
  r.z = a.z < b.z;
  return r;
}

xyzFloat operator>(xyzFloat a, xyzFloat b) {
  xyzFloat r;
  r.x = a.x > b.x;
  r.y = a.y > b.y;
  r.z = a.z > b.z;
  return r;
}

xyzFloat operator<(xyzFloat a, float b) {
  xyzFloat r;
  r.x = a.x < b;
  r.y = a.y < b;
  r.z = a.z < b;
  return r;
}

xyzFloat operator>(xyzFloat a, float b) {
  xyzFloat r;
  r.x = a.x > b;
  r.y = a.y > b;
  r.z = a.z > b;
  return r;
}

// Use PI control for ang
xyzFloat angRef;
xyzFloat angErr;
xyzFloat angErrTot;
xyzFloat angErrPre;
xyzFloat angErrDer;

xyzFloat angKp;
xyzFloat angKi;
xyzFloat angKd;

xyzFloat angEff;

xyzFloat ang;
xyzFloat acc;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);

  motorA.attach(2);
  motorB.attach(3);
  motorC.attach(4);
  motorD.attach(5);

  motorA.setThrottle(0);
  motorB.setThrottle(0);
  motorC.setThrottle(0);
  motorD.setThrottle(0);

  _delay_ms(1000);
  imu.autoOffsets();

  // ACC CONFIG
  imu.setAccRange(MPU9250_ACC_RANGE_2G); // Max range +- 2g
  imu.enableAccDLPF(true); // Disable low pass filter, allowing smallest delay possible
  imu.setAccDLPF(MPU9250_DLPF_6);

  // Control systems intial values

  // ang refernce
  angRef.x = 0;
  angRef.y = 0;
  angRef.z = 0;

  // ang intergral term
  angErrTot.x = 0;
  angErrTot.y = 0;
  angErrTot.z = 0;
  
  // ang last 
  angErrPre.x = 0;
  angErrPre.y = 0;
  angErrPre.z = 0;

  // ang kp
  angKp.x = 5;
  angKp.y = 5;
  angKp.z = 0;

  // ang ki
  angKi.x = 0.0;
  angKi.y = 0.0;
  angKi.z = 0;

  // ang kd
  angKd.x = 0.6;
  angKd.y = 0.6;
  angKd.z = 0;

  // Setup timer for calculating control frequnecy
  TCCR1A = 0;
  TCCR1B = 0;
  
  TCCR1B |= (0<<CS12) | (1<<CS10);
  TCNT1 = 0;

  digitalWrite(13, LOW);
  _delay_ms(500);

}

void loop() {
    lastCycleTicks = TCNT1;
    TCNT1 = 0;
    acc = imu.getGValues();
    ang = imu.getAngles();
    
    baseThrottle = map(analogRead(A0), 0, 1023, 0, 1000);

    // ANGLE CONTROL SYSTEM
    angErr = angRef - ang; // Compute the error of the ang

    angErr.x = angErr.x * (abs(angErr.x) > 0.1);
    angErr.y = angErr.y * (abs(angErr.y) > 0.1);
    angErr.z = angErr.z * (abs(angErr.z) > 0.1);
      
    angErrTot = angErrTot + (angErr * (lastCycleTicks * (double)0.0625 / 1000000)); // Intergrate error
    
    angErrDer = (angErr - angErrPre) / (lastCycleTicks * ((double)0.0625 / 1000000)); // Find slope of error

    angErrPre = angErr; // Set previous to current
    
    angEff = (angErr * angKp) + (angErrTot * angKi) + (angErrDer * angKd); // Compute control efforts for each axis
  
    // Apply 6 axis efforts to 4 motors angordingly
  
    effortA = baseThrottle - angEff.z - angEff.x + angEff.y;
    effortB = baseThrottle - angEff.z + angEff.x + angEff.y;
    effortC = baseThrottle - angEff.z - angEff.x - angEff.y;
    effortD = baseThrottle - angEff.z + angEff.x - angEff.y;
  
    // Saturate and step into correct range for dshot
    effortA = (sat(effortA, 0, EFFORT_SATURATION))*(baseThrottle > 5) + 48;
    effortB = (sat(effortB, 0, EFFORT_SATURATION))*(baseThrottle > 5) + 48;
    effortC = (sat(effortC, 0, EFFORT_SATURATION))*(baseThrottle > 5) + 48;
    effortD = (sat(effortD, 0, EFFORT_SATURATION))*(baseThrottle > 5) + 48;
    
    // Update the throttle values
    motorA.setThrottle(baseThrottle);
    motorB.setThrottle(baseThrottle);
    motorC.setThrottle(baseThrottle);
    motorD.setThrottle(baseThrottle);


//    Serial.print(" errx:");
//    Serial.print(angErr.x);
    Serial.print(" erry:");
    Serial.print(angErr.y);
    
//    Serial.print(" px:");
//    Serial.print(angErr.x * angKp.x);
    Serial.print(" py:");
    Serial.print(angErr.y * angKp.y);
//    Serial.print(" ix:");
//    Serial.print(angErrTot.x * angKi.x);
    Serial.print(" iy:");
    Serial.print(angErrTot.y * angKi.y);
//    Serial.print(" dx:");
//    Serial.print(angErrDer.x * angKd.x);
    Serial.print(" dy:");
    Serial.print(angErrDer.y * angKd.y);

    Serial.print(" effy:");
    Serial.println((int)angEff.y);
    
//    Serial.print(" effa:");
//    Serial.print(effortA);
//    Serial.print(" effb:");
//    Serial.print(effortB);
//    Serial.print(" effc:");
//    Serial.print(effortC);
//    Serial.print(" effd:");
//    Serial.println(effortD);  

    DShot::callSendData();
}

int sat(int val, int low, int up) {
  if(val < low) {
    val = low;
  }
  if(val > up) {
    val = up;
  }
  return val;
}
