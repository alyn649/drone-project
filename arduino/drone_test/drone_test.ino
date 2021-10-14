#include <MPU9250_WE.h>
#include <Wire.h>
#include <util/delay.h>

MPU9250_WE imu = MPU9250_WE(0x68);

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(13, OUTPUT);

  digitalWrite(13, HIGH);

  Serial.println("on");
  _delay_ms(1000);
  Serial.println("delay");
  imu.autoOffsets();
  Serial.println("cal");

  // GYRO CONFIG
  imu.enableGyrDLPF(); // Enable low pass filer
  imu.setGyrDLPF(MPU9250_DLPF_7); // 
  imu.setSampleRateDivider(0); // Keep sample rate same as internal sample rate
  imu.setGyrRange(MPU9250_GYRO_RANGE_250); // Max range +- 250 d/s

  // ACC CONFIG
  imu.setAccRange(MPU9250_ACC_RANGE_2G); // Max range +- 2g
  imu.enableAccDLPF(false); // Disable low pass filter, allowing smallest delay possible

  digitalWrite(13, LOW);
}

void loop() {
  
}
