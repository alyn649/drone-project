#include <Arduino.h>
#include <TinyMPU6050.h>

MPU6050 imu (Wire);

void setup() {

  Serial.begin(115200);
  
  //initilise imu
  imu.Initialize();

  imu.RegisterWrite(MPU6050_CONFIG, 0x03); // configure low pass filter

  imu.Calibrate();

  Serial.print("Gyro X Offset: ");
  Serial.println(imu.GetGyroXOffset(), 6);
  Serial.print("Gyro Y Offset: ");
  Serial.println(imu.GetGyroYOffset(), 6);
  Serial.print("Gyro Z Offset: ");
  Serial.println(imu.GetGyroZOffset(), 6);

}

void loop() {
}
