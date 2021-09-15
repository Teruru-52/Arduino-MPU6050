#include "MPU6050.h"

MPU6050 IMU;
float axRaw = 0, ayRaw = 0, azRaw = 0, gxRaw = 0, gyRaw = 0, gzRaw = 0;
float Add_axRaw = 0, Add_ayRaw = 0, Add_azRaw = 0, Add_gxRaw = 0, Add_gyRaw = 0, Add_gzRaw = 0;
float Offset_axRaw, Offset_ayRaw, Offset_azRaw, Offset_gxRaw, Offset_gyRaw, Offset_gzRaw;
//float driftgz = 0;

//Calibration
void offset_cal() {
  delay(3000);
  for (int i = 0; i < 1000; i++) {
    IMU.getAccelData(&axRaw, &ayRaw, &azRaw);
    IMU.getGyroData(&gxRaw, &gyRaw, &gzRaw);
    Add_axRaw += axRaw;
    Add_ayRaw += ayRaw;
    Add_azRaw += azRaw;
    Add_gxRaw += gxRaw;
    Add_gyRaw += gyRaw;
    Add_gzRaw += gzRaw;
  }
  Offset_axRaw = Add_axRaw / 1000.0;
  Offset_ayRaw = Add_ayRaw / 1000.0;
  Offset_azRaw = Add_azRaw / 1000.0;

  Offset_gxRaw = Add_gxRaw / 1000.0;
  Offset_gyRaw = Add_gyRaw / 1000.0;
  Offset_gzRaw = Add_gzRaw / 1000.0;
}

void setup() {
  Serial.begin(9600);
  IMU.Init();
  offset_cal();
  IMU.SetAccelFsr(IMU.AFS_2G);
  IMU.SetGyroFsr(IMU.FS_500DPS);
}

void loop() {
  IMU.getAccelData(&axRaw, &ayRaw, &azRaw);
  IMU.getGyroData(&gxRaw, &gyRaw, &gzRaw);

  float ax = axRaw - Offset_axRaw;
  float ay = ayRaw - Offset_ayRaw;
  float az = azRaw - Offset_azRaw + 1.0;

  float gx = gxRaw - Offset_gxRaw;
  float gy = gyRaw - Offset_gyRaw;
  float gz = gzRaw - Offset_gzRaw;

  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.print(gx, 4); Serial.print(",");
  Serial.print(gy, 4); Serial.print(",");
  Serial.println(gz, 4);
  //Serial.print(azRaw, 4); Serial.print(",");
  //Serial.println(Offset_azRaw, 4);
  //Serial.println(driftgz,4);
  //driftgz += gz;
  delay(1000);

}
#Arduino-MPU6050
