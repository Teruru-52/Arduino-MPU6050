#include "MPU6050.h"
#include <math.h>
#include <Arduino.h>

MPU6050::MPU6050() {

}

void MPU6050::I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer) {

  Wire.beginTransmission(driver_Addr);
  Wire.write(start_Addr);
  Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(driver_Addr, number_Bytes, true);

  //! Put read results in the Rx buffer
  while (Wire.available()) {
    read_Buffer[i++] = Wire.read();
  }
}

void MPU6050::I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer) {

  Wire.beginTransmission(driver_Addr);
  Wire.write(start_Addr);
  Wire.write(*write_Buffer);
  Wire.endTransmission();
}

int MPU6050::Init(void) {
  unsigned char tempdata[1];
  unsigned char regdata;

  Wire.begin();

  /*I2C_Read_NBytes(ADDRESS, WHO_AM_I, 1, tempdata);
    Serial.print("%02X\r\n",tempdata[0]);
    if(tempdata[0] != 0x19)
    return -1;
    delay(1);*/

  regdata = 0x00; //internal clock 8MHz
  I2C_Write_NBytes(ADDRESS, PWR_MGMT_1, 1, &regdata);
  delay(10);

  regdata = 0x10; //AFS_SEL = 2, Full Scale Range = ±8G, LSB Sensitibity（スケールファクター）4096LSB/G
  I2C_Write_NBytes(ADDRESS, ACCEL_CONFIG, 1, &regdata);
  delay(1);

  regdata = 0x08; //FS_SEL = 1, Full Scale Range = ±500°/s,　LSB　Sensitibity（スケールファクター）65.5LSB/(°/s)
  I2C_Write_NBytes(ADDRESS, GYRO_CONFIG, 1, &regdata);
  delay(1);

  regdata = 0x03; //ジャイロセンサ LPF 43Hz
  I2C_Write_NBytes(ADDRESS, CONFIG, 1, &regdata);

  delay(100);
  getGres();
  getAres();
  return 0;
}

void MPU6050::getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az) {

  uint8_t buf[6];
  I2C_Read_NBytes(ADDRESS, ACCEL_XOUT_H, 6, buf);

  *ax = ((int16_t)buf[0] << 8) | buf[1];
  *ay = ((int16_t)buf[2] << 8) | buf[3];
  *az = ((int16_t)buf[4] << 8) | buf[5];

}
void MPU6050::getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz) {

  uint8_t buf[6];
  I2C_Read_NBytes(ADDRESS, GYRO_XOUT_H, 6, buf);

  *gx = ((uint16_t)buf[0] << 8) | buf[1];
  *gy = ((uint16_t)buf[2] << 8) | buf[3];
  *gz = ((uint16_t)buf[4] << 8) | buf[5];

}

void MPU6050::getTempAdc(int16_t *t) {

  uint8_t buf[2];
  I2C_Read_NBytes(ADDRESS, TEMP_OUT_H, 2, buf);

  *t = ((uint16_t)buf[0] << 8) | buf[1];
}

void MPU6050::getGres() {

  switch (Gyscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    case FS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case FS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case FS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case FS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }

}

void MPU6050::getAres() {
  switch (Acscale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }

}

void MPU6050::SetGyroFsr(Gscale scale)
{
  //return IIC_Write_Byte(MPU_GYRO_CFG_REG,scale<<3);
  unsigned char regdata;
  regdata = (scale << 3);
  I2C_Write_NBytes(ADDRESS, GYRO_CONFIG, 1, &regdata);
  delay(10);

  Gyscale = scale;
  getGres();
}

void MPU6050::SetAccelFsr(Ascale scale)
{
  unsigned char regdata;
  regdata = (scale << 3);
  I2C_Write_NBytes(ADDRESS, ACCEL_CONFIG, 1, &regdata);
  delay(10);

  Acscale = scale;
  getAres();
}




void MPU6050::getAccelData(float* ax, float* ay, float* az) {


  int16_t accX = 0;
  int16_t accY = 0;
  int16_t accZ = 0;
  getAccelAdc(&accX, &accY, &accZ);


  *ax = (float)accX * aRes;
  *ay = (float)accY * aRes;
  *az = (float)accZ * aRes;

}

void MPU6050::getGyroData(float* gx, float* gy, float* gz) {
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  getGyroAdc(&gyroX, &gyroY, &gyroZ);

  *gx = (float)gyroX * gRes;
  *gy = (float)gyroY * gRes;
  *gz = (float)gyroZ * gRes;
}

void MPU6050::getTempData(float *t) {

  int16_t temp = 0;
  getTempAdc(&temp);

  *t = (float)temp / 326.8 + 25.0;
}
