#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <Wire.h>
#include <Arduino.h>

#define ADDRESS           0x68
#define WHO_AM_I          0x75
#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40

#define TEMP_OUT_H        0x41
#define TEMP_OUT_L        0x42

#define GYRO_XOUT_H       0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

#define PWR_MGMT_1        0x6B
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C

class MPU6050 {
  public:
    enum Ascale {
      AFS_2G = 0,
      AFS_4G,
      AFS_8G,
      AFS_16G
    };

    enum Gscale {
      FS_250DPS = 0,
      FS_500DPS,
      FS_1000DPS,
      FS_2000DPS
    };

    Gscale Gyscale = FS_500DPS;
    Ascale Acscale = AFS_8G;

  public:
    MPU6050();
    int Init(void);
    void getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az);
    void getGyroAdc(int16_t* gx, int16_t* gy, int16_t* gz);
    void getTempAdc(int16_t *t);

    void getAccelData(float* ax, float* ay, float* az);
    void getGyroData(float* gx, float* gy, float* gz);
    void getTempData(float *t);

    void SetGyroFsr(Gscale scale);
    void SetAccelFsr(Ascale scale);
  public:
    float aRes, gRes;

  private:
    float _last_theta = 0;
    float _last_phi = 0;
    float _alpha = 0.5;
  private:
    void I2C_Read_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *read_Buffer);
    void I2C_Write_NBytes(uint8_t driver_Addr, uint8_t start_Addr, uint8_t number_Bytes, uint8_t *write_Buffer);
    void getGres();
    void getAres();
};
#endif
