/*
 * MPU6050_Simple.h
 * 
 * MPU9250/MPU6500 IMU 센서를 위한 간단한 라이브러리
 * Roll, Pitch, Yaw 각도를 계산하는 클래스
 * 
 * 작성자: Arduino User
 * 버전: 1.0
 */

#ifndef MPU6050_SIMPLE_H
#define MPU6050_SIMPLE_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// MPU9250 I2C 주소 및 레지스터 정의
#define MPU9250_ADDRESS     0x68
#define PWR_MGMT_1          0x6B
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_XOUT_H        0x3B
#define GYRO_XOUT_H         0x43
#define WHO_AM_I_MPU9250    0x75

// 센서 스케일 팩터
#define ACCEL_SCALE_FACTOR  16384.0  // ±2g 범위
#define GYRO_SCALE_FACTOR   131.0    // ±250°/s 범위

// 필터 상수
#define FILTER_ALPHA        0.98     // 상보 필터 계수

class MPU6050_Simple {
private:
    // 센서 원시 데이터
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    
    // 물리값 데이터
    float ax, ay, az;
    float gx, gy, gz;
    
    // 자세 각도
    float roll, pitch, yaw;
    
    // 타이밍
    unsigned long previousTime;
    
    // 오프셋 값 (캘리브레이션용)
    int16_t accelOffsetX, accelOffsetY, accelOffsetZ;
    int16_t gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    
    // 내부 함수들
    void writeByte(uint8_t regAddr, uint8_t data);
    uint8_t readByte(uint8_t regAddr);
    void convertToPhysicalValues();
    void calculateOrientation();

public:
    // 생성자
    MPU6050_Simple();
    
    // 초기화 및 설정
    bool begin();
    bool testConnection();
    void setFilterAlpha(float alpha);
    
    // 데이터 읽기
    void update();
    void readSensorData();
    
    // 자세 각도 getter
    float getRoll();
    float getPitch();
    float getYaw();
    
    // 원시 센서 데이터 getter
    void getAcceleration(float* x, float* y, float* z);
    void getRotation(float* x, float* y, float* z);
    void getAccelerationRaw(int16_t* x, int16_t* y, int16_t* z);
    void getRotationRaw(int16_t* x, int16_t* y, int16_t* z);
    
    // 캘리브레이션
    void calibrate(int samples = 500);
    void setAccelOffset(int16_t x, int16_t y, int16_t z);
    void setGyroOffset(int16_t x, int16_t y, int16_t z);
    
    // 유틸리티
    void resetOrientation();
    void printData();
    void printFormattedData();
    
    // 초기 상태 확인
    bool isLevelSurface(float tolerance = 5.0);
    bool checkInitialOrientation(float tolerance = 10.0);
    void waitForLevelSurface(float tolerance = 5.0);
    float getTotalAcceleration();
    
    // 센서 설정
    void setSampleRate(uint8_t rate);
    void setAccelRange(uint8_t range);
    void setGyroRange(uint8_t range);
    void setLowPassFilter(uint8_t bandwidth);
};

#endif // MPU6050_SIMPLE_H
