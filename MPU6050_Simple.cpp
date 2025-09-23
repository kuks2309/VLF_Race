/*
 * MPU6050_Simple.cpp
 * 
 * MPU9250/MPU6500 IMU 센서를 위한 간단한 라이브러리 구현
 * 
 * 작성자: Arduino User
 * 버전: 1.0
 */

#include "MPU6050_Simple.h"

// 생성자
MPU6050_Simple::MPU6050_Simple() {
    // 변수 초기화
    accelX = accelY = accelZ = 0;
    gyroX = gyroY = gyroZ = 0;
    ax = ay = az = 0.0;
    gx = gy = gz = 0.0;
    roll = pitch = yaw = 0.0;
    previousTime = 0;
    
    // 오프셋 초기화
    accelOffsetX = accelOffsetY = accelOffsetZ = 0;
    gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0;
}

// 센서 초기화
bool MPU6050_Simple::begin() {
    // I2C 초기화는 외부에서 수행되었다고 가정
    
    // 연결 테스트
    if (!testConnection()) {
        return false;
    }
    
    // 절전 모드 해제
    writeByte(PWR_MGMT_1, 0x00);
    delay(100);
    
    // 기본 설정
    setSampleRate(9);           // 100Hz (1kHz / (1 + 9))
    setLowPassFilter(3);        // 44Hz 저역통과 필터
    setGyroRange(0);            // ±250°/s
    setAccelRange(0);           // ±2g
    
    // 시간 초기화
    previousTime = millis();
    
    delay(100);
    return true;
}

// 연결 테스트
bool MPU6050_Simple::testConnection() {
    Wire.beginTransmission(MPU9250_ADDRESS);
    return (Wire.endTransmission() == 0);
}

// 필터 알파값 설정
void MPU6050_Simple::setFilterAlpha(float alpha) {
    // 헤더에서 FILTER_ALPHA 상수를 사용하므로 런타임 변경은 제한적
    // 필요시 멤버 변수로 변경 가능
}

// 센서 데이터 업데이트 (메인 함수)
void MPU6050_Simple::update() {
    readSensorData();
    convertToPhysicalValues();
    calculateOrientation();
}

// 센서 데이터 읽기
void MPU6050_Simple::readSensorData() {
    // 가속도 데이터 읽기 (6바이트)
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU9250_ADDRESS, 6);
    
    accelX = (Wire.read() << 8) | Wire.read();
    accelY = (Wire.read() << 8) | Wire.read();
    accelZ = (Wire.read() << 8) | Wire.read();
    
    // 자이로스코프 데이터 읽기 (6바이트)
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU9250_ADDRESS, 6);
    
    gyroX = (Wire.read() << 8) | Wire.read();
    gyroY = (Wire.read() << 8) | Wire.read();
    gyroZ = (Wire.read() << 8) | Wire.read();
    
    // 오프셋 적용
    accelX -= accelOffsetX;
    accelY -= accelOffsetY;
    accelZ -= accelOffsetZ;
    gyroX -= gyroOffsetX;
    gyroY -= gyroOffsetY;
    gyroZ -= gyroOffsetZ;
}

// 물리값으로 변환
void MPU6050_Simple::convertToPhysicalValues() {
    // 가속도를 g 단위로 변환
    ax = accelX / ACCEL_SCALE_FACTOR;
    ay = accelY / ACCEL_SCALE_FACTOR;
    az = accelZ / ACCEL_SCALE_FACTOR;
    
    // 자이로스코프를 °/s 단위로 변환
    gx = gyroX / GYRO_SCALE_FACTOR;
    gy = gyroY / GYRO_SCALE_FACTOR;
    gz = gyroZ / GYRO_SCALE_FACTOR;
}

// 자세 각도 계산
void MPU6050_Simple::calculateOrientation() {
    // 현재 시간 계산
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;
    
    // deltaTime이 비정상적으로 클 경우 제한
    if (deltaTime > 0.1) deltaTime = 0.01;
    
    // 가속도계로부터 Roll, Pitch 계산
    float accelRoll = atan2(ay, az) * 180.0 / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    
    // 자이로스코프 적분으로 각도 변화량 계산
    float gyroRoll = roll + gx * deltaTime;
    float gyroPitch = pitch + gy * deltaTime;
    float gyroYaw = yaw + gz * deltaTime;
    
    // 상보 필터 적용 (Roll, Pitch)
    roll = FILTER_ALPHA * gyroRoll + (1.0 - FILTER_ALPHA) * accelRoll;
    pitch = FILTER_ALPHA * gyroPitch + (1.0 - FILTER_ALPHA) * accelPitch;
    
    // Yaw는 자이로스코프만 사용 (자력계 없음)
    yaw = gyroYaw;
    
    // Yaw 각도를 -180~180 범위로 제한
    if (yaw > 180.0) yaw -= 360.0;
    if (yaw < -180.0) yaw += 360.0;
}

// Roll 각도 반환
float MPU6050_Simple::getRoll() {
    return roll;
}

// Pitch 각도 반환
float MPU6050_Simple::getPitch() {
    return pitch;
}

// Yaw 각도 반환
float MPU6050_Simple::getYaw() {
    return yaw;
}

// 가속도 값 반환 (g 단위)
void MPU6050_Simple::getAcceleration(float* x, float* y, float* z) {
    *x = ax;
    *y = ay;
    *z = az;
}

// 회전 속도 반환 (°/s 단위)
void MPU6050_Simple::getRotation(float* x, float* y, float* z) {
    *x = gx;
    *y = gy;
    *z = gz;
}

// 원시 가속도 값 반환
void MPU6050_Simple::getAccelerationRaw(int16_t* x, int16_t* y, int16_t* z) {
    *x = accelX;
    *y = accelY;
    *z = accelZ;
}

// 원시 자이로 값 반환
void MPU6050_Simple::getRotationRaw(int16_t* x, int16_t* y, int16_t* z) {
    *x = gyroX;
    *y = gyroY;
    *z = gyroZ;
}

// 센서 캘리브레이션
void MPU6050_Simple::calibrate(int samples) {
    Serial.println("센서 캘리브레이션 시작...");
    Serial.println("센서를 평평한 곳에 놓고 움직이지 마세요.");
    Serial.print(samples);
    Serial.println("개 샘플을 수집합니다...");
    
    long sumAx = 0, sumAy = 0, sumAz = 0;
    long sumGx = 0, sumGy = 0, sumGz = 0;
    
    delay(2000); // 2초 대기
    
    for (int i = 0; i < samples; i++) {
        readSensorData();
        
        // 오프셋을 적용하지 않은 원시 데이터 사용
        sumAx += (accelX + accelOffsetX);
        sumAy += (accelY + accelOffsetY);
        sumAz += (accelZ + accelOffsetZ);
        sumGx += (gyroX + gyroOffsetX);
        sumGy += (gyroY + gyroOffsetY);
        sumGz += (gyroZ + gyroOffsetZ);
        
        delay(10);
        
        if (i % 100 == 0) {
            Serial.print(".");
        }
    }
    Serial.println();
    
    // 평균값 계산 (오프셋)
    accelOffsetX = sumAx / samples;
    accelOffsetY = sumAy / samples;
    accelOffsetZ = (sumAz / samples) - 16384; // Z축은 중력 보정
    gyroOffsetX = sumGx / samples;
    gyroOffsetY = sumGy / samples;
    gyroOffsetZ = sumGz / samples;
    
    Serial.println("캘리브레이션 완료!");
    Serial.println("계산된 오프셋 값:");
    Serial.print("Accel - X: "); Serial.print(accelOffsetX);
    Serial.print(", Y: "); Serial.print(accelOffsetY);
    Serial.print(", Z: "); Serial.println(accelOffsetZ);
    Serial.print("Gyro - X: "); Serial.print(gyroOffsetX);
    Serial.print(", Y: "); Serial.print(gyroOffsetY);
    Serial.print(", Z: "); Serial.println(gyroOffsetZ);
}

// 가속도 오프셋 설정
void MPU6050_Simple::setAccelOffset(int16_t x, int16_t y, int16_t z) {
    accelOffsetX = x;
    accelOffsetY = y;
    accelOffsetZ = z;
}

// 자이로 오프셋 설정
void MPU6050_Simple::setGyroOffset(int16_t x, int16_t y, int16_t z) {
    gyroOffsetX = x;
    gyroOffsetY = y;
    gyroOffsetZ = z;
}

// 자세 각도 리셋
void MPU6050_Simple::resetOrientation() {
    roll = 0;
    pitch = 0;
    yaw = 0;
    previousTime = millis();
}

// 데이터 출력 (원본 형식)
void MPU6050_Simple::printData() {
    Serial.print("# G ");
    Serial.print(roll, 2);
    Serial.print(" ");
    Serial.print(pitch, 2);
    Serial.print(" ");
    Serial.print(yaw, 2);
    Serial.println(" *");
}

// 포맷된 데이터 출력
void MPU6050_Simple::printFormattedData() {
    Serial.print("Roll: ");
    Serial.print(roll, 1);
    Serial.print("°\t");
    Serial.print("Pitch: ");
    Serial.print(pitch, 1);
    Serial.print("°\t");
    Serial.print("Yaw: ");
    Serial.print(yaw, 1);
    Serial.println("°");
    Serial.println("--------------------");
}

// 샘플 레이트 설정
void MPU6050_Simple::setSampleRate(uint8_t rate) {
    writeByte(SMPLRT_DIV, rate);
}

// 가속도 범위 설정 (0=±2g, 1=±4g, 2=±8g, 3=±16g)
void MPU6050_Simple::setAccelRange(uint8_t range) {
    writeByte(ACCEL_CONFIG, range << 3);
}

// 자이로 범위 설정 (0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s)
void MPU6050_Simple::setGyroRange(uint8_t range) {
    writeByte(GYRO_CONFIG, range << 3);
}

// 저역통과 필터 설정
void MPU6050_Simple::setLowPassFilter(uint8_t bandwidth) {
    writeByte(CONFIG, bandwidth);
}

// I2C 바이트 쓰기
void MPU6050_Simple::writeByte(uint8_t regAddr, uint8_t data) {
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(regAddr);
    Wire.write(data);
    Wire.endTransmission();
}

// I2C 바이트 읽기
uint8_t MPU6050_Simple::readByte(uint8_t regAddr) {
    Wire.beginTransmission(MPU9250_ADDRESS);
    Wire.write(regAddr);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU9250_ADDRESS, 1);
    return Wire.read();
}

// 평평한 면에 놓였는지 확인
bool MPU6050_Simple::isLevelSurface(float tolerance) {
    // 몇 개의 샘플을 읽어서 평균 계산
    float sumAx = 0, sumAy = 0, sumAz = 0;
    int samples = 50;
    
    for (int i = 0; i < samples; i++) {
        readSensorData();
        convertToPhysicalValues();
        
        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        
        delay(10);
    }
    
    // 평균값 계산
    float avgAx = sumAx / samples;
    float avgAy = sumAy / samples;
    float avgAz = sumAz / samples;
    
    // 중력 벡터의 크기 (약 1g여야 함)
    float totalAccel = sqrt(avgAx * avgAx + avgAy * avgAy + avgAz * avgAz);
    
    // Z축이 중력 방향과 얼마나 일치하는지 확인
    float zAngleFromVertical = acos(abs(avgAz) / totalAccel) * 180.0 / PI;
    
    Serial.print("중력 벡터 크기: "); Serial.print(totalAccel, 3); Serial.println("g");
    Serial.print("수직으로부터 기울어진 각도: "); Serial.print(zAngleFromVertical, 1); Serial.println("°");
    
    // 허용 오차 내에 있는지 확인
    return (zAngleFromVertical <= tolerance);
}

// 초기 자세 확인
bool MPU6050_Simple::checkInitialOrientation(float tolerance) {
    Serial.println("초기 센서 자세 확인 중...");
    
    // 센서 데이터를 몇 번 읽어서 안정화
    for (int i = 0; i < 10; i++) {
        readSensorData();
        delay(50);
    }
    
    // 평평한 면 확인
    bool isLevel = isLevelSurface(tolerance);
    
    if (isLevel) {
        Serial.println("✓ 센서가 평평한 면에 올바르게 놓여있습니다.");
        return true;
    } else {
        Serial.println("✗ 경고: 센서가 기울어져 있습니다!");
        Serial.print("허용 기울기: ±"); Serial.print(tolerance, 1); Serial.println("°");
        Serial.println("정확한 측정을 위해 센서를 평평한 면에 놓으세요.");
        return false;
    }
}

// 평평한 면에 놓일 때까지 대기
void MPU6050_Simple::waitForLevelSurface(float tolerance) {
    Serial.println("센서를 평평한 면에 놓고 안정화될 때까지 대기 중...");
    
    int attemptCount = 0;
    while (!isLevelSurface(tolerance)) {
        attemptCount++;
        Serial.print("시도 "); Serial.print(attemptCount); 
        Serial.println(": 센서를 평평하게 놓아주세요...");
        
        // 5초 대기 후 재확인
        for (int i = 5; i > 0; i--) {
            Serial.print(i); Serial.print("...");
            delay(1000);
        }
        Serial.println();
        
        // 10번 시도 후에도 안되면 경고하고 진행
        if (attemptCount >= 10) {
            Serial.println("⚠ 경고: 센서가 여전히 기울어져 있지만 계속 진행합니다.");
            Serial.println("측정 정확도가 떨어질 수 있습니다.");
            break;
        }
    }
    
    if (attemptCount < 10) {
        Serial.println("✓ 센서가 올바른 자세로 설정되었습니다!");
    }
    
    // 안정화를 위한 추가 대기
    Serial.println("센서 안정화 중...");
    delay(2000);
}

// 총 가속도 크기 반환
float MPU6050_Simple::getTotalAcceleration() {
    return sqrt(ax * ax + ay * ay + az * az);
}
