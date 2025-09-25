#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <avr/io.h>  // AVR register definitions
#include <avr/wdt.h> // Watchdog Timer header
#include <semphr.h>
#include <Wire.h>    // I2C communication
#include <VL53L0X.h> // VL53L0X ToF sensor
#include "MPU6050_Simple.h" // MPU6050 IMU sensor
#include "RecursiveMovingAverage.h" // Moving average filter

// Camera Control
#define TSL1401_CLK 12 // TSL1401 clock pin
#define TSL1401_SI 13  // TSL1401 SI pin
#define TSL1401_AO A0  // TSL1401 analog output pin
#define NPIXELS 128    // Total pixel count
#define Camera_calibration 2.1  // Pixel to mm conversion factor
#define H 250.0 // Distance between camera and wheel axis in mm


// Test Pin Setup

#define TSL1401_TEST_PIN 42
#define TOF_TEST_PIN   43
#define CONTROL_TEST_PIN   44
#define IMU_TEST_PIN   45

//////////////////////////// Pulse 관련 ////////////////
#define pulse 232.0
#define pulse_m 1160.0
double  pulse_distance;

// Motor Control
#define MOTOR_DIR_PIN 4
#define MOTOR_PWM_PIN 5
#define encodPinA1   2
#define encodPinB1   3

// Servo control
#define SERVO_PIN 22
#define STRAIGHT_ANGLE 87
#define MAX_LEFT_Steering_Angle   -30
#define MAX_RIGHT_Steering_Angle   30

// VL53L0X ToF Sensor Pins and Addresses
#define XSHUT_PIN_R 11 // Right sensor
#define XSHUT_PIN_C 10 // Center sensor
#define XSHUT_PIN_L 9  // Left sensor

#define SENSOR_R_ADDRESS 0x30
#define SENSOR_C_ADDRESS 0x31
#define SENSOR_L_ADDRESS 0x32

#define FASTADC 1 // Fast ADC setting

// Macro definitions for ADC register manipulation (used in FASTADC)
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

// Semaphore declarations
SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t xDataSemaphore; // Pixel data protection
SemaphoreHandle_t xSteeringSemaphore; // Steering data protection
SemaphoreHandle_t xVL53L0XSemaphore; // VL53L0X sensor data protection
SemaphoreHandle_t xIMUSemaphore; // IMU sensor data protection
SemaphoreHandle_t xI2CSemaphore; // I2C bus protection

// Task handle declarations
TaskHandle_t xCameraTaskHandle = NULL;
TaskHandle_t xSerialTaskHandle = NULL;
TaskHandle_t xControlTaskHandle = NULL;
TaskHandle_t xVL53L0XTaskHandle = NULL;
TaskHandle_t xIMUTaskHandle = NULL;

// Task function declarations
void TaskSerial(void *pvParameters);
void TaskCamera(void *pvParameters);
void TaskControl(void *pvParameters);
void TaskVL53L0X(void *pvParameters);
void TaskIMU(void *pvParameters);

// Servo object
Servo myServo;

// VL53L0X sensor objects
VL53L0X sensorR;
VL53L0X sensorC;
VL53L0X sensorL;

// MPU6050 IMU sensor object
MPU6050_Simple mpu;

// Moving average filter for IMU yaw angle
RecursiveMovingAverage yawFilter;
volatile float imu_yaw_filter_avg = 0.0;  // IMU yaw filter average

// Moving average filter for TOF Sensors
RecursiveMovingAverage tofFilterR;  // Right sensor filter
RecursiveMovingAverage tofFilterC;  // Center sensor filter
RecursiveMovingAverage tofFilterL;  // Left sensor filter

// Filter average values (global)
volatile float tof_filter_avg_r = 0.0;
volatile float tof_filter_avg_c = 0.0;
volatile float tof_filter_avg_l = 0.0;

// H value definition (used in calculate_yaw_angle_y_axis function)


// Task control flags
volatile bool enableCamera = true;      // true: camera running, false: camera stopped
volatile bool cameraTaskRunning = true; // Current camera task status
volatile bool serialTaskRunning = true; // Current serial task status
volatile bool vl53l0xTaskRunning = true; // Current VL53L0X task status

// VL53L0X sensor variables
volatile uint16_t vl53l0x_distance_r = 0;  // Right sensor distance (mm) - filtered
volatile uint16_t vl53l0x_distance_c = 0;  // Center sensor distance (mm) - filtered
volatile uint16_t vl53l0x_distance_l = 0;  // Left sensor distance (mm) - filtered
volatile uint16_t vl53l0x_raw_r = 0;  // Right sensor raw value
volatile uint16_t vl53l0x_raw_c = 0;  // Center sensor raw value
volatile uint16_t vl53l0x_raw_l = 0;  // Left sensor raw value
volatile bool vl53l0x_initialized = false;  // VL53L0X initialization status

// IMU sensor variables
volatile float imu_roll = 0.0;   // Left-right tilt (degrees)
volatile float imu_pitch = 0.0;  // Front-back tilt (degrees)
volatile float imu_yaw = 0.0;    // Left-right rotation (degrees)
volatile float imu_yaw_filtered = 0.0;  // Filtered yaw angle using moving average
volatile bool imu_initialized = false;  // IMU initialization status

// Steering control variables
volatile float steering_angle = 0.0;    // Calculated steering angle
volatile float vision_steer_angle = 0.0; // Vision-based steering angle
volatile float line_center_pos = NPIXELS / 2.0; // Line center position (center of mass)
volatile int mission_flag = 0;          // Mission state flag

// PD control variables for Lane Control
float Kp = 0.42;  // Proportional gain (response to line center error)
float Kd = 1.5;  // Derivative gain (response to error rate of change)
float previous_error = 0.0;  // Store previous error value


// PD control variables for Yaw Control
float Kp_yaw = 0.42;  // Proportional gain (response to line center error)
float Kd_yaw = 1.5;  // Derivative gain (response to error rate of change)
float previous_error_yaw = 0.0;  // Store previous error value

// PD control variables for Wall follwoing Control
float Kp_wall = 0.42;  // Proportional gain (response to line center error)
float Kd_wall = 1.5;  // Derivative gain (response to error rate of change)
float previous_error_wall = 0.0;  // Store previous error value

// Encoder variables
volatile long encoderPos = 0;

// Camera threshold setting
static int cameraThreshold = 120; // Default 128, can be changed as needed


byte Pixel[NPIXELS];          // Original pixel data
byte PixelBuffer[NPIXELS];    // Transmission buffer
byte threshold_data[NPIXELS]; // Threshold processed data

// Yaw Angle
float current_yaw_angle = 0.0;

//====================================================================
// CAMERA CONTROL FUNCTIONS
//====================================================================

void Enable_Camera()
{
    if (xCameraTaskHandle != NULL && !cameraTaskRunning)
    {
        vTaskResume(xCameraTaskHandle);
        cameraTaskRunning = true;

        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial1.println("Camera task enabled");
            xSemaphoreGive(xSerialSemaphore);
        }
    }
}

void Disable_Camera()
{
    if (xCameraTaskHandle != NULL && cameraTaskRunning)
    {
        vTaskSuspend(xCameraTaskHandle);
        cameraTaskRunning = false;

        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial1.println("Camera task disabled");
            xSemaphoreGive(xSerialSemaphore);
        }
    }
}

// Serial task control functions
void Enable_Serial()
{
    if (xSerialTaskHandle != NULL && !serialTaskRunning)
    {
        vTaskResume(xSerialTaskHandle);
        serialTaskRunning = true;

        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial1.println("Serial task enabled");
            xSemaphoreGive(xSerialSemaphore);
        }
    }
}

void Disable_Serial()
{
    if (xSerialTaskHandle != NULL && serialTaskRunning)
    {
        vTaskSuspend(xSerialTaskHandle);
        serialTaskRunning = false;

        // Output final message when serial task is disabled
        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial1.println("Serial task disabled");
            xSemaphoreGive(xSerialSemaphore);
        }
    }
}

// VL53L0X task control functions
void Enable_VL53L0X()
{
    if (xVL53L0XTaskHandle != NULL && !vl53l0xTaskRunning)
    {
        vTaskResume(xVL53L0XTaskHandle);
        vl53l0xTaskRunning = true;

        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial1.println("VL53L0X task enabled");
            xSemaphoreGive(xSerialSemaphore);
        }
    }
}

void Disable_VL53L0X()
{
    if (xVL53L0XTaskHandle != NULL && vl53l0xTaskRunning)
    {
        vTaskSuspend(xVL53L0XTaskHandle);
        vl53l0xTaskRunning = false;

        // Output final message when VL53L0X task is disabled
        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial1.println("VL53L0X task disabled");
            xSemaphoreGive(xSerialSemaphore);
        }
    }
}

// Threshold-based binarization function (1 if value >= threshold, 0 otherwise)
void line_threshold(int threshold)
{
    for (int i = 0; i < NPIXELS; i++)
    {
        threshold_data[i] = (Pixel[i] > threshold) ? 1 : 0;
    }
}

float find_line_center(void)
{
    static float prev_line_center = NPIXELS / 2.0;
    int i;
    long sum = 0;
    long x_sum = 0;

    for (i = 0; i < NPIXELS; i++)
    {
        sum += threshold_data[i];
        x_sum += (threshold_data[i] * i);
    }

    if (sum == 0)
    {
        return prev_line_center;
    }

    float line_center = (float) x_sum / sum;
    prev_line_center = line_center;
    return line_center;
}

// Read camera data (analogRead / 4 → convert to 0~255 range)
void read_TSL1401_camera()
{
    // Start SI pulse
    digitalWrite(TSL1401_SI, HIGH);
    digitalWrite(TSL1401_CLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(TSL1401_CLK, LOW);
    digitalWrite(TSL1401_SI, LOW);
    delayMicroseconds(1);

    // Read pixel values
    for (int i = 0; i < NPIXELS; i++)
    {
        digitalWrite(TSL1401_CLK, HIGH);
        delayMicroseconds(1);
        Pixel[i] = analogRead(TSL1401_AO) / 4; // 0~1023 → 0~255
        digitalWrite(TSL1401_CLK, LOW);
        delayMicroseconds(1);
    }
}

void send_pixel_data()
{
    for (int i = 0; i < NPIXELS; i++)
    {

        Serial.print(Pixel[i]);
        Serial.print(",");
    }
    Serial.println();
}

//====================================================================
// Maze CONTROL FUNCTIONS
//====================================================================

bool detect_maze_entrance(void)
{
    if( (vl53l0x_distance_r <= 250) && (vl53l0x_distance_l <= 250 ) && (vl53l0x_distance_c <= 500))
    {
       return true;
    }
    else
    {
        return false;
    }
}

//====================================================================
// MOTOR CONTROL FUNCTIONS
//====================================================================

// Car Wall Following Control
void car_wall_following(void)
{
    // Calculate error between right and left wall distances
    float error = vl53l0x_distance_r - vl53l0x_distance_l;

    // Calculate derivative term (error rate of change)
    float derivative = error - previous_error_wall;

    // Apply PD control
    float wall_control_steering_angle = (Kp_wall * error) + (Kd_wall * derivative);

    // Update previous error value
    previous_error_wall = error;

    // Apply steering control with calculated angle
    Steering_Control(wall_control_steering_angle);
}

// Car Yaw Control
void car_yaw_control(float target_yaw)
{
    // Calculate error between target and current yaw angle
    float error = target_yaw - imu_yaw_filtered;

    // Calculate derivative term (error rate of change)
    float derivative = error - previous_error_yaw;

    // Apply PD control
    float yaw_control_steering_angle = (Kp_yaw * error) + (Kd_yaw * derivative);

    // Update previous error value
    previous_error_yaw = error;

    // Apply steering control with calculated angle
    Steering_Control(yaw_control_steering_angle);
}

void motor_control(int8_t direction, uint8_t motor_speed)
{
    if (direction == 1)
    {
        digitalWrite(MOTOR_DIR_PIN, HIGH);
    }
    else
    {
        digitalWrite(MOTOR_DIR_PIN, LOW);
    }
    analogWrite(MOTOR_PWM_PIN, motor_speed);
}

// PD control function - calculate steering angle based on line center error
float calculate_pd_steering(float line_center)
{
    // Error calculation: deviation from center (64)
    // Positive error means line is on the right -> steer right
    // Negative error means line is on the left -> steer left
    float error = line_center - (NPIXELS / 2.0);  // 64 pixels is center
    
    // Calculate derivative term (error rate of change)
    float derivative = error - previous_error;
    
    // Calculate PD control output
    float steering_output = (Kp * error) + (Kd * derivative);
    
    // Update previous error value
    previous_error = error;
    
    // Limit steering angle (-30 ~ +30 degrees)
    if (steering_output > 37.0) {
        steering_output = 37.0;
    } else if (steering_output < -37.0) {
        steering_output = -37.0;
    }
    
    return steering_output;
}

//====================================================================
// IMU SENSOR FUNCTIONS
//====================================================================

bool IMU_Sensor_Setup()
{
    // Set I2C communication speed (400kHz)
    Wire.setClock(400000);

    // Initialize MPU sensor
    Serial1.println("Initializing IMU sensor...");
    Serial1.flush();

    if (!mpu.begin()) {
        Serial1.println("IMU sensor initialization failed!");
        Serial1.println("Check IMU connection.");
        Serial1.flush();
        return false;
    }

    Serial1.println("IMU sensor initialized successfully!");
    Serial1.flush();

    // Check initial orientation (within 10 degrees)
    Serial1.println("Checking initial orientation...");
    if (mpu.checkInitialOrientation(10.0)) {
        Serial1.println("IMU orientation is correct!");
    } else {
        Serial1.println("IMU sensor is tilted. Please level the sensor...");
        // Wait until sensor is level within 5 degrees (max 5 seconds)
        unsigned long startTime = millis();
        while (!mpu.checkInitialOrientation(5.0) && (millis() - startTime) < 5000) {
            delay(100);
        }
        if (mpu.checkInitialOrientation(5.0)) {
            Serial1.println("IMU sensor leveled successfully!");
        } else {
            Serial1.println("WARNING: IMU sensor not perfectly level, continuing anyway...");
        }
    }

    // Perform calibration (500 samples)
    Serial1.println("Calibrating IMU sensor...");
    Serial1.println("Keep the sensor still during calibration!");
    mpu.calibrate(500);
    Serial1.println("IMU calibration completed!");
    Serial1.flush();

    // Reset orientation angles
    mpu.resetOrientation();
    Serial1.println("IMU orientation reset completed!");

    // Connection test
    if (mpu.testConnection()) {
        Serial1.println("IMU connection test passed!");
        Serial1.println("------------------------");
        Serial1.flush();
        imu_initialized = true;
        return true;
    } else {
        Serial1.println("IMU connection test failed!");
        Serial1.flush();
        return false;
    }
}

// Read IMU sensor data and update global variables
void read_IMU_data()
{
    if (imu_initialized) {
        // Protect I2C bus access
        if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(5)) == pdTRUE) {
            // Update sensor data
            mpu.update();

            xSemaphoreGive(xI2CSemaphore);

            // Read orientation angles and update global variables
            if (xSemaphoreTake(xIMUSemaphore, pdMS_TO_TICKS(5)) == pdTRUE) {
                //imu_roll = mpu.getRoll();
                //imu_pitch = mpu.getPitch();
                imu_yaw = mpu.getYaw();

                // Apply moving average filter to yaw angle with proper method
                imu_yaw_filter_avg = yawFilter.update(imu_yaw, imu_yaw_filter_avg);
                imu_yaw_filtered = imu_yaw_filter_avg;

                xSemaphoreGive(xIMUSemaphore);
            }
        }
    }
}

//====================================================================
// VL53L0X SENSOR FUNCTIONS
//====================================================================

void VL530L0x_Sensor_Setup()
{
    Wire.begin();

    // Set XSHUT pins as output
    pinMode(XSHUT_PIN_R, OUTPUT);
    pinMode(XSHUT_PIN_C, OUTPUT);
    pinMode(XSHUT_PIN_L, OUTPUT);

    // Disable all sensors
    digitalWrite(XSHUT_PIN_R, LOW);
    digitalWrite(XSHUT_PIN_C, LOW);
    digitalWrite(XSHUT_PIN_L, LOW);
    delay(10);

    // Initialize Right sensor and change address
    digitalWrite(XSHUT_PIN_R, HIGH);
    delay(10);
    sensorR.init();
    sensorR.setAddress(SENSOR_R_ADDRESS);
    Serial1.println("Right sensor initialized");
    Serial1.flush();

    // Initialize Center sensor and change address
    digitalWrite(XSHUT_PIN_C, HIGH);
    delay(10);
    sensorC.init();
    sensorC.setAddress(SENSOR_C_ADDRESS);
    Serial1.println("Center sensor initialized");
    Serial1.flush();

    // Initialize Left sensor and change address
    digitalWrite(XSHUT_PIN_L, HIGH);
    delay(10);
    sensorL.init();
    sensorL.setAddress(SENSOR_L_ADDRESS);
    Serial1.println("Left sensor initialized");
    Serial1.flush();

    // Set measurement timing budget (higher = more accurate but slower)
    sensorR.setMeasurementTimingBudget(20000); // 20ms
    sensorC.setMeasurementTimingBudget(20000); // 20ms
    sensorL.setMeasurementTimingBudget(20000); // 20ms

    // Start continuous reading mode with period (0 = as fast as possible)
    sensorR.startContinuous(0);
    sensorC.startContinuous(0);
    sensorL.startContinuous(0);

    Serial1.println("All VL530L0x sensors ready!");
    Serial1.println("------------------------");
    Serial1.flush();

    vl53l0x_initialized = true;
}


//====================================================================
// ENCODER FUNCTIONS
//====================================================================

void encoder_read()
{
  delayMicroseconds(2);
  if (digitalRead(encodPinB1) == LOW) encoderPos++;
  else                                encoderPos--;
}

void reset_encoder(void)
{
  encoderPos = 0;
}

void encoder_to_distance(void)
{

    pulse_distance = encoderPos / pulse_m;

}

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);        // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);        // quadrature encoder input B
  attachInterrupt(0, encoder_read, FALLING);    // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;         // To prevent Motor Noise
}

float Steering_Angle_Limit(float s_angle, float limit)
{
    if(s_angle <= -limit)
    {
        s_angle = -limit;
    }

    if(s_angle >= limit)
    {
        s_angle = limit;
    }

    return s_angle;
}

void Steering_Control(float s_angle)
{
  if(s_angle <= MAX_LEFT_Steering_Angle) 
  {
    s_angle = MAX_LEFT_Steering_Angle;
  }

  if(s_angle >= MAX_RIGHT_Steering_Angle) 
  {
    s_angle = MAX_RIGHT_Steering_Angle;
  }

  myServo.write(STRAIGHT_ANGLE + s_angle); // 부호만 바꿀 것
}

// Threshold setting (moved to global section)

void setup()
{
    // WDT 초기 비활성화 (초기화 중 리셋 방지)
    wdt_disable();

    Serial.begin(115200);
    while(!Serial)
    {
        ;
    }
    Serial1.begin(115200);  // Serial1 디버깅용
    while(!Serial1)
    {
        ;
    }
    Serial1.println("Serial1 Debug Started at 115200 baud");
    Serial1.flush();
    Serial1.println("Serial1 initialized for debugging");
    Serial1.flush();

    pinMode(TSL1401_CLK, OUTPUT);
    pinMode(TSL1401_SI, OUTPUT);

    myServo.attach(SERVO_PIN);
    myServo.write(STRAIGHT_ANGLE);

    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);


    // Test pin setup
    pinMode(TSL1401_TEST_PIN, OUTPUT);
    pinMode(TOF_TEST_PIN, OUTPUT);
    pinMode(CONTROL_TEST_PIN, OUTPUT);
    pinMode(IMU_TEST_PIN, OUTPUT);
    
    // VL53L0X 센서 초기화
    VL530L0x_Sensor_Setup();

    // IMU 센서 초기화
    if (!IMU_Sensor_Setup()) {
        Serial1.println("WARNING: IMU sensor not available, continuing without IMU.");
        Serial1.flush();
        imu_initialized = false;
    }

    // 엔코더 초기화
    interrupt_setup();

    // TOF 필터 및 변수 초기화
    tofFilterR.reset();
    tofFilterC.reset();
    tofFilterL.reset();

    // 필터된 값들 명시적으로 0으로 초기화
    vl53l0x_distance_r = 0;
    vl53l0x_distance_c = 0;
    vl53l0x_distance_l = 0;

    // 필터 평균값들도 초기화
    tof_filter_avg_r = 0.0;
    tof_filter_avg_c = 0.0;
    tof_filter_avg_l = 0.0;
    imu_yaw_filter_avg = 0.0;

    Serial1.println("TOF filters and variables initialized");
    Serial1.flush();

#if FASTADC && defined(__AVR__) && defined(ADCSRA)
    // AVR 아키텍처에서만 ADC 속도 최적화 (Arduino Uno, Nano, Mega 등)
    // ADC 프리스케일러를 16으로 설정 (더 빠른 ADC 변환)
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);
#endif

    // 세마포어 생성
    xSerialSemaphore = xSemaphoreCreateMutex();
    xDataSemaphore = xSemaphoreCreateMutex();
    xSteeringSemaphore = xSemaphoreCreateMutex();
    xVL53L0XSemaphore = xSemaphoreCreateMutex();
    xIMUSemaphore = xSemaphoreCreateMutex();
    xI2CSemaphore = xSemaphoreCreateMutex();

    if (xSerialSemaphore != NULL && xDataSemaphore != NULL && xSteeringSemaphore != NULL && xVL53L0XSemaphore != NULL && xIMUSemaphore != NULL && xI2CSemaphore != NULL)
    {
        // 시리얼 송신 태스크 생성
        xTaskCreate(TaskSerial,
                    "Serial",
                    512, // 스택 크기 증가
                    NULL,
                    0, // 우선순위 (최저)
                    &xSerialTaskHandle);

        // 카메라 읽기 태스크 생성 (임계값 전달)
        xTaskCreate(TaskCamera,
                    "Camera",
                    1024,                       // 스택 크기 증가
                    (void *) &cameraThreshold, // threshold 값 전달
                    4,                         // 우선순위 (최고)
                    &xCameraTaskHandle);

        // 제어 태스크 생성
        xTaskCreate(TaskControl,
                    "Control",
                    512,    // 스택 크기
                    NULL,
                    1,      // 우선순위 (최저)
                    &xControlTaskHandle);

        // VL53L0X sensor task creation
        xTaskCreate(TaskVL53L0X,
                    "VL53L0X",
                    512,    // Stack size
                    NULL,
                    3,      // Priority (높음)
                    &xVL53L0XTaskHandle);

        // IMU sensor task creation
        xTaskCreate(TaskIMU,
                    "IMU",
                    256,    // Stack size
                    NULL,
                    2,      // Priority (중간)
                    &xIMUTaskHandle);
        // WDT 활성화 (2초 타임아웃)
        wdt_enable(WDTO_2S);

        // 태스크 스케줄러 시작
        vTaskStartScheduler();
    }
    else
    {
        // 세마포어 생성 실패 시 처리
        Serial1.println("Failed to create semaphore");
        Serial1.flush();
    }
}

//====================================================================
// FREERTOS TASK FUNCTIONS
//====================================================================

void TaskCamera(void *pvParameters)
{
    // pvParameters로 전달받은 threshold 값 추출
    int threshold = *((int *) pvParameters);

    const TickType_t xFrequency = pdMS_TO_TICKS(40); // 20ms = 50Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 태스크 실행 시작을 알리는 신호 (디버깅용)
        digitalWrite(TSL1401_TEST_PIN, HIGH);

        // WDT 리셋 (removed - Control task resets WDT)
        // wdt_reset();

        // 카메라 데이터 읽기
        read_TSL1401_camera();

        // 데이터 세마포어 획득하여 버퍼에 복사
        if (xSemaphoreTake(xDataSemaphore, portMAX_DELAY) == pdTRUE)
        {
            memcpy(PixelBuffer, Pixel, NPIXELS); // 버퍼에 복사
            xSemaphoreGive(xDataSemaphore);
        }

        // 임계값 처리 (0과 255는 특수 케이스)
        // 0: 모든 픽셀이 0 (전부 검정)
        // 255: 모든 픽셀이 그대로 (임계값 무시)
        line_threshold(threshold); // 전달받은 threshold 값 사용

        float center = find_line_center();

        // 데이터 세마포어로 보호하여 라인 중심 위치 저장
        if (xSemaphoreTake(xSteeringSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            line_center_pos = center;
            xSemaphoreGive(xSteeringSemaphore);
        }

        // PD 제어를 사용하여 조향각 계산
        vision_steer_angle = calculate_pd_steering(center);

        // 태스크 실행 종료를 알리는 신호 (디버깅용)
        digitalWrite(TSL1401_TEST_PIN, LOW);

        // 정확한 25Hz 주기 유지
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}



// 시리얼 통신 태스크 (송신만 수행)
void TaskSerial(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xDelay = pdMS_TO_TICKS(200); // 200ms 간격으로 데이터 전송 (초당 5회)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    byte localPixelBuffer[NPIXELS]; // 로컬 버퍼

    for (;;)
    {
        // WDT 리셋 (removed - only Camera task resets WDT)
        // wdt_reset();

        // 100ms마다 픽셀 데이터 전송
        if (xSemaphoreTake(xDataSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(localPixelBuffer, PixelBuffer, NPIXELS); // 로컬 버퍼에 복사
            xSemaphoreGive(xDataSemaphore);

            // 시리얼로 데이터 전송 (Serial 0번 사용)
            if (xSemaphoreTake(xSerialSemaphore, portMAX_DELAY) == pdTRUE)
            {
                // 128개 픽셀 데이터 전송 (Serial0로 카메라 데이터)
                for (int i = 0; i < NPIXELS; i++)
                {
                    Serial.print(localPixelBuffer[i]);
                    Serial.print(",");
                }

                // 라인 중심 위치 값 추가 (129번째 값)
                float center_value = line_center_pos;
                float current_steer_angle = vision_steer_angle;
                if (xSemaphoreTake(xSteeringSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    center_value = line_center_pos;
                    current_steer_angle = vision_steer_angle;
                    xSemaphoreGive(xSteeringSemaphore);
                }
                Serial.print(center_value);
                Serial.println();
                
                // Serial1로 디버깅 정보 출력
                Serial1.print("LC:");
                Serial1.print(center_value, 1);
                Serial1.print(",SA:");
                Serial1.print(current_steer_angle, 1);

                // TOF sensor values - both raw and filtered (protected by semaphore)
                uint16_t tof_r_filtered = 0, tof_c_filtered = 0, tof_l_filtered = 0;
                uint16_t tof_r_raw = 0, tof_c_raw = 0, tof_l_raw = 0;
                if (xSemaphoreTake(xVL53L0XSemaphore, pdMS_TO_TICKS(1)) == pdTRUE) {
                    tof_r_filtered = vl53l0x_distance_r;
                    tof_c_filtered = vl53l0x_distance_c;
                    tof_l_filtered = vl53l0x_distance_l;
                    tof_r_raw = vl53l0x_raw_r;
                    tof_c_raw = vl53l0x_raw_c;
                    tof_l_raw = vl53l0x_raw_l;
                    xSemaphoreGive(xVL53L0XSemaphore);
                }

                // Display raw values
                Serial1.print(",RAW[R:");
                Serial1.print(tof_r_raw);
                Serial1.print(",C:");
                Serial1.print(tof_c_raw);
                Serial1.print(",L:");
                Serial1.print(tof_l_raw);

                // Display filtered values
                Serial1.print("],FLT[R:");
                Serial1.print(tof_r_filtered);
                Serial1.print(",C:");
                Serial1.print(tof_c_filtered);
                Serial1.print(",L:");
                Serial1.print(tof_l_filtered);

                // IMU yaw angle (protected by semaphore)
                float yaw = 0.0;
                if (xSemaphoreTake(xIMUSemaphore, pdMS_TO_TICKS(1)) == pdTRUE) {
                    yaw = imu_yaw_filtered;  // Use filtered yaw value
                    xSemaphoreGive(xIMUSemaphore);
                }
                Serial1.print("],Yaw:");
                Serial1.print(yaw, 1);

                Serial1.print(",MF:");
                Serial1.print(mission_flag);
                Serial1.print(",Servo:");
                Serial1.print(STRAIGHT_ANGLE + current_steer_angle);
                Serial1.print(",Enc:");
                Serial1.println(encoderPos);
                
                xSemaphoreGive(xSerialSemaphore);
            }
        }

        // 정확한 200ms 주기 유지
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

// 제어 태스크 (미션 수행)
void TaskControl(void *pvParameters)
{
    (void) pvParameters;

    const TickType_t xFrequency = pdMS_TO_TICKS(40); // 40ms = 25Hz (서보 제어 주기)
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        digitalWrite(CONTROL_TEST_PIN, HIGH);
        wdt_reset(); // Control task resets WDT (shortest execution time)

        switch (mission_flag)
        {
            case 0:
                // start_stop 미션 로직
                if( vl53l0x_distance_c < 100)
                {
                    motor_control(true, 0);
                }
                else
                {
                    mission_flag = 1;
                }
                break;
            case 1:
                // lane_control 미션 로직
                motor_control(true, 250);
                Steering_Control(vision_steer_angle);

                // 제어 상태는 Serial Task에서 출력됨
                // maze inlet detection with continuous detection logic
                {
                    static int maze_detection_count = 0;
                    const int MAZE_DETECTION_THRESHOLD = 3;  // 3회 연속 감지

                    if (detect_maze_entrance()) 
                    {
                        maze_detection_count++;
                        if (maze_detection_count >= MAZE_DETECTION_THRESHOLD) 
                        {
                            motor_control(true, 0);
                            mission_flag = 2;
                            maze_detection_count = 0;  // 리셋
                            current_yaw_angle = imu_yaw_filtered;  // read filtered imu yaw angle
                        }
                    } 
                    else 
                    {
                        maze_detection_count = 0;  // 조건 미만족 시 리셋
                    }
                }

                break;
            case 2:  // turn90 미션 로직
                car_yaw_control(current_yaw_angle - 90);

                break;
            case 3:
               
                break;
            case 4:
                // 추가 미션 로직
                break;
            case 5:
                // 90도 turn 미션 로직
                break;
            case 6:
                // lane_control 미션 로직
                break;
            default:
                mission_flag = 0;
                break;
        }
        digitalWrite(CONTROL_TEST_PIN, LOW);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }


    // 태스크 제어 예제:
    // Enable_Camera();
    // Disable_Camera();
    // Enable_Serial();
    // Disable_Serial();
    // Enable_VL53L0X();
    // Disable_VL53L0X();
}

// VL53L0X 센서 태스크
void TaskVL53L0X(void *pvParameters)
{
    (void) pvParameters;

    // 측정 주기 설정:
    // - VL53L0X는 최대 50Hz(20ms) 까지 안정적으로 동작
    // - 벽 추종: 30ms (33.3Hz) - 안정적인 반응
    // - 일반 감지: 30ms (33.3Hz) - 안정적이고 신뢰성 높은 감지
    const TickType_t xFrequency = pdMS_TO_TICKS(40); // 40ms = 25Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 태스크 실행 시작 신호 (디버깅용) - 센서 초기화와 관계없이 동작
         digitalWrite(TOF_TEST_PIN, HIGH);
        
        // WDT 리셋 (removed - only Camera task resets WDT)
        // wdt_reset();

        // VL53L0X sensor measurement only if initialized
        if (vl53l0x_initialized)
        {
            // Protect I2C bus access
            if (xSemaphoreTake(xI2CSemaphore, pdMS_TO_TICKS(5)) == pdTRUE) 
            {
                // Read all three TOF sensors
                uint16_t dist_r = sensorR.readRangeContinuousMillimeters();
                uint16_t dist_c = sensorC.readRangeContinuousMillimeters();
                uint16_t dist_l = sensorL.readRangeContinuousMillimeters();

                xSemaphoreGive(xI2CSemaphore);

                // Protect distance values with semaphore
                if (xSemaphoreTake(xVL53L0XSemaphore, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    // Store raw values
                    vl53l0x_raw_r = dist_r;
                    vl53l0x_raw_c = dist_c;
                    vl53l0x_raw_l = dist_l;

                    // Apply filter to all sensors with proper method
                    if (dist_r >= 20 && dist_r <= 2000) 
                    {
                        tof_filter_avg_r = tofFilterR.update((float)dist_r, tof_filter_avg_r);
                        vl53l0x_distance_r = (uint16_t)tof_filter_avg_r;
                    } 
                    else 
                    {
                        vl53l0x_distance_r = dist_r;
                    }

                    if (dist_c >= 20 && dist_c <= 2000) {
                        tof_filter_avg_c = tofFilterC.update((float)dist_c, tof_filter_avg_c);
                        vl53l0x_distance_c = (uint16_t)tof_filter_avg_c;
                    } else {
                        vl53l0x_distance_c = dist_c;
                    }

                    if (dist_l >= 20 && dist_l <= 2000) {
                        tof_filter_avg_l = tofFilterL.update((float)dist_l, tof_filter_avg_l);
                        vl53l0x_distance_l = (uint16_t)tof_filter_avg_l;
                    } else {
                        vl53l0x_distance_l = dist_l;
                    }

                    /*  // Debug output (commented out)
                    static int debug_count = 0;
                    if (debug_count % 20 == 0) {
                        float filterArray[5];
                        tofFilterR.getSensorData(filterArray);
                        Serial1.print("FILTER_ARRAY[");
                        for (int i = 0; i < 5; i++) {
                            Serial1.print(filterArray[i]);
                            if (i < 4) Serial1.print(",");
                        }
                        Serial1.print("] RAW:");
                        Serial1.print(dist_r);
                        Serial1.print(" AVG:");
                        Serial1.print(tof_filter_avg_r);
                        Serial1.print(" FLT:");
                        Serial1.println(vl53l0x_distance_r);
                        Serial1.flush();
                    }
                    debug_count++;
                    */

                    xSemaphoreGive(xVL53L0XSemaphore);
                }
            }
        }

        // 태스크 실행 종료 신호 (디버깅용)
        digitalWrite(TOF_TEST_PIN, LOW);

        // 정확한 주기 유지
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// IMU sensor task
void TaskIMU(void *pvParameters)
{
    (void) pvParameters;

    // Measurement period setting:
    // - IMU data should be read frequently for accurate orientation
    // - 50ms (20Hz) provides good balance between accuracy and performance
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50ms = 20Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // 태스크 실행 시작 신호 (디버깅용)
        digitalWrite(IMU_TEST_PIN, HIGH);

        // WDT reset (removed - only Camera task resets WDT)
        // wdt_reset();

        // Read IMU data if initialized
        read_IMU_data();

        // 태스크 실행 종료 신호 (디버깅용)
        digitalWrite(IMU_TEST_PIN, LOW);

        // Maintain accurate period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Test variables for filter simulation
RecursiveMovingAverage testFilter;
uint16_t test_raw_values[] = {100, 150, 200, 250, 300, 350, 400, 450, 500, 8190, 8191, 50, 75, 125};
int test_index = 0;
float test_filtered = 0.0;

void loop()
{
    // Re-enable tasks with properly initialized filters
    Enable_Serial();
    Enable_VL53L0X();
    vTaskDelay(pdMS_TO_TICKS(10));
}
