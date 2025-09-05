#include <Arduino_FreeRTOS.h>
#include <Servo.h>
#include <avr/io.h>  // AVR 레지스터 정의 포함
#include <avr/wdt.h> // Watchdog Timer 헤더
#include <semphr.h>

// Camera Control
#define TSL1401_CLK 11 // TSL1401 클럭 핀
#define TSL1401_SI 10  // TSL1401 SI 핀
#define TSL1401_AO A0  // TSL1401 아날로그 출력 핀
#define NPIXELS 128    // 총 픽셀 수
#define Camera_calibration 2.1  // 펙셀을 mm로 바꾸는 계수
#define H 250.0 // 카메라와 바퀴축 사이 거리 mm

// Motor Control
#define MOTOR_DIR_PIN 4
#define MOTOR_PWM_PIN 5
#define encodPinA1   2
#define encodPinB1   3

// Servo control
#define SERVO_PIN 8
#define STRAIGHT_ANGLE 87
#define MAX_LEFT_Steering_Angle   -30
#define MAX_RIGHT_Steering_Angle   30

#define FASTADC 1 // ADC 속도 빠르게 설정

// ADC 레지스터 조작을 위한 매크로 정의 (FASTADC에서 사용)
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

// 세마포어 선언
SemaphoreHandle_t xSerialSemaphore;
SemaphoreHandle_t xDataSemaphore; // 픽셀 데이터 보호용
SemaphoreHandle_t xSteeringSemaphore; // 조향 데이터 보호용

// 태스크 핸들 선언
TaskHandle_t xCameraTaskHandle = NULL;
TaskHandle_t xSerialTaskHandle = NULL;
TaskHandle_t xControlTaskHandle = NULL;

// 태스크 함수 선언
void TaskSerial(void *pvParameters);
void TaskCamera(void *pvParameters);
void TaskControl(void *pvParameters);

// Servo 객체
Servo myServo;

// H 값 정의 (calculate_yaw_angle_y_axis 함수에서 사용)


// 태스크 제어 플래그
volatile bool enableCamera = true;      // true: 카메라 실행, false: 카메라 정지
volatile bool cameraTaskRunning = true; // 현재 카메라 태스크 상태
volatile bool serialTaskRunning = true; // 현재 시리얼 태스크 상태

// 조향 제어 변수
volatile float steering_angle = 0.0;    // 계산된 조향 각도
volatile float vision_steer_angle = 0.0; // 비전 기반 조향 각도
volatile float line_center_pos = NPIXELS / 2.0; // 라인 중심 위치 (무게 중심)
volatile int mission_flag = 0;          // 미션 상태 플래그

// PD 제어 변수
float Kp = 1.5;  // 비례 게인 (라인 중심 오차에 대한 반응)
float Kd = 0.8;  // 미분 게인 (오차 변화율에 대한 반응)
float previous_error = 0.0;  // 이전 오차값 저장

byte Pixel[NPIXELS];          // 원본 픽셀 데이터
byte PixelBuffer[NPIXELS];    // 전송용 버퍼
byte threshold_data[NPIXELS]; // 임계값 처리된 데이터

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
            Serial.println("Camera task enabled");
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
            Serial.println("Camera task disabled");
            xSemaphoreGive(xSerialSemaphore);
        }
    }
}

// 시리얼 태스크 제어 함수
void Enable_Serial()
{
    if (xSerialTaskHandle != NULL && !serialTaskRunning)
    {
        vTaskResume(xSerialTaskHandle);
        serialTaskRunning = true;

        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial.println("Serial task enabled");
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
        
        // 시리얼 태스크가 비활성화되면 마지막 메시지 출력
        if (xSemaphoreTake(xSerialSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            Serial.println("Serial task disabled");
            xSemaphoreGive(xSerialSemaphore);
        }
    }
}

// 임계값 기반 이진화 함수 (값이 threshold 이상이면 1, 아니면 0)
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

// 카메라 데이터 읽기 (analogRead / 4 → 0~255 범위로 변환)
void read_TSL1401_camera()
{
    // SI 펄스 시작
    digitalWrite(TSL1401_SI, HIGH);
    digitalWrite(TSL1401_CLK, HIGH);
    delayMicroseconds(1);
    digitalWrite(TSL1401_CLK, LOW);
    digitalWrite(TSL1401_SI, LOW);
    delayMicroseconds(1);

    // 픽셀 값 읽기
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
// MOTOR CONTROL FUNCTIONS
//====================================================================

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

// PD 제어 함수 - 라인 중심 오차를 기반으로 조향각 계산
float calculate_pd_steering(float line_center)
{
    // 오차 계산: 중심(64)에서 벗어난 정도
    // 오차가 양수면 라인이 오른쪽에 있음 -> 오른쪽으로 조향
    // 오차가 음수면 라인이 왼쪽에 있음 -> 왼쪽으로 조향
    float error = line_center - (NPIXELS / 2.0);  // 64픽셀이 중심
    
    // 미분항 계산 (오차 변화율)
    float derivative = error - previous_error;
    
    // PD 제어 출력 계산
    float steering_output = (Kp * error) + (Kd * derivative);
    
    // 이전 오차값 업데이트
    previous_error = error;
    
    // 조향각 제한 (-30 ~ +30도)
    if (steering_output > 30.0) {
        steering_output = 30.0;
    } else if (steering_output < -30.0) {
        steering_output = -30.0;
    }
    
    return steering_output;
}

//====================================================================
// ENCODER FUNCTIONS
//====================================================================

volatile long encoderPos = 0;

void encoderB()
{
  delayMicroseconds(2);
  if (digitalRead(encodPinB1) == LOW) encoderPos++;
  else                                encoderPos--;
}

void reset_encoder(void)
{
  encoderPos = 0;
}

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);        // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);        // quadrature encoder input B
  attachInterrupt(0, encoderB, FALLING);    // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;         // To prevent Motor Noise
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

// 임계값 설정 (setup 내에서 static으로 선언하여 메모리에 유지)
static int cameraThreshold = 70; // 기본값 128, 필요에 따라 변경 가능

void setup()
{
    // WDT 초기 비활성화 (초기화 중 리셋 방지)
    wdt_disable();

    Serial.begin(115200);
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
    Serial1.begin(115200);  // Serial1 디버깅용 (Mega, Leonardo 등)
    Serial1.println("Serial1 Debug Started at 115200 baud");
    Serial.println("Serial1 initialized for debugging");
    #else
    Serial.println("Serial1 not available on this board");
    #endif
    while (!Serial)
    {
        ; // 시리얼 포트가 연결될 때까지 대기
    }

    pinMode(TSL1401_CLK, OUTPUT);
    pinMode(TSL1401_SI, OUTPUT);

    myServo.attach(SERVO_PIN);
    myServo.write(STRAIGHT_ANGLE);

    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    
    // 엔코더 초기화
    interrupt_setup();

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

    if (xSerialSemaphore != NULL && xDataSemaphore != NULL && xSteeringSemaphore != NULL)
    {
        // 시리얼 송신 태스크 생성
        xTaskCreate(TaskSerial,
                    "Serial",
                    256, // 스택 크기 증가
                    NULL,
                    2, // 우선순위
                    &xSerialTaskHandle);

        // 카메라 읽기 태스크 생성 (임계값 전달)
        xTaskCreate(TaskCamera,
                    "Camera",
                    256,                       // 스택 크기 증가
                    (void *) &cameraThreshold, // threshold 값 전달
                    1,                         // 우선순위
                    &xCameraTaskHandle);

        // 제어 태스크 생성
        xTaskCreate(TaskControl,
                    "Control",
                    256,    // 스택 크기
                    NULL,
                    3,      // 우선순위 (가장 높음)
                    &xControlTaskHandle);

        // WDT 활성화 (2초 타임아웃)
        wdt_enable(WDTO_2S);

        // 태스크 스케줄러 시작
        vTaskStartScheduler();
    }
    else
    {
        // 세마포어 생성 실패 시 처리
        Serial.println("Failed to create semaphore");
    }
}

//====================================================================
// FREERTOS TASK FUNCTIONS
//====================================================================

void TaskCamera(void *pvParameters)
{
    // pvParameters로 전달받은 threshold 값 추출
    int threshold = *((int *) pvParameters);

    const TickType_t xFrequency = pdMS_TO_TICKS(40); // 40ms = 25Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        // WDT 리셋 (태스크가 정상 동작 중임을 알림)
        wdt_reset();

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
        // WDT 리셋 (태스크가 정상 동작 중임을 알림)
        wdt_reset();

        // 100ms마다 픽셀 데이터 전송
        if (xSemaphoreTake(xDataSemaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(localPixelBuffer, PixelBuffer, NPIXELS); // 로컬 버퍼에 복사
            xSemaphoreGive(xDataSemaphore);

            // 시리얼로 데이터 전송 (Serial 0번 사용)
            if (xSemaphoreTake(xSerialSemaphore, portMAX_DELAY) == pdTRUE)
            {
                // 128개 픽셀 데이터 전송
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
                
                // Serial1로 디버깅 정보 출력 (Mega, Leonardo 등에서만)
                #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
                Serial1.print("LC:");
                Serial1.print(center_value, 1);
                Serial1.print(",SA:");
                Serial1.print(current_steer_angle, 1);
                Serial1.print(",MF:");
                Serial1.print(mission_flag);
                Serial1.print(",Servo:");
                Serial1.println(STRAIGHT_ANGLE + current_steer_angle);
                #endif
                
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

    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms = 50Hz (서보 제어 주기)
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        wdt_reset();

        switch (mission_flag)
        {
            case 0:
                // start_stop 미션 로직
                break;
            case 1:
                // lane_control 미션 로직
                motor_control(true, 200);
                Steering_Control(vision_steer_angle);
                
                // 제어 상태는 Serial Task에서 출력됨
                break;
            case 2:
                // wall_following 미션 로직
                break;
            case 3:
                // turn90 미션 로직
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

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }


    //Enable_Camera();
    //Enable_Serial();
    //Disable_Serial();
}

void loop()
{
    // 테스트를 위해 mission_flag를 1로 설정하여 모터 제어 활성화
    mission_flag = 1;
    Enable_Serial();
    vTaskDelay(pdMS_TO_TICKS(10));
}
