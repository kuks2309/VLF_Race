#include "RecursiveMovingAverage.h"

// 생성자
RecursiveMovingAverage::RecursiveMovingAverage() : windowSize(WINDOW_SIZE)
{
    for (int i = 0; i < windowSize; i++)
    {
        sensorData[i] = 0.0;
    }
}

// 새로운 값으로 이동평균 업데이트
float RecursiveMovingAverage::update(float adValue, float avgOld)
{
    float avg = 0.0;
    float oldValue = sensorData[0];

    // 배열을 왼쪽으로 시프트
    for (int i = 0; i < windowSize - 1; i++)
    {
        sensorData[i] = sensorData[i + 1];
    }
    sensorData[windowSize - 1] = adValue;

    // 새로운 평균 계산 - 배열 전체의 평균을 직접 계산
    for (int i = 0; i < windowSize; i++)
    {
        avg += sensorData[i];
    }
    avg /= windowSize;

    return avg;
}

// 배열 초기화 (선택사항)
void RecursiveMovingAverage::reset()
{
    for (int i = 0; i < windowSize; i++)
    {
        sensorData[i] = 0.0;
    }
}

// 현재 윈도우 크기 반환
int RecursiveMovingAverage::getWindowSize() const
{
    return windowSize;
}

// 센서 데이터 배열 읽기
void RecursiveMovingAverage::getSensorData(float* data) const
{
    for (int i = 0; i < windowSize; i++)
    {
        data[i] = sensorData[i];
    }
}