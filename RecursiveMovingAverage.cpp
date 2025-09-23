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

    // 새로운 평균 계산
    avg = avgOld + (sensorData[windowSize - 1] - oldValue) / windowSize;

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