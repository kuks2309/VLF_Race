#ifndef RECURSIVE_MOVING_AVERAGE_H
#define RECURSIVE_MOVING_AVERAGE_H

// 기본 윈도우 크기 정의 (필요에 따라 수정 가능)
#ifndef WINDOW_SIZE
#define WINDOW_SIZE 5
#endif

class RecursiveMovingAverage
{
  private:
    float sensorData[WINDOW_SIZE];
    int windowSize;

  public:
    // 생성자
    RecursiveMovingAverage();

    // 새로운 값으로 이동평균 업데이트
    float update(float adValue, float avgOld);

    // 배열 초기화 (선택사항)
    void reset();

    // 현재 윈도우 크기 반환
    int getWindowSize() const;
};

#endif // RECURSIVE_MOVING_AVERAGE_H