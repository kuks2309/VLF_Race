# VLF Racer Camera Viewer 사용법

## 개요
VLF Racer Camera Viewer는 TSL1401 라인 센서의 128픽셀 데이터를 실시간으로 시각화하는 Python 프로그램입니다.

## 필요 패키지 설치

### Windows
```bash
pip install pyserial matplotlib numpy
```

### Linux/macOS
```bash
pip3 install pyserial matplotlib numpy
```

## Windows에서 COM 포트 설정

### 1. COM 포트 확인하기

#### 방법 1: 장치 관리자 사용
1. **Windows + X** 키를 누르고 **장치 관리자** 선택
2. **포트(COM & LPT)** 항목 확장
3. Arduino가 연결된 COM 포트 확인 (예: "Arduino Uno (COM3)")

#### 방법 2: Arduino IDE 사용
1. Arduino IDE 열기
2. **도구 → 포트** 메뉴에서 연결된 Arduino 확인

#### 방법 3: 명령 프롬프트 사용
```cmd
mode
```
또는 PowerShell에서:
```powershell
[System.IO.Ports.SerialPort]::getportnames()
```

### 2. Camera Viewer 실행

#### Windows 명령어 예제:
```bash
# COM3 포트 사용 시
python camera_viewer.py -p COM3 -b 115200

# COM7 포트 사용 시
python camera_viewer.py -p COM7

# 기본값 사용 (수정 필요)
python camera_viewer.py
```

## Linux에서 포트 설정

### 1. 포트 확인
```bash
# 연결된 USB 장치 확인
ls /dev/ttyACM* /dev/ttyUSB*

# 또는 dmesg로 확인
dmesg | grep tty
```

### 2. 권한 설정
```bash
# 일시적 권한 부여
sudo chmod 666 /dev/ttyACM0

# 영구적 권한 설정 (사용자를 dialout 그룹에 추가)
sudo usermod -a -G dialout $USER
# 로그아웃 후 다시 로그인 필요
```

### 3. 실행
```bash
# /dev/ttyACM0 사용 (기본값)
python3 camera_viewer.py

# /dev/ttyUSB0 사용
python3 camera_viewer.py -p /dev/ttyUSB0
```

## macOS에서 포트 설정

### 1. 포트 확인
```bash
ls /dev/tty.usb*
```

### 2. 실행
```bash
python3 camera_viewer.py -p /dev/tty.usbmodem14101
```

## 프로그램 사용법

### 명령줄 옵션
- `-p`, `--port`: 시리얼 포트 지정
- `-b`, `--baudrate`: 통신 속도 지정 (기본: 115200)

### 실행 예제

#### Windows:
```bash
# COM3, 115200 baud
python camera_viewer.py -p COM3 -b 115200

# COM5, 기본 baudrate
python camera_viewer.py -p COM5
```

#### Linux:
```bash
# 기본 설정 (/dev/ttyACM0, 115200)
python3 camera_viewer.py

# 특정 포트 지정
python3 camera_viewer.py -p /dev/ttyUSB0
```

## 화면 설명

### 상단 그래프 (Line Plot)
- **파란선**: 128개 픽셀의 실시간 값 (0-255)
- **빨간 점선**: 감지된 라인 중심 위치
- **왼쪽 상단 박스**: 현재 라인 중심값과 오프셋

### 하단 그래프 (Bar Graph)
- **파란 막대**: 각 픽셀의 값을 막대 그래프로 표시
- **빨간 수직선**: 라인 중심 위치
  - 녹색: 중앙 근처 (오프셋 < 10)
  - 빨간색: 중앙에서 벗어남 (오프셋 > 10)

## 문제 해결

### Windows에서 "포트를 열 수 없음" 오류
1. **COM 포트 번호 확인**: 장치 관리자에서 정확한 포트 확인
2. **다른 프로그램 종료**: Arduino IDE 시리얼 모니터 닫기
3. **관리자 권한**: 명령 프롬프트를 관리자 권한으로 실행

### Linux에서 "Permission denied" 오류
```bash
# 일시적 해결
sudo python3 camera_viewer.py -p /dev/ttyACM0

# 영구적 해결
sudo usermod -a -G dialout $USER
# 재로그인 필요
```

### "ModuleNotFoundError" 오류
필요한 패키지 설치:
```bash
pip install pyserial matplotlib numpy
```

### 데이터가 표시되지 않을 때
1. **Arduino 코드 확인**: Serial.print로 데이터 전송 중인지 확인
2. **Baudrate 확인**: Arduino와 동일한 속도 사용 (115200)
3. **데이터 형식 확인**: 128개 픽셀값 + 1개 중심값 (쉼표 구분)

## Arduino 시리얼 출력 형식
Arduino는 다음 형식으로 데이터를 전송해야 합니다:
```
pixel0,pixel1,pixel2,...,pixel127,center_value
```
예시:
```
45,67,89,120,...,45,64.5
```

## 종료
- **Ctrl+C** 키를 눌러 프로그램 종료
- 창을 닫아도 종료됨

## 주의사항
- Arduino 시리얼 모니터와 동시 사용 불가
- 한 번에 하나의 프로그램만 시리얼 포트 사용 가능
- Windows는 COM1~COM256 범위의 포트 사용
- Linux는 /dev/ttyACM* 또는 /dev/ttyUSB* 형식