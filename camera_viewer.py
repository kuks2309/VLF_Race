#!/usr/bin/env python3
"""
VLF Racer Camera Viewer
실시간으로 TSL1401 라인 센서 데이터를 시각화합니다.
128개 픽셀 데이터와 라인 중심 위치를 표시합니다.
"""

import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import argparse

class CameraViewer:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        """
        카메라 뷰어 초기화
        
        Args:
            port: 시리얼 포트 (기본: /dev/ttyUSB0)
            baudrate: 통신 속도 (기본: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.pixels = np.zeros(128)
        self.line_center = 64  # 초기 중심값
        
        # 시리얼 연결
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            print(f"Connected to {port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {port}")
            print(f"Details: {e}")
            sys.exit(1)
        
        # 플롯 설정
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # 상단: 라인 센서 데이터
        self.line1, = self.ax1.plot([], [], 'b-', label='Pixel Data')
        self.center_line1 = self.ax1.axvline(x=64, color='r', linestyle='--', 
                                              linewidth=2, label='Line Center')
        self.ax1.set_xlim(0, 127)
        self.ax1.set_ylim(0, 255)
        self.ax1.set_xlabel('Pixel Index')
        self.ax1.set_ylabel('Pixel Value')
        self.ax1.set_title('TSL1401 Line Sensor Data')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.legend(loc='upper right')
        
        # 하단: 바 그래프 형태로 표시
        self.bars = self.ax2.bar(range(128), np.zeros(128), color='blue', alpha=0.7)
        self.center_line2 = self.ax2.axvline(x=64, color='r', linestyle='-', 
                                              linewidth=3, label='Line Center')
        self.ax2.set_xlim(-0.5, 127.5)
        self.ax2.set_ylim(0, 255)
        self.ax2.set_xlabel('Pixel Index')
        self.ax2.set_ylabel('Pixel Value')
        self.ax2.set_title('Bar Graph View')
        self.ax2.legend(loc='upper right')
        
        # 중심 위치 텍스트
        self.center_text = self.ax1.text(0.02, 0.95, '', transform=self.ax1.transAxes,
                                         fontsize=12, verticalalignment='top',
                                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        
    def read_serial_data(self):
        """시리얼 데이터 읽기"""
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    values = line.split(',')
                    if len(values) >= 129:  # 128 픽셀 + 1 중심값
                        # 픽셀 데이터 (0~127)
                        self.pixels = np.array([int(v) for v in values[:128]])
                        # 라인 중심 위치 (129번째 값)
                        self.line_center = float(values[128])
                        return True
        except (serial.SerialException, ValueError, UnicodeDecodeError) as e:
            print(f"Read error: {e}")
        return False
        
    def update_plot(self, frame):
        """플롯 업데이트"""
        if self.read_serial_data():
            # 라인 플롯 업데이트
            self.line1.set_data(range(128), self.pixels)
            
            # 바 그래프 업데이트
            for i, bar in enumerate(self.bars):
                bar.set_height(self.pixels[i])
            
            # 중심선 업데이트 (붉은색 수직선)
            self.center_line1.set_xdata([self.line_center, self.line_center])
            self.center_line2.set_xdata([self.line_center, self.line_center])
            
            # 중심 위치 텍스트 업데이트
            self.center_text.set_text(f'Line Center: {self.line_center:.1f}\n'
                                     f'Offset: {self.line_center - 64:.1f}')
            
            # 중심이 치우친 방향에 따라 색상 변경
            if abs(self.line_center - 64) > 10:
                self.center_line1.set_color('red')
                self.center_line2.set_color('red')
                self.center_line1.set_linewidth(3)
                self.center_line2.set_linewidth(4)
            else:
                self.center_line1.set_color('green')
                self.center_line2.set_color('green')
                self.center_line1.set_linewidth(2)
                self.center_line2.set_linewidth(3)
                
        return self.line1, self.center_line1, self.center_line2, self.bars, self.center_text
    
    def start(self):
        """애니메이션 시작"""
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, 
                                blit=False, cache_frame_data=False)
        plt.show()
        
    def close(self):
        """리소스 정리"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            print("Serial port closed")

def main():
    parser = argparse.ArgumentParser(description='VLF Racer Camera Viewer')
    parser.add_argument('-p', '--port', default='/dev/ttyACM0', 
                       help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baudrate (default: 115200)')
    
    args = parser.parse_args()
    
    viewer = CameraViewer(args.port, args.baudrate)
    
    try:
        print("Starting camera viewer...")
        print("Press Ctrl+C to exit")
        viewer.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        viewer.close()

if __name__ == "__main__":
    main()