# WTGT
Work in progress — detailed description coming soon

Use ESP-NOW

1. Transmitter
   - PWR On/Off
   - Motor Start/Stop
   - Direction Left/Right
   - Volume 0 ~ 3.3V
   - LED
  
24V → TSR-1-2450e → 5V (ESP32)

SW1 = PWR

SW2 = Enable

SW3 = Direction

SW4 = RSV

POC = Volume

LED = pin LED

2. Receiver
   - PWR On/Off
   - Get Enable Signal
   - Get Direction Signal
  
24V → TSR-1-2450e → 5V (ESP32)

24V → Motor Driver

Motor Driver - Enable = Level Shift 5V to 3.3V → ESP32

Motor Driver - Direction = Level Shift 5V to 3.3V → ESP32

Motor Driver - PWM = Level Shift 5V to 3.3V → ESP32

<--- 251208 --->
* 추 후 controller 회로 배선 작업 필요 사항
 - 가변저항 볼륨 RC필터 추가 필요 (예시 1kΩ + 0.1µF)
 - 전원 공급부에 커패시터 추가 필요 (약 0.1uF? or 10uF?)
 - 방향핀을 adc gpio핀으로 사용중이여서 이 부분 땜에 노이즈가 있음 바꿔줘야 함

 = 노이즈, 배선 정리 등에 문제로 가변저항 값이 올라 갔다가 떨어지길 반복 및 방향핀 바꾸면 adc 값 변동 심함
