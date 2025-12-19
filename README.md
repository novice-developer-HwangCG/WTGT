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

