WTGT 본체 PCB 추가

모터 MS-BL7045W-G1
모터드라이버 BLC-151

소유중 송신기 사양
Chip type:          ESP32-D0WDQ6 (revision v1.1)
Features:           Wi-Fi, BT, Dual Core + LP Core, 240MHz, Vref calibration in eFuse, Coding Scheme None
Crystal frequency:  40MHz
MAC:                94:54:c5:b5:d3:00

소유중 수신기 사양
Chip type:          ESP32-D0WD-V3 (revision v3.1)
Features:           Wi-Fi, BT, Dual Core + LP Core, 240MHz, Vref calibration in eFuse, Coding Scheme None
Crystal frequency:  40MHz
MAC:                ec:e3:34:1b:61:d8

<--- ESP32 (기판) --->
1. Vcc / GND (3pin)
 - 5V
 - 3.3V
 - GND
 → PCB J4

2. HIT_{num} → ESP32 (3pin)
 - HIT_1
 - HIT_2
 - HIT_3
 → PCB J11

3. Motor Driver ← ESP32 (4pin)
 - ENA_L
 - DIR_L
 - PWM_L
 - GND
 → PCB J14 

4. LED ← ESP32 (4pin)
 - LA
 - LB
 - LC
 - LD
 → PCB J12

<--- PCB (외부 IN) --->
1. Vcc / GND (2pin)
 - 24V
 - GND
 → PCB J3

2. HIT_IN_{num} (4pin)
 - HIT_IN_1
 - HIT_IN_2
 - HIT_IN_3
 - GND
 → PCB J9

3. LED out (5pin)
 - LA+
 - LB+
 - LC+
 - LD+
 - GND
 → PCB J13

4. Motor driver (4pin)
 - ENA_H
 - DIR_H
 - PWM_H
 - GND
 → PCB J15

<--- ESP32 PIN SET --->
1. Motor
 - PWM = gpio27
 - DIR = gpio19
 - ENA = gpio17

2. Hit
 - HIT_1 = gpio34
 - HIT_2 = gpio39
 - HIT_3 = gpio35
 * ESP32 pin 34, 35, 36, 39 [INPUT ONLY]는 내부 풀업/풀다운이 약하거나 없다고 보고 외부 저항으로 안정화하는 게 안전

3. LED
 - LED_A = gpio23
 - LED_B = gpio25
 - LED_C = gpio26
 - LED_D = gpio22
 * 각 저항 1k
 * 기판 기구부 고려 작업

