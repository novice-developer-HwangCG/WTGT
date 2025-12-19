MAC 주소

1번 WTGT
송신기(controller) - 0xEC,0xE3,0x34,0x1A,0xCF,0xA4
수신기(WTGT) - 0xEC,0xE3,0x34,0x1A,0xA5,0x78

마지막 업데이트 코드
1208_update_new_controller.c (송신기) -> 1204_update_new_controller.c도 사용 가능
1204_update_new_motor.c (수신기)

* 추 후 controller 회로 배선 작업 필요 사항
 - 가변저항 볼륨 RC필터 추가 필요 (예시 1kΩ + 0.1µF)
 - 전원 공급부에 커패시터 추가 필요 (약 0.1uF? or 10uF?)
 - 방향핀을 adc gpio핀으로 사용중이여서 이 부분 땜에 노이즈가 있음 바꿔줘야 함

 = 노이즈, 배선 정리 등에 문제로 가변저항 값이 올라 갔다가 떨어지길 반복 및 방향핀 바꾸면 adc 값 변동 심함

 * 송신기
 - tsr 1 2450e 사용 5V 전압 공급
 - SW1 = PWR, SW2 = ENABLE, SW3 = DIRECTION, POC = PWM(가변저항 볼륨), SW4 = NULL (연결 X)
 - 모든 라인 3.3V 사용
 - LED

 * 수신기
 - tsr 1 2450e 사용 5V 전압 공급
 - 모터 드라이버 전압은 배터리 전압 그대로 받아 사용
 - ENABLE, DIRECTION, PWM <- LVF 5V to 3.3V -> ENABLE, DIRECTION, PWM