#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/*
수신기 코드

모터 현재 급하게 짜 맞춘거라 나중에 계측해서 다시 해야 할 수 도 있음 현재는 잘 되니 냅둘 것
*/

#define PIN_EN   21   // Enable: LOW=정지, HIGH=동작
#define PIN_DIR  18   // Direction: LOW=역방향, HIGH=정방향
#define PIN_PWM  16   // PWM 출력 (레벨시프터 거쳐 5V 로직)

#define CHANNEL 1
#define FAILSAFE_MS 1000       // 패킷 확인, 해당 시간 동안 없으면 통신이 끊긴 것으로 간주 정지
#define PWM_FREQ    20000     // 20 kHz (필요시 드라이버 사양에 맞춰 조정)
#define PWM_BITS    12        // 12-bit (0..4095)

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
const int LED_PIN = LED_BUILTIN;

// 송신기 주소
uint8_t SENDER_MAC[] = {0xEC,0xE3,0x34,0x1A,0xCF,0xA4};

typedef struct __attribute__((packed)) {
  uint8_t  ver;     // 프로토콜 버전
  uint8_t  flags;   // bit0: enable, bit1: dir
  uint16_t speed;   // 0~1000
  uint16_t seq;     // 시퀀스
} Payload;

volatile bool     got_msg = false;
volatile Payload  last_msg{};
volatile uint32_t last_us  = 0;

void onSend(const wifi_tx_info_t * /*info*/, esp_now_send_status_t status) {
  // Serial.printf("[ACK-SEND] %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// 송신기에게 콜백 (송신기가 보낸 인터페이스를 다시 보냄 예시: enable 바이트로 1을 받으면 송신기에게 enable=1 보내기)
void onRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(Payload)) return;
  Payload p;
  memcpy(&p, incomingData, sizeof(p));

  // volatile에 쓸 때는 memcpy 사용
  memcpy((void*)&last_msg, &p, sizeof(Payload));
  last_us  = (uint32_t)esp_timer_get_time(); // µs
  got_msg  = true;

  digitalWrite(LED_PIN, HIGH);
  // 에코: 고정 송신기 MAC으로 회신 (여러 송신자 지원하려면 info->src_addr 사용)
  esp_now_send(SENDER_MAC, (uint8_t*)&p, sizeof(p));
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_EN, LOW);       // 안전 기본값: 정지
  digitalWrite(PIN_DIR, LOW);

  analogWrite(PIN_PWM, 0);
  //ledcAttach(PIN_PWM, PWM_FREQ, PWM_BITS);
  //ledcWrite(PIN_PWM, 0);

  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  // 같은 채널로 고정
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); while (1) {}
  }
  // 에코용 peer 등록
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, SENDER_MAC, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);

  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onRecv);
}

void loop() {
  // digitalWrite(PIN_EN, HIGH);
  // digitalWrite(PIN_DIR, HIGH);
  // analogWrite(PIN_PWM, 120);
  // delay(3000);

  // analogWrite(PIN_PWM, 0);
  // digitalWrite(PIN_EN, LOW);

  uint32_t now_us = (uint32_t)esp_timer_get_time();
  Payload p;
  uint32_t last_time_us;
  bool have;
  noInterrupts();
  memcpy(&p, (const void*)&last_msg, sizeof(Payload));
  last_time_us = last_us;
  have = got_msg;
  interrupts();

  int enable_bit = 0;
  int dir_bit = 0;
  uint16_t sp = 0;

  if (!have || (now_us - last_time_us) > (FAILSAFE_MS * 1000UL)) {
    //ledcWrite(PIN_PWM, 0);        // PWM OFF
    analogWrite(PIN_PWM, 0);
    digitalWrite(PIN_EN, LOW);   // Enable=LOW → 정지
  } else {
    enable_bit = (p.flags & (1 << 0)) ? 1 : 0; // 0 or 1
    dir_bit    = (p.flags & (1 << 1)) ? 1 : 0; // 0 or 1
    // int enable   = (p.flags & (1 << 0));  // 0 또는 1 판별
    // int dir_cw   = (p.flags & (1 << 1));  // 0 또는 1 판별

    // 송신기로부터 받은 enable 값 판별 0이면 모터 구동 on, 1이면 모터 구동 off => ? if 조건문에서 enable_bit 값이 1이면 모터 구동이고 0이면 모터 구동 off인데? 뭐가 맞는건지?
    if (enable_bit == 1){
      digitalWrite(PIN_EN, HIGH);
      // 회전 방향 판별 0 -> CW, 1 -> CCW
      if (dir_bit == 0){
        digitalWrite(PIN_DIR, LOW);
      } else{
        digitalWrite(PIN_DIR, HIGH);
      }
      sp = p.speed;
      if (sp > 1000) sp = 1000;
      // uint32_t duty = (uint32_t)sp * ((1u << PWM_BITS) - 1u) / 1000u; // 0..4095
      // ledcWrite(PIN_PWM, duty);
      uint8_t duty8 = (uint8_t)((sp * 255UL) / 1000UL);
      analogWrite(PIN_PWM, duty8);
    } else{
      // ledcWrite(PIN_PWM, 0);
      analogWrite(PIN_PWM, 0);
      digitalWrite(PIN_EN, LOW);
    } 
  }
  // 디버깅 메세지
  // Serial.printf("ena=%d, dir=%d, sp=%d\n", enable_bit, dir_bit, sp);

  // 루프 주기(짧게)
  delay(20);
}
