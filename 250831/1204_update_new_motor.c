#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define PIN_EN   21   // Enable: LOW=정지, HIGH=동작
#define PIN_DIR  18   // Direction: LOW=좌/역방향, HIGH=우/정방향
#define PIN_PWM  16   // PWM 출력 (레벨시프터 거쳐 5V 로직)

#define CHANNEL 1
#define FAILSAFE_MS 1000       // 이 시간 동안 패킷 없으면 정지
#define PWM_FREQ    20000      // 20 kHz
#define PWM_BITS    12         // 12-bit (0..4095)

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
const int LED_PIN = LED_BUILTIN;

// 송신기 MAC 주소 (컨트롤러 보드 MAC으로 맞춰야 함)
uint8_t SENDER_MAC[] = {0xEC,0xE3,0x34,0x1A,0xCF,0xA4};

// ───── 송신기와 맞춘 Payload 구조체 ─────
typedef struct __attribute__((packed)) {
  uint8_t  ver;        // 프로토콜 버전
  uint8_t  enable;     // 0: OFF, 1: ON
  uint8_t  direction;  // 0: LEFT, 1: RIGHT
  uint8_t  pwm;        // 0~255 (0 또는 25 이상)
  uint16_t seq;        // 시퀀스
} Payload;

volatile bool     got_msg = false;
volatile Payload  last_msg{};
volatile uint32_t last_us  = 0;

// ───── ESP-NOW 콜백 ─────
void onSend(const wifi_tx_info_t * /*info*/, esp_now_send_status_t status) {
  // ACK 전송 결과 (원하면 디버깅용으로 사용)
  // Serial.printf("[ACK-SEND] %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(Payload)) return;

  Payload p;
  memcpy(&p, incomingData, sizeof(p));

  // volatile에 쓰기
  memcpy((void*)&last_msg, &p, sizeof(Payload));
  last_us  = (uint32_t)esp_timer_get_time(); // µs
  got_msg  = true;

  digitalWrite(LED_PIN, HIGH);

  // ACK 에코: 송신기 MAC으로 회신
  // (여러 송신기 지원하려면 info->src_addr 사용 가능)
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
  // 필요하면 ledc로 변경 가능
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
    Serial.println("ESP-NOW init failed");
    while (1) {}
  }

  // ACK용 peer 등록 (송신기 MAC)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, SENDER_MAC, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);

  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onRecv);

  Serial.println("Receiver ready");
}

void loop() {
  uint32_t now_us = (uint32_t)esp_timer_get_time();
  Payload p;
  uint32_t last_time_us;
  bool have;

  noInterrupts();
  memcpy(&p, (const void*)&last_msg, sizeof(Payload));
  last_time_us = last_us;
  have = got_msg;
  interrupts();

  bool en    = false;
  bool dirR  = false;    // true: 오른쪽/정방향, false: 왼쪽/역방향
  uint8_t pwm = 0;

  if (!have || (now_us - last_time_us) > (FAILSAFE_MS * 1000UL)) {
    // 타임아웃 → 정지
    analogWrite(PIN_PWM, 0);
    digitalWrite(PIN_EN, LOW);
    digitalWrite(LED_PIN, LOW);
  } else {
    en   = (p.enable != 0);
    dirR = (p.direction != 0);
    pwm  = p.pwm;           // 0~255 (송신기에서 이미 최소값 보정 끝난 값)

    if (!en || pwm == 0) {
      // 정지
      analogWrite(PIN_PWM, 0);
      digitalWrite(PIN_EN, LOW);
      digitalWrite(LED_PIN, LOW);
    } else {
      // Enable ON
      digitalWrite(PIN_EN, HIGH);

      // 방향
      if (dirR) {
        // 우/정방향
        digitalWrite(PIN_DIR, HIGH);
      } else {
        // 좌/역방향
        digitalWrite(PIN_DIR, LOW);
      }

      // PWM 출력
      analogWrite(PIN_PWM, pwm);

      // 수신/동작 표시용 LED
      digitalWrite(LED_PIN, HIGH);
    }
  }

  // 디버깅 필요하면 활성화
  // Serial.printf("en=%d dirR=%d pwm=%u seq=%u\n", en, dirR, pwm, p.seq);

  delay(20);
}
