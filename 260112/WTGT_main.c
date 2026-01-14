#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define PIN_EN   17
#define PIN_DIR  19
#define PIN_PWM  27

#define HIT_1   35
#define HIT_2   34
#define HIT_3   39

#define LED_A   23
#define LED_B   22
#define LED_C   25
#define LED_D   26

#define CHANNEL 1
#define FAILSAFE_MS 1000
#define PWM_FREQ    20000
#define PWM_BITS    8

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
const int PWR_LED = LED_BUILTIN;

uint8_t SENDER_MAC[] = {0x94,0x54,0xC5,0xB5,0xD3,0x00};

typedef struct __attribute__((packed)) {
  uint8_t  ver;
  uint8_t  enable;
  uint8_t  direction;
  uint8_t  pwm;        // 0~255
  uint16_t seq;
} Payload;

volatile bool     got_msg = false;
volatile Payload  last_msg{};
volatile uint64_t last_us = 0;   // 64-bit 권장

// ───── HIT/LED 타이머 설정 ─────
static const uint32_t HIT_BLINK_TOTAL_MS = 3000;  // 3초
static const uint32_t HIT_BLINK_PERIOD_MS = 150; // 점멸 주기(원하는대로 조절)

struct BlinkJob {
  bool active = false;
  uint32_t start_ms = 0;
  uint32_t last_toggle_ms = 0;
  bool level = false;
};

BlinkJob jobA, jobB, jobC;
bool prev_hit1 = false, prev_hit2 = false, prev_hit3 = false;

// LED_D(통신 끊김 1초 주기 점멸)
bool ledD_level = false;
uint32_t ledD_last_toggle_ms = 0;

// ───── ESP-NOW 콜백 ─────
void onSend(const wifi_tx_info_t * /*info*/, esp_now_send_status_t status) {
  (void)status;
}

void onRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  (void)info;
  if (len != sizeof(Payload)) return;

  Payload p;
  memcpy(&p, incomingData, sizeof(p));

  memcpy((void*)&last_msg, &p, sizeof(Payload));
  last_us  = (uint64_t)esp_timer_get_time(); // µs
  got_msg  = true;

  // ACK 에코
  //esp_now_send(SENDER_MAC, (uint8_t*)&p, sizeof(p));
  esp_now_send(info->src_addr, (uint8_t*)&p, sizeof(p));
}

// ───── 유틸: 특정 LED를 3초 점멸 시작 ─────
static inline void startBlink(BlinkJob &job, int ledPin) {
  uint32_t now = millis();
  job.active = true;
  job.start_ms = now;
  job.last_toggle_ms = now;
  job.level = true;                 // 시작은 ON
  digitalWrite(ledPin, HIGH);
}

// ───── 유틸: 점멸 업데이트(논블로킹) ─────
static inline void updateBlink(BlinkJob &job, int ledPin, uint32_t now_ms) {
  if (!job.active) return;

  if (now_ms - job.start_ms >= HIT_BLINK_TOTAL_MS) {
    job.active = false;
    job.level = false;
    digitalWrite(ledPin, LOW);
    return;
  }

  // 3초 동안 LED on 이후 off
  digitalWrite(ledPin, job.level ? HIGH : LOW);
  // if (now_ms - job.last_toggle_ms >= HIT_BLINK_PERIOD_MS) {
  //   job.last_toggle_ms = now_ms;
  //   job.level = !job.level;
  //   digitalWrite(ledPin, job.level ? HIGH : LOW);
  // }
}

// ───── HIT 입력 감지(상승엣지) ─────
static inline void updateHitDetect(uint32_t now_ms) {
  bool h1 = (digitalRead(HIT_1) == HIGH);
  bool h2 = (digitalRead(HIT_2) == HIGH);
  bool h3 = (digitalRead(HIT_3) == HIGH);

  // HIGH가 들어온 순간만 트리거(상승엣지)
  if (h1 && !prev_hit1) startBlink(jobA, LED_A);
  if (h2 && !prev_hit2) startBlink(jobB, LED_B);
  if (h3 && !prev_hit3) startBlink(jobC, LED_C);

  prev_hit1 = h1;
  prev_hit2 = h2;
  prev_hit3 = h3;

  // 각 LED 점멸 상태 업데이트
  updateBlink(jobA, LED_A, now_ms);
  updateBlink(jobB, LED_B, now_ms);
  updateBlink(jobC, LED_C, now_ms);
}

void setup() {
  pinMode(PWR_LED, OUTPUT);
  digitalWrite(PWR_LED, LOW);

  pinMode(LED_A, OUTPUT); 
  digitalWrite(LED_A, LOW);
  
  pinMode(LED_B, OUTPUT); 
  digitalWrite(LED_B, LOW);
  
  pinMode(LED_C, OUTPUT); 
  digitalWrite(LED_C, LOW);
  
  pinMode(LED_D, OUTPUT); 
  digitalWrite(LED_D, LOW);

  pinMode(HIT_1, INPUT);
  pinMode(HIT_2, INPUT);
  pinMode(HIT_3, INPUT);

  pinMode(PIN_EN, OUTPUT); 
  digitalWrite(PIN_EN, LOW);
  
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, LOW);

  // PWM (LEDC)
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_BITS);
  ledcWrite(PIN_PWM, 0);

  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) {}
  }

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
  uint64_t now_us = (uint64_t)esp_timer_get_time();
  uint32_t now_ms = millis();

  // HIT/LED 처리
  updateHitDetect(now_ms);

  // 수신 데이터 스냅샷
  Payload p;
  uint64_t last_time_us;
  bool have;

  noInterrupts();
  memcpy(&p, (const void*)&last_msg, sizeof(Payload));
  last_time_us = last_us;
  have = got_msg;
  interrupts();

  bool comm_ok = have && ((now_us - last_time_us) <= (uint64_t)FAILSAFE_MS * 1000ULL);

  // LED_D: 통신 정상=ON, 통신 끊김=1초 주기 점멸
  if (comm_ok) {
    digitalWrite(LED_D, HIGH);
  } else {
    // led 1초 마다 1회 점멸 (1초 on, 1초 off, 1초 on ...)
    if (now_ms - ledD_last_toggle_ms >= 1000) {
      ledD_last_toggle_ms = now_ms;
      ledD_level = !ledD_level;
      digitalWrite(LED_D, ledD_level ? HIGH : LOW);
    }
  }

  if (!comm_ok) {
    ledcWrite(PIN_PWM, 0);
    digitalWrite(PIN_EN, LOW);
    digitalWrite(PWR_LED, LOW);
  } else {
    bool en   = (p.enable != 0);
    bool dirR = (p.direction != 0);
    uint8_t pwm = p.pwm;

    if (!en || pwm == 0) {
      ledcWrite(PIN_PWM, 0);
      digitalWrite(PIN_EN, LOW);
      digitalWrite(PWR_LED, LOW);
    } else {
      digitalWrite(PIN_EN, HIGH);
      if (dirR) {
        
        digitalWrite(PIN_DIR, HIGH);  // 우/정방향
      } else {
        digitalWrite(PIN_DIR, LOW);   // 좌/역방향
      }
      ledcWrite(PIN_PWM, pwm);  // 0~255
      digitalWrite(PWR_LED, HIGH);
    }
  }

  delay(10);
}
