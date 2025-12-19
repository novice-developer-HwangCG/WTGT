#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/*
  ────────────────────────────────────────────────
  컨트롤러(송신기) 코드 - 단순 테스트용

  1) 내장 LED + GPIO 14 외부 LED 사용
     - GPIO 14 : 수신기와 "연결"되면 2번 깜빡인 후 계속 ON
     - 연결 끊기면 3초 간격으로 1번씩 짧게 점멸

  2) 핀 변경
     - PIN_SW1 : 삭제
     - PIN_SW2 : Enable 스위치, GPIO 35 (ACTIVE-LOW)
     - PIN_SW3 : Direction 스위치, GPIO 34 (ACTIVE-LOW)
     - PIN_SW4 : 삭제
     - POT_PIN : 가변저항, GPIO 36 (ADC1)

  3) 모터 최소 RPM 개념 (PWM 0~255 기준)
     - 가변저항 → PWM 0~255 로 매핑
     - PWM 값이 0이면 그대로 0
     - PWM 값이 1~24 사이면 강제로 25로 올림
       (예: 0, 25, 28, 35... 이런 식으로 올라감)

  4) 송신기와 수신기 간의 "데이터 내용"은 크게 신경 안 씀
     - 단순 heartbeat 프레임만 전송
     - 수신기에서 이 값을 실제로 쓰지 않아도 됨

  5) DEBUG 플래그
     - DEBUG == true  → 시리얼 로그 출력
     - DEBUG == false → 시리얼 로그 출력 안 함
  ────────────────────────────────────────────────
*/

// ====== 설정값 =================================================
#define CHANNEL        1                 // ESP-NOW 채널
#ifndef LED_BUILTIN
#define LED_BUILTIN    2                 // 보드 내장 LED
#endif
const int LED_PIN       = LED_BUILTIN;   // 내장 LED
const int LINK_LED_PIN  = 14;           // 외부 링크 상태 LED

// 스위치/가변저항 핀 (ESP32 - GPIO34/35/36은 입력 전용)
#define PIN_SW2  35   // Enable (ACTIVE-LOW)
#define PIN_SW3  34   // Direction (ACTIVE-LOW)
#define POT_PIN  36   // 가변저항 (ADC1)

// 디버그 출력 여부
const bool DEBUG = false;

// 수신기 MAC 주소 (필요시 변경)
uint8_t RECEIVER_MAC[] = {0xEC,0xE3,0x34,0x1A,0xA5,0x78};

// 디버그 매크로
#define DBG_PRINT(...)   do { if (DEBUG) Serial.printf(__VA_ARGS__); } while (0)

// ====== 전송용 간단 Payload 구조체 =============================
//  - 실제로 수신기에서 안 써도 되지만 형식 맞춰 heartbeat만 보냄
typedef struct __attribute__((packed)) {
  uint8_t  ver;      // 프로토콜 버전 (고정 1)
  uint8_t  enable;   // 0 or 1
  uint8_t  direction;   // 0 or 1 (1: 정방향, 0: 역방향)
  uint8_t  pwm;      // 0~255 (0은 정지, 1~24 → 25로 보정)
  uint16_t seq;      // 시퀀스 번호
} Payload;

Payload msg;
static uint16_t seq = 0;

// ====== 링크 상태 관리용 전역 변수 ============================
// 마지막으로 ACK를 받은 시각
volatile uint32_t g_lastAckMillis = 0;

// ====== ESP-NOW 콜백 ==========================================
void onSend(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;  // info 안 쓰면 경고 나올 수 있어서 무시 처리

  // 전송 성공/실패 여부만 간단 확인
  //DBG_PRINT("[TX] status = %s\n", status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

void onRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  g_lastAckMillis = millis();

  const uint8_t *mac = info->src_addr;  // 보낸 쪽 MAC 주소

  // DBG_PRINT("[RX] %d bytes ACK from %02X:%02X:%02X:%02X:%02X:%02X\n",
  //           len,
  //           mac[0], mac[1], mac[2],
  //           mac[3], mac[4], mac[5]);
}

// ====== 링크 LED 상태 업데이트 함수 ============================
//  - 연결됨 : 2번 깜빡 → 계속 ON
//  - 연결끊김 : 3초마다 1번 짧게 깜빡
void updateLinkLed() {
  static bool     prevLinked            = false;
  static bool     firstConnectBlinkDone = false;
  static uint32_t blinkStartMillis      = 0;
  static uint32_t slowBlinkStartMillis  = 0;

  uint32_t now = millis();

  // 최근 2초 이내에 ACK를 받은 적이 있으면 "연결됨"으로 간주
  bool nowLinked = (g_lastAckMillis != 0) && ((now - g_lastAckMillis) < 2000);

  if (nowLinked) {
    // 방금 연결된 순간: 2번 깜빡이기 시작
    if (!prevLinked) {
      firstConnectBlinkDone = false;
      blinkStartMillis = now;
    }

    if (!firstConnectBlinkDone) {
      // 0~200ms : ON
      // 200~400ms: OFF
      // 400~600ms: ON
      // 600~800ms: OFF
      // 이후 : 항상 ON
      uint32_t t = now - blinkStartMillis;
      if (t < 200) {
        digitalWrite(LINK_LED_PIN, HIGH);
      } else if (t < 400) {
        digitalWrite(LINK_LED_PIN, LOW);
      } else if (t < 600) {
        digitalWrite(LINK_LED_PIN, HIGH);
      } else if (t < 800) {
        digitalWrite(LINK_LED_PIN, LOW);
      } else {
        digitalWrite(LINK_LED_PIN, HIGH);
        firstConnectBlinkDone = true;
      }
    } else {
      // 이미 2번 깜빡임 완료 → 계속 켜두기
      digitalWrite(LINK_LED_PIN, HIGH);
    }

  } else {
    // 연결이 안 되어 있을 때: 3초마다 1번 짧게 깜빡
    uint32_t elapsed = now - slowBlinkStartMillis;

    if (elapsed >= 3000) {
      // 새로운 깜빡임 시작
      slowBlinkStartMillis = now;
      digitalWrite(LINK_LED_PIN, HIGH);   // 0~200ms 켜짐
    } else if (elapsed >= 200) {
      digitalWrite(LINK_LED_PIN, LOW);    // 이후 꺼짐
    }

    // 연결 끊긴 상태에서는 다시 연결되면 또 2번 깜빡이도록 플래그 리셋
    firstConnectBlinkDone = false;
  }

  prevLinked = nowLinked;
}

// ====== 설정 함수 ==============================================
void setup() {
  // 내장 LED: 전원 확인용 (항상 ON)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // 링크 표시용 외부 LED (GPIO 14)
  pinMode(LINK_LED_PIN, OUTPUT);
  digitalWrite(LINK_LED_PIN, LOW);

  // 스위치 입력 (GPIO35/34는 내부 풀업 없음 인지할 것)
  pinMode(PIN_SW2, INPUT);   // Enable
  pinMode(PIN_SW3, INPUT);   // Direction

  // 가변저항 ADC 설정
  analogReadResolution(12);                 // 0..4095
  analogSetPinAttenuation(POT_PIN, ADC_11db); // 0~3.3V

  if (DEBUG) {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[SETUP] Controller start");
  }

  // ESP-NOW 초기화 (단순 heartbeat용)
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    if (DEBUG) Serial.println("ESP-NOW init failed");
    // 실패 시 내장 LED를 꺼두고 멈춤
    digitalWrite(LED_PIN, LOW);
    while (1) { delay(1000); }
  }

  esp_now_register_send_cb(onSend);
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer {};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    if (DEBUG) Serial.println("Add peer failed");
    digitalWrite(LED_PIN, LOW);
    while (1) { delay(1000); }
  }

  // Payload 기본값
  msg.ver    = 1;
  msg.enable = 0;
  msg.direction = 0;
  msg.pwm    = 0;
  msg.seq    = 0;

  if (DEBUG) Serial.println("[SETUP] ESP-NOW ready");
}

// ====== 메인 루프 ==============================================
void loop() {
  // 1) 스위치 상태 읽기
  int enable  = digitalRead(PIN_SW2);   // LOW=OFF, HIGH=ON
  int direction = digitalRead(PIN_SW3);   // LOW=LEFT, HIGH=RIGHT

  // 2) 가변저항 → PWM 값 (0~255)
  int raw = analogRead(POT_PIN);   // 0..4095
  if (raw < 0)    raw = 0;
  if (raw > 4095) raw = 4095;

  int pwm = map(raw, 0, 4095, 0, 255);   // 0~255

  // 3) 최소 PWM 보정
  //    - enable OFF 이면 무조건 0
  //    - enable ON 이고 pwm 1~100 → 100로 올림
  if (enable == 0) {
    pwm = 0;
  } else {
    if (pwm > 0 && pwm < 100) {
      pwm = 100;
    }
  }

  // 4) Payload 채우기
  msg.enable = enable ? 1 : 0;
  msg.direction = direction ? 1 : 0;
  msg.pwm    = (uint8_t)pwm;
  msg.seq    = ++seq;

  // 5) 디버그 출력 (필요할 때만)
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (DEBUG && (now - lastPrint) >= 1000) {
    Serial.printf("[I/O] EN=%d DIR=%d raw=%4d pwm=%3d seq=%u\n",
                  enable, direction, raw, pwm, msg.seq);
    lastPrint = now;
  }

  // 6) ESP-NOW 전송 (heartbeat용)
  esp_err_t r = esp_now_send(RECEIVER_MAC, (uint8_t*)&msg, sizeof(msg));
  if (DEBUG && r != ESP_OK) {
    Serial.printf("[ERR] esp_now_send = %d\n", r);
  }

  // 7) 링크 LED 상태 갱신
  updateLinkLed();

  // 8) 루프 주기
  delay(100);  // 100ms 주기 (10Hz)
}
