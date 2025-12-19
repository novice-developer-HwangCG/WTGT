#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/*
컨트롤러 코드

테스트를 위해 수정 필요 사항
1. 내장 LED 외 별도 LED 추가 gpio 14 = 수신기와 연결이 되었을 시 led 2번 깜빡이기 이후 LED 계속 커두기(연결 되었음을 육안으로 확인하기용), 연결 끊기면 3초간격으로 1번씩 점멸
2. 각 pin 수정 PIN_SW1 = 삭제 해당 스위치는 이제 POWER 스위치로 사용(전력 공급, 차단), PIN_SW2 = Enable 그대로 사용 핀 번호는 gpio 35, PIN_SW3 = Direction 그대로 사용 핀 번호는 gpio 34, PIN_SW4 = 삭제, POT_PIN = 가변저항 그대로 사용 핀 번호는 gpio 36
3. 모터 최소 rpm 추가 필요, PWM 0~255 기준으로 와이퍼 돌릴 시 0에서 바로 25로 올려서 시작 (예: 기존 가변 볼륨 저항 돌릴 시 0 ..1..4..9..17 식으로 처음부터 올라간다면 최소 25를 넣을 시 0 .. 25.. 28.. 35.. 이런식으로)
4. 송신기와 수신기 간 메세지 전달 하지 않아도 됨
5. 송신기 코드에 DEBUG bool 변수 추가하여 DEBUG 값이 true이면 debug 메세지 출력, false이면 debug 메세지 출력안함
*/

#define CHANNEL 1
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
const int LED_PIN = LED_BUILTIN;

#define PIN_SW1 22  // 제어 on/off (ACTIVE-LOW)
#define PIN_SW2 21  // 모터 Enable  (ACTIVE-LOW)
#define PIN_SW3 18  // 모터 Dir     (ACTIVE-LOW)
#define PIN_SW4 17  // 비상 정지    (인터럽트) => 현재 문제가 있어서 사용하지 않음 해당 기능 주석 처리
#define POT_PIN 34  // ADC1 핀(가변저항)

// 수신기 MAC 주소
uint8_t RECEIVER_MAC[] = {0xEC,0xE3,0x34,0x1A,0xA5,0x78};

typedef struct __attribute__((packed)) {
  uint8_t  ver;      // 프로토콜 버전
  uint8_t  flags;    // bit0: enable, bit1: dir
  uint16_t speed;    // 0~1000 (가변저항 값 맵핑)
  uint16_t seq;      // 시퀀스 번호
} Payload;

Payload msg;
static uint16_t seq = 0;

void onSend(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // 전송 관련 정보 필요 없음 기능은 살려두고 주석 처리만
  // 간단 확인용 로그:
  // Serial.printf("Send: %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(Payload)) {
    Serial.printf("[RX] %d bytes (unexpected size)\n", len);
    return;
  }
  Payload ack;
  memcpy(&ack, incomingData, sizeof(ack));
  const uint8_t* mac = info->src_addr;
  Serial.printf("[ACK] from %02X:%02X:%02X:%02X:%02X:%02X | seq=%u flags(E/D)=%u/%u speed=%u\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                ack.seq,
                (ack.flags & 0x01) ? 1 : 0,
                (ack.flags & 0x02) ? 1 : 0,
                ack.speed);
  digitalWrite(LED_PIN, HIGH);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);
  pinMode(PIN_SW3, INPUT_PULLUP);
  //pinMode(PIN_SW4, INPUT_PULLUP);

  analogReadResolution(12);            // 0..4095
  analogSetPinAttenuation(POT_PIN, ADC_11db); // 0~3.3V

  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); digitalWrite(LED_PIN, LOW); while (1) {}
  }

  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, RECEIVER_MAC, 6);
  peer.channel = CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;         // v3.x에서 명시

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer failed"); digitalWrite(LED_PIN, LOW); while (1) {}
  }

  esp_now_register_recv_cb(onRecv);

  msg.ver = 1;  // 프로토콜 고정값
}

void loop() {
  static uint8_t  prev_flags = 0xFF;
  static int      prev_raw   = -1;
  static uint32_t last_print = 0;

  bool control   = (digitalRead(PIN_SW1) == LOW);
  bool enable    = (digitalRead(PIN_SW2) == LOW);
  bool dir_cw    = (digitalRead(PIN_SW3) == LOW);
  //bool emergency = (digitalRead(PIN_SW4) == LOW);

  int raw = analogRead(POT_PIN);                 // 0..4095
  if (raw < 0) raw = 0; if (raw > 4095) raw = 4095;
  uint16_t speed = (uint16_t)((raw * 1000UL) / 4095UL);  // 0..1000

  uint8_t enable_bit = (enable) ? 1 : 0;
  uint8_t dir_bit    = (dir_cw) ? 1 : 0;

    // PIN_SW1 판별 control 신호가 LOW 라면 컨트롤러기를 제어 하지 않음
  if (control == LOW){
    delay(100);
    return;  //
  } else{
    msg.flags = (enable_bit<<0) | (dir_bit<<1);
    msg.speed = speed;
    msg.seq   = ++seq;
  }

  // 1초마다 한 번씩 출력 (디버깅용)
  // if ((millis() - last_print) > 1000) {
  //   Serial.printf("[I/O] Control(sw)=%d Enable(sw)=%d Dir(sw)=%d | flags(E/D)=%u/%u | raw=%4d speed=%4u seq=%u\n",
  //                 control, enable, dir_cw,
  //                 enable_bit, dir_bit,
  //                 raw, speed, msg.seq);
  //   last_print = millis();
  // }

  esp_err_t r = esp_now_send(RECEIVER_MAC, (uint8_t*)&msg, sizeof(msg));
  if (r != ESP_OK) Serial.printf("Send err: %d\n", r);

  delay(100);
}
