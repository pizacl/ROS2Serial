#include "packet.h"
#include "Arduino.h"

IntervalTimer timer;

void packet_recv_cb(const int id, const int len, const uint8_t* data){
  // 受信したときの処理
}

void timer_callback() {
  packet_send();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  packet_begin();
  digitalWrite(LED_BUILTIN, HIGH);
  timer.begin(timer_callback, 10 * 1000); // 10ms interval
}

void loop() {
  packet_update();
  static unsigned long led_ts = 0;
  static bool led = false;
  unsigned long now_ts = millis();
  if((now_ts - led_ts) >= LED_WAIT_MS){
    led_ts = now_ts; 
    led = !led;
    if((now_ts - packet_recv_ts) < LED_WAIT_MS){
      digitalWrite(LED_BUILTIN, led);
    }else{
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }
  // Serial.println("send");
}