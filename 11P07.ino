#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9        // LED
#define PIN_TRIG 12       // sonar TRIGGER
#define PIN_ECHO 13       // sonar ECHO
#define PIN_SERVO 10      // servo motor

// Sonar config
#define SND_VEL 346.0     // sound velocity (m/s)
#define INTERVAL 25        // sampling interval (ms)
#define PULSE_DURATION 10  // trigger pulse duration (µs)
#define _DIST_MIN 180.0    // 18 cm
#define _DIST_MAX 360.0    // 36 cm
#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL) // duration→distance(mm)
#define _EMA_ALPHA 0.3     // EMA alpha

// Servo duty settings
#define _DUTY_MIN 1000     // 0°
#define _DUTY_MAX 2000     // 180°

// global variables
float dist_prev = _DIST_MAX;
float dist_ema = _DIST_MAX;
unsigned long last_sampling_time = 0;
Servo myservo;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO);
  myservo.write(90); // 초기 중립
  Serial.begin(57600);
}

void loop() {
  if (millis() < last_sampling_time + INTERVAL) return;

  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  float dist_filtered;

  // ----------------- 범위 필터 -----------------
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX) || (dist_raw < _DIST_MIN)) {
    dist_filtered = dist_prev;
    digitalWrite(PIN_LED, HIGH);   // 범위 밖 → LED 꺼짐 (active-low)
  } else {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, LOW);    // 범위 안 → LED 켜짐 (active-low)
  }

  // ----------------- EMA 필터 -----------------
  dist_ema = _EMA_ALPHA * dist_filtered + (1 - _EMA_ALPHA) * dist_ema;

  // ----------------- 거리→서보 각도 변환 -----------------
  int angle;
  if (dist_ema <= _DIST_MIN) angle = 0;
  else if (dist_ema >= _DIST_MAX) angle = 180;
  else {
    // 18cm~36cm → 0~180° 비례
    angle = map((int)dist_ema, (int)_DIST_MIN, (int)_DIST_MAX, 0, 180);
  }
  myservo.write(angle);

  // ----------------- 시리얼 플로터 출력 -----------------
  Serial.print("Min:");  Serial.print(_DIST_MIN);
  Serial.print(",dist:"); Serial.print(dist_raw);
  Serial.print(",ema:"); Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(angle);
  Serial.print(",Max:");  Serial.print(_DIST_MAX);
  Serial.println("");

  last_sampling_time = millis();
}

// ----------------- 초음파 거리 측정 함수 -----------------
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // mm 단위
}
