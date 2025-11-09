#include <Servo.h>

// Arduino pin assignment
#define PIN_IR    A0       // IR sensor at Pin A0
#define PIN_LED   9
#define PIN_SERVO 10

#define _DUTY_MIN 540 // servo full clock-wise position (0 degree) 
#define _DUTY_NEU 1000 // servo neutral position (90 degree) 
#define _DUTY_MAX 2390 // servo full counter-clockwise position (180 degree)

#define _DIST_MIN 100.0 // minimum distance 100mm
#define _DIST_MAX 250.0 // maximum distance 250mm

#define EMA_ALPHA  0.5     // for EMA filter
#define LOOP_INTERVAL 20   // Loop Interval (unit: msec)

Servo myservo;
unsigned long last_loop_time = 0;

float dist_prev = _DIST_MIN;
float dist_ema = _DIST_MIN;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds((_DUTY_MIN + _DUTY_MAX) / 2);
  Serial.begin(1000000);
}

void loop() {
  unsigned long time_curr = millis();
  if (time_curr < last_loop_time + LOOP_INTERVAL) return;
  last_loop_time = time_curr;

  int a_value = analogRead(PIN_IR);
  float dist_raw = ((6762.0 / (a_value - 9.0)) - 4.0) * 10.0 - 60.0;

  float dist_filtered;

  if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX) {
    dist_filtered = dist_raw;
    digitalWrite(PIN_LED, HIGH);
  } else {
    dist_filtered = dist_prev;
    digitalWrite(PIN_LED, LOW);
  }
  dist_prev = dist_filtered;

  dist_ema = EMA_ALPHA * dist_filtered + (1.0 - EMA_ALPHA) * dist_ema;

  float ratio = (dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN);
  if (ratio < 0.0) ratio = 0.0;
  if (ratio > 1.0) ratio = 1.0;
  int duty = _DUTY_MIN + (int)((_DUTY_MAX - _DUTY_MIN) * ratio);

  myservo.writeMicroseconds(duty);

  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print(",_DIST_MIN:"); Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println("");
}
