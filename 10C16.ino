#include <Servo.h>
#include <math.h>

// ---------------- 핀 설정 ----------------
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13
#define PIN_SERVO 10

// ---------------- 상수 정의 ----------------
#define SND_VEL 346.0          // sound velocity (m/s)
#define INTERVAL 100            // ms
#define PULSE_DURATION 10       // µs
#define TIMEOUT 30000           // µs
#define SCALE (0.001 * 0.5 * SND_VEL) // duration→distance(mm)

// 차량 감지 기준
#define DETECT_DIST 200.0       // 20cm 이내 감지

// 서보 제어 범위
#define SERVO_MIN 0
#define SERVO_MAX 90            // 차단기 열림 각도

// ---------------- 전역 변수 ----------------
Servo myservo;
unsigned long last_time = 0;
float dist = 400.0;             // 초기 거리
bool car_detected = false;      // 차량 감지 상태

// ---------------- 함수 선언 ----------------
float USS_measure(int TRIG, int ECHO);
void servoMoveSigmoid(int startAngle, int endAngle, int duration_ms);
void servoMoveSmooth(int startAngle, int endAngle, int duration_ms);

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  
  myservo.attach(PIN_SERVO);
  myservo.write(SERVO_MAX); // 시작 시 차단기 열림 상태
  Serial.begin(57600);
}

void loop() {
  if (millis() < last_time + INTERVAL) return;
  last_time = millis();

  dist = USS_measure(PIN_TRIG, PIN_ECHO);

  if (dist > 0 && dist < DETECT_DIST) {  // 차량 접근
    if (!car_detected) {
      car_detected = true;
      digitalWrite(PIN_LED, LOW); // LED 꺼짐
      Serial.println("[Sigmoid] 차량 감지 → 차단기 닫힘");
      servoMoveSigmoid(SERVO_MAX, SERVO_MIN, 2000);  // sigmoid 방식 (내림)
    }
  } 
  else {  // 차량 없음
    if (car_detected) {
      car_detected = false;
      digitalWrite(PIN_LED, HIGH); // LED 켜짐
      Serial.println("[Smooth] 차량 없음 → 차단기 열림");
      servoMoveSmooth(SERVO_MIN, SERVO_MAX, 2000);  // 커스텀 부드러운 상승
    }
  }

  // 거리 출력 (시리얼 플로터용)
  Serial.print("dist:"); Serial.println(dist);
}

// ---------------- 초음파 거리 측정 ----------------
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // mm
}

// ---------------- (1) Sigmoid 함수 기반 부드러운 제어 ----------------
void servoMoveSigmoid(int startAngle, int endAngle, int duration_ms) {
  const int steps = 50;
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    // sigmoid 곡선 (중간부에서 부드럽게 가속/감속)
    float sigmoid = 1.0 / (1.0 + exp(-12.0 * (t - 0.5)));
    int angle = startAngle + (endAngle - startAngle) * sigmoid;
    myservo.write(angle);
    delay(duration_ms / steps);
  }
}

// ---------------- (2) 함수 하나 더 (EaseInOutQuad) ----------------
void servoMoveSmooth(int startAngle, int endAngle, int duration_ms) {
  const int steps = 50;
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / steps;
    // Ease In-Out Quadratic curve
    float smooth = t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
    int angle = startAngle + (endAngle - startAngle) * smooth;
    myservo.write(angle);
    delay(duration_ms / steps);
  }
}
