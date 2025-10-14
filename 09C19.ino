// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define SAMPLE_SIZE 30    // 중위수 필터용 샘플 개수 (3, 10, 30 중 하나로 변경 가능)

unsigned long last_sampling_time;   // unit: msec
float samples[SAMPLE_SIZE];         // 최근 측정값 저장 배열
int sample_index = 0;               // 현재 배열 인덱스
bool buffer_full = false;           // 샘플 버퍼가 꽉 찼는지 여부

// ====== 함수 선언 ======
float USS_measure(int TRIG, int ECHO);
float getMedian();

// ====== setup() ======
void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(57600);
  last_sampling_time = millis();
}

// ====== loop() ======
void loop() {
  if (millis() < last_sampling_time + INTERVAL)
    return;

  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 범위 필터 제거 (직전 유효값 적용 금지)
  // 측정값을 버퍼에 저장
  samples[sample_index] = dist_raw;
  sample_index = (sample_index + 1) % SAMPLE_SIZE;
  if (sample_index == 0) buffer_full = true;

  // 중위수 계산
  float dist_median = getMedian();

  // 출력 형식: 시리얼 플로터용
  Serial.print("Min:");
  Serial.print(_DIST_MIN);
  Serial.print(",raw:");
  Serial.print(dist_raw);
  Serial.print(",median:");
  Serial.print(dist_median);
  Serial.print(",Max:");
  Serial.print(_DIST_MAX);
  Serial.println("");

  // LED 동작 (범위 안이면 ON, 벗어나면 OFF)
  if ((dist_median >= _DIST_MIN) && (dist_median <= _DIST_MAX))
    digitalWrite(PIN_LED, LOW);   // ON
  else
    digitalWrite(PIN_LED, HIGH);  // OFF

  last_sampling_time = millis();
}

// ====== 초음파 거리 측정 함수 ======
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // 단위: mm
}

// ====== 중위수 계산 함수 ======
float getMedian() {
  int count = buffer_full ? SAMPLE_SIZE : sample_index;
  if (count == 0) return 0;

  float temp[SAMPLE_SIZE];
  for (int i = 0; i < count; i++) temp[i] = samples[i];

  // 간단한 버블 정렬
  for (int i = 0; i < count - 1; i++) {
    for (int j = i + 1; j < count; j++) {
      if (temp[i] > temp[j]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  // 중앙값 반환
  if (count % 2 == 1)
    return temp[count / 2];
  else
    return (temp[count / 2 - 1] + temp[count / 2]) / 2.0;
}
