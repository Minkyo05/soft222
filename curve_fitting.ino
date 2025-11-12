#include <Arduino.h>
#include <stdlib.h>

#define PIN_IR A0

// ==============================
// 유틸: 줄 읽기/입력 파서
// ==============================
static String readLine() {
  while (true) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n'); // CR/LF/CRLF 모두 처리
      s.trim();
      return s; // 빈 문자열도 반환 (기본값 선택에 사용)
    }
  }
}

static long readIntLine(const char* prompt, long defVal, long minVal, long maxVal) {
  while (true) {
    Serial.print(prompt);
    Serial.print(" [기본 ");
    Serial.print(defVal);
    Serial.println("] : ");
    String s = readLine();
    if (s.length() == 0) return defVal;
    char *endp = nullptr;
    long v = strtol(s.c_str(), &endp, 10);
    if (endp && *endp == '\0' && v >= minVal && v <= maxVal) return v;
    Serial.println("→ 정수가 아니거나 범위를 벗어났습니다. 다시 입력해주세요.");
  }
}

static float readFloatLine(const char* prompt, float defVal, float minVal, float maxVal) {
  while (true) {
    Serial.print(prompt);
    Serial.print(" [기본 ");
    Serial.print(defVal, 3);
    Serial.println("] : ");
    String s = readLine();
    if (s.length() == 0) return defVal;
    char *endp = nullptr;
    float v = strtod(s.c_str(), &endp);
    if (endp && *endp == '\0' && v >= minVal && v <= maxVal) return v;
    Serial.println("→ 실수가 아니거나 범위를 벗어났습니다. 다시 입력해주세요.");
  }
}

// 아무 키(엔터 포함) 대기: 라인 엔딩/설정 무관
static void waitForKey(const char* prompt) {
  if (prompt && prompt[0]) {
    Serial.print(prompt);
    Serial.println("  (엔터/아무 키)");
  }
  // 입력 버퍼 비우기
  while (Serial.available()) Serial.read();
  // 최소 1바이트 수신될 때까지 대기
  while (Serial.available() == 0) { /* wait */ }
  delay(5);
  while (Serial.available()) Serial.read();
  Serial.println("OK");
}

// ==============================
// 정렬 비교 (제공 코드 준용)
// ==============================
int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

// ==============================
// 스파이크 제거 필터 (제공 코드 준용)
// ==============================
unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  if ((n == 0) || (n > 100) || (position < 0.0f) || (position > 1.0f))
    return 0;

  if (position == 1.0f) position = 0.999f;

  unsigned int *ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL) return 0;

  if (verbose == 1) {
    Serial.print("n="); Serial.print(n);
    Serial.print(", pos="); Serial.print(position, 3);
    Serial.print(" | IR:");
  }

  for (unsigned int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
    if (verbose == 1) { Serial.print(" "); Serial.print(ir_val[i]); }
  }

  qsort(ir_val, n, sizeof(unsigned int), compare);
  unsigned int ret_val = ir_val[(unsigned int)(n * position)];

  if (verbose == 1) {
    Serial.print(" -> sorted:");
    for (unsigned int i = 0; i < n; i++) { Serial.print(" "); Serial.print(ir_val[i]); }
    Serial.print(" | pick="); Serial.println(ret_val);
  }
  free(ir_val);
  return ret_val;
}

// ==============================
// 선형계 풀이 (가우스 소거, 부분 피벗)
// ==============================
static bool solveLinearSystem(float *A, float *b, float *x, int n) {
  for (int i = 0; i < n; i++) {
    // 부분 피벗
    int piv = i; float maxAbs = fabs(A[i*n + i]);
    for (int r = i+1; r < n; r++) {
      float v = fabs(A[r*n + i]);
      if (v > maxAbs) { maxAbs = v; piv = r; }
    }
    if (maxAbs < 1e-12f) return false; // 특이행렬

    if (piv != i) {
      for (int c = i; c < n; c++) {
        float tmp = A[i*n + c]; A[i*n + c] = A[piv*n + c]; A[piv*n + c] = tmp;
      }
      float tb = b[i]; b[i] = b[piv]; b[piv] = tb;
    }

    float pivVal = A[i*n + i];
    for (int c = i; c < n; c++) A[i*n + c] /= pivVal;
    b[i] /= pivVal;

    for (int r = i+1; r < n; r++) {
      float factor = A[r*n + i]; if (factor == 0) continue;
      for (int c = i; c < n; c++) A[r*n + c] -= factor * A[i*n + c];
      b[r] -= factor * b[i];
    }
  }

  for (int i = n-1; i >= 0; i--) {
    float sum = b[i];
    for (int c = i+1; c < n; c++) sum -= A[i*n + c]*x[c];
    x[i] = sum;
  }
  return true;
}

// ==============================
// 다항식 피팅 (정규방정식, 최소제곱)
// x: 설명변수, y: 목표(거리)
// deg: 차수 (미지수는 deg+1개)
// ==============================
static bool polyfit(const float *x, const float *y, int N, int deg, float *coef) {
  const int M = deg + 1;
  float *A = (float*)malloc(M * M * sizeof(float));
  float *B = (float*)malloc(M * sizeof(float));
  if (!A || !B) { if(A)free(A); if(B)free(B); return false; }

  // Σ x^k 누적합
  float S[24]; // deg<=10 가정 시 2*deg<=20, 여기선 deg<=5 권장
  int maxK = 2*deg;
  for (int k=0; k<=maxK; k++) S[k] = 0.0f;
  for (int i=0; i<N; i++) {
    float p = 1.0f;
    for (int k=0; k<=maxK; k++) { S[k] += p; p *= x[i]; }
  }

  // 우변 T_m = Σ y*x^m
  for (int m=0; m<M; m++) B[m] = 0.0f;
  for (int i=0; i<N; i++) {
    float p = 1.0f;
    for (int m=0; m<M; m++) { B[m] += y[i] * p; p *= x[i]; }
  }

  // 정규방정식 A(j,k) = Σ x^(j+k)
  for (int j=0; j<M; j++)
    for (int k=0; k<M; k++)
      A[j*M + k] = S[j+k];

  bool ok = solveLinearSystem(A, B, coef, M);
  free(A); free(B);
  return ok;
}

// ==============================
// 코드 출력 (모델별)
// ==============================
static void printPolynomialAsCode(const float *c, int deg) {
  Serial.println("float volt_to_distance(unsigned int x) {");
  Serial.print("  return ");
  for (int i=0; i<=deg; i++) {
    float a = c[i];
    if (i==0) {
      Serial.print(a, 9);
    } else {
      Serial.print((a >= 0.0f) ? " + " : " - ");
      Serial.print(fabs(a), 9);
      Serial.print("*");
      if (i==1) {
        Serial.print("x");
      } else {
        Serial.print("x");
        for (int k=1; k<i; k++) Serial.print("*x");
      }
    }
  }
  Serial.println(";");
  Serial.println("}");
}

static void printInvPolynomialAsCode(const float *c, int deg) {
  Serial.println("float volt_to_distance(unsigned int x) {");
  Serial.print("  return ");
  for (int i=0; i<=deg; i++) {
    float a = c[i];
    if (i==0) {
      Serial.print(a, 9);
    } else {
      Serial.print((a >= 0.0f) ? " + " : " - ");
      Serial.print(fabs(a), 9);
      Serial.print("*(");
      Serial.print("1.0/(");
      Serial.print("x");
      for (int k=1; k<i; k++) Serial.print("*x");
      Serial.print(")");
      Serial.print(")");
    }
  }
  Serial.println(";");
  Serial.println("}");
}

// ==============================
// 설정/데이터
// ==============================
enum Model { MODEL_POLY=1, MODEL_INV=2 };

const int   K = 7; // 0..30 cm, 5 cm step
float distances[K] = {0, 5, 10, 15, 20, 25, 30};

// ==============================
// 진입
// ==============================
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(100);

  Serial.println();
  Serial.println("=== IR Curve Fitting (모델/차수 시리얼 입력) ===");
  Serial.println("측정: 0,5,10,15,20,25,30 cm 순서로 배치 후 엔터/아무 키");
  Serial.println();

  // ---- 설정 입력 ----
  Serial.println("[모델 선택] 1=POLY (distance=f(x)), 2=INV (distance=f(1/x))");
  long modelSel = readIntLine("모델 번호를 입력하세요 (1/2)", 2, 1, 2);
  Model model = (modelSel == 2) ? MODEL_INV : MODEL_POLY;

  long deg = readIntLine("차수(degree)를 입력하세요 (1~5 권장 1~3)", 2, 1, 5);
  long nRep = readIntLine("필터 반복 측정 횟수 n (1~100)", 7, 1, 100);
  float pct = readFloatLine("퍼센타일 (0.0~1.0, 중앙값=0.5)", 0.5f, 0.0f, 1.0f);
  long verbose = readIntLine("필터 verbose(0,1)", 0, 0, 1);

  Serial.println();
  Serial.print("선택된 모델: ");
  Serial.println(model == MODEL_INV ? "INV (distance=f(1/x))" : "POLY (distance=f(x))");
  Serial.print("차수: "); Serial.println(deg);
  Serial.print("필터 n="); Serial.print(nRep);
  Serial.print(", percentile="); Serial.print(pct, 3);
  Serial.print(", verbose="); Serial.println(verbose);
  Serial.println();

  // ---- 데이터 수집 ----
  float x_raw[K];
  for (int i=0; i<K; i++) {
    char msg[64];
    snprintf(msg, sizeof(msg), "[%d/%d] %d cm 위치로 옮긴 뒤 엔터/키 입력",
             i+1, K, (int)distances[i]);
    waitForKey(msg);

    unsigned int filtered = ir_sensor_filtered((unsigned int)nRep, pct, (int)verbose);
    x_raw[i] = (float)filtered;
    Serial.print("ADC(filtered) = "); Serial.println(filtered);
  }

  Serial.println();
  Serial.println("=== 수집 데이터 (distance[cm] vs ADC) ===");
  for (int i=0; i<K; i++) {
    Serial.print("d="); Serial.print(distances[i], 3);
    Serial.print(" cm, x="); Serial.println(x_raw[i], 3);
  }

  // ---- 피팅 준비 ----
  if (deg >= K) {
    deg = K - 1;
    Serial.print("경고: 차수를 "); Serial.print(deg);
    Serial.println(" 로 낮춥니다(샘플 수 - 1 제한).");
  }

  float z[K]; // 설명변수
  if (model == MODEL_INV) {
    for (int i=0; i<K; i++) {
      z[i] = (x_raw[i] <= 0.0f) ? 1e9f : (1.0f / x_raw[i]); // 0 보호
    }
  } else {
    for (int i=0; i<K; i++) {
      z[i] = x_raw[i];
    }
  }

  // ---- 최소제곱 피팅 ----
  float coef[12]; // deg<=5면 충분
  bool ok = polyfit(z, distances, K, (int)deg, coef);

  Serial.println();
  Serial.print("=== 곡선맞춤 결과 (");
  Serial.print(model == MODEL_INV ? "INV" : "POLY");
  Serial.print(", degree=");
  Serial.print(deg);
  Serial.println(") ===");

  if (!ok) {
    Serial.println("오류: 선형계 해결 실패(데이터가 거의 동일/0 포함/특이행렬).");
    Serial.println("→ 배선/전원/거리 순서/센서 출력 변화를 확인 후 다시 시도하세요.");
    return;
  }

  // 사람 친화적 표기
  if (model == MODEL_POLY) {
    Serial.print("distance ≈ ");
    for (int i=0; i<=deg; i++) {
      if (i==0) {
        Serial.print(coef[i], 9);
      } else {
        Serial.print((coef[i]>=0.0f) ? " + " : " - ");
        Serial.print(fabs(coef[i]), 9);
        Serial.print("*x");
        if (i>=2) for (int k=1; k<i; k++) Serial.print("*x");
      }
    }
    Serial.println();
  } else {
    Serial.print("distance ≈ ");
    for (int i=0; i<=deg; i++) {
      if (i==0) {
        Serial.print(coef[i], 9);
      } else {
        Serial.print((coef[i]>=0.0f) ? " + " : " - ");
        Serial.print(fabs(coef[i]), 9);
        Serial.print("*(1/x");
        if (i>=2) for (int k=1; k<i; k++) Serial.print("*x");
        Serial.print(")");
      }
    }
    Serial.println();
  }

  // ---- 아두이노용 함수 출력 ----
  Serial.println();
  Serial.println("=== 아두이노에 바로 붙여넣어 쓸 수 있는 함수 ===");
  if (model == MODEL_POLY) printPolynomialAsCode(coef, (int)deg);
  else                     printInvPolynomialAsCode(coef, (int)deg);

  Serial.println();
  Serial.println("사용 예: distance_cm = volt_to_distance(filtered_adc);");
  Serial.println("끝.");
}

void loop() {
  delay(1000);
}
