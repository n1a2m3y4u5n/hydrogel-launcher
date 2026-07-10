#include <Servo.h>

// ─── 서보 ────────────────────────────────
const int SERVO_PIN = 8;
Servo myServo;
int currentAngle = 90;
String inputBuffer = "";

// ─── 색센서 핀 ───────────────────────────
const int S0_PIN  = 3;
const int S1_PIN  = 4;
const int S2_PIN  = 5;
const int S3_PIN  = 6;
const int OUT_PIN = 2;
const int LED_PIN = 7;

// ─── 색센서 타이머 ────────────────────────
unsigned long lastColorRead = 0;
const unsigned long COLOR_INTERVAL = 200;

// ─── 함수 전방 선언 ──────────────────────
void processInput(String input);
void runAndReturn(String angleStr);      // run 명령어
void moveServoSmooth(int from, int to);
void moveServoFast(int from, int to);    // 빠른 복귀용
bool isNumericString(String s);
void readAndPrintColor();
int  readColorChannel(int s2Val, int s3Val);

// ─────────────────────────────────────────
void setup() {
  myServo.attach(SERVO_PIN);
  myServo.write(currentAngle);

  pinMode(S0_PIN,  OUTPUT);
  pinMode(S1_PIN,  OUTPUT);
  pinMode(S2_PIN,  OUTPUT);
  pinMode(S3_PIN,  OUTPUT);
  pinMode(OUT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(S0_PIN, HIGH);  // 주파수 스케일링 20% 고정
  digitalWrite(S1_PIN, LOW);
  digitalWrite(LED_PIN, HIGH); // 보조 LED 켜기

  Serial.begin(9600);
  Serial.println("=== 서보 + 색센서 통합 시스템 ===");
  Serial.println("[서보]  +각도/-각도  →  각도 이동");
  Serial.println("[서보]  run +각도    →  이동 후 즉시 복귀");
  Serial.println("[색센서] 0.5초마다 자동 측정 (낮을수록 해당 색 강함)");
  Serial.print  ("[서보]  초기 위치: ");
  Serial.print  (currentAngle);
  Serial.println("도");
  Serial.println("─────────────────────────────────");
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processInput(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }

  if (millis() - lastColorRead >= COLOR_INTERVAL) {
    lastColorRead = millis();
    readAndPrintColor();
  }
}

// ─── 입력 분기 ───────────────────────────
void processInput(String input) {
  input.trim();

  // "run" 명령어 감지 ("run+45", "run +45", "run-30" 모두 처리)
  if (input.length() >= 3 && input.substring(0, 3) == "run") {
    String angleStr = input.substring(3);
    angleStr.trim();
    runAndReturn(angleStr);
    return;
  }

  // 기존 서보 이동 처리
  if (input.charAt(0) != '+' && input.charAt(0) != '-') {
    Serial.println("[오류] '+'/'-' 기호로 시작하거나 'run +각도' 형식으로 입력하세요.");
    return;
  }
  String numStr = input.substring(1);
  if (numStr.length() == 0 || !isNumericString(numStr)) {
    Serial.println("[오류] 올바른 숫자를 입력하세요. 예: +30, -90");
    return;
  }
  int delta = numStr.toInt();
  if (delta <= 0) {
    Serial.println("[오류] 0보다 큰 각도를 입력하세요.");
    return;
  }
  int targetAngle = (input.charAt(0) == '+')
                    ? currentAngle + delta
                    : currentAngle - delta;
  if (targetAngle > 180) {
    Serial.print("[경고] 범위 초과 -> 180도로 제한 (요청: ");
    Serial.print(targetAngle); Serial.println("도)");
    targetAngle = 180;
  } else if (targetAngle < 0) {
    Serial.print("[경고] 범위 초과 -> 0도로 제한 (요청: ");
    Serial.print(targetAngle); Serial.println("도)");
    targetAngle = 0;
  }
  Serial.print("[서보] 이동 중: ");
  Serial.print(currentAngle); Serial.print("도 -> ");
  Serial.print(targetAngle);  Serial.println("도");
  moveServoSmooth(currentAngle, targetAngle);
  currentAngle = targetAngle;
  Serial.print("[서보] 현재 위치: ");
  Serial.print(currentAngle); Serial.println("도");
}

// ─── run 명령어: 이동 후 즉시 복귀 ───────
void runAndReturn(String angleStr) {
  if (angleStr.charAt(0) != '+' && angleStr.charAt(0) != '-') {
    Serial.println("[오류] 예: run +45, run -30");
    return;
  }
  String numStr = angleStr.substring(1);
  if (numStr.length() == 0 || !isNumericString(numStr)) {
    Serial.println("[오류] 올바른 숫자 입력. 예: run +45");
    return;
  }
  int delta = numStr.toInt();
  if (delta <= 0) {
    Serial.println("[오류] 0보다 큰 각도를 입력하세요.");
    return;
  }

  int savedAngle = currentAngle;
  int targetAngle = (angleStr.charAt(0) == '+')
                    ? currentAngle + delta
                    : currentAngle - delta;
  targetAngle = constrain(targetAngle, 0, 180);

  Serial.print("[run] ");
  Serial.print(savedAngle);  Serial.print("도 -> ");
  Serial.print(targetAngle); Serial.print("도  →  ");
  Serial.print(savedAngle);  Serial.println("도 복귀");

  // 목표로 부드럽게 이동
  moveServoSmooth(savedAngle, targetAngle);
  currentAngle = targetAngle;

  // 즉시 최대 속도로 복귀
  moveServoFast(targetAngle, savedAngle);
  currentAngle = savedAngle;

  Serial.print("[run] 복귀 완료: ");
  Serial.print(currentAngle); Serial.println("도");
}

// ─── 서보 이동 함수 ──────────────────────
void moveServoSmooth(int from, int to) {
  // 10ms/도 간격으로 부드럽게
  int step = (to > from) ? 1 : -1;
  for (int angle = from; angle != to; angle += step) {
    myServo.write(angle);
    delay(10);
  }
  myServo.write(to);
}

void moveServoFast(int from, int to) {
  // 즉시 복귀: 서보 최대 속도 (약 6ms/도)
  myServo.write(to);
  delay(abs(to - from) * 6);
}

// ─── 색센서 함수 ─────────────────────────
void readAndPrintColor() {
  int r = readColorChannel(LOW,  LOW );
  int g = readColorChannel(HIGH, HIGH);
  int b = readColorChannel(LOW,  HIGH);
  Serial.print("[색센서] R:"); Serial.print(r);
  Serial.print("us  G:");      Serial.print(g);
  Serial.print("us  B:");      Serial.print(b);
  Serial.println("us");
}

int readColorChannel(int s2Val, int s3Val) {
  digitalWrite(S2_PIN, s2Val);
  digitalWrite(S3_PIN, s3Val);
  delay(30);
  return pulseIn(OUT_PIN, LOW, 50000);
}

// ─────────────────────────────────────────
bool isNumericString(String s) {
  for (int i = 0; i < s.length(); i++) {
    if (!isDigit(s.charAt(i))) return false;
  }
  return true;
}