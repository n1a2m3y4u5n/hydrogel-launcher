/**
 * Bio-hydrogel 발사 장치 - 릴레이 버전 v6 (1:1 비율 피드백 제어)
 *
 * ── 하드웨어 구조 ──
 *   [왼쪽 탱크] → 왼쪽 펌프(핀4) → 왼쪽 유량계(핀3) → ─┐
 *                                                          Y합류 → [저장 탱크] → 앞쪽 펌프(핀6) → 발사
 *   [오른쪽 탱크] → 오른쪽 펌프(핀5) → 오른쪽 유량계(핀2) → ─┘
 *
 * ── v6 변경사항 ──
 *   ★ 1:1 비율 피드백 제어 추가 (controlLoop 내 온오프 방식)
 *     - 200ms마다 totalVolLeft / totalVolRight 비율 체크
 *     - 비율 > 1.05 → 왼쪽 펌프 일시 정지 (오른쪽만 가동)
 *     - 비율 < 0.95 → 오른쪽 펌프 일시 정지 (왼쪽만 가동)
 *     - 0.95 ~ 1.05 → 양쪽 정상 가동
 *     - 초기 10mL 미만 구간은 비율 체크 제외 (노이즈 방지)
 *   ★ 유량 로그에 실시간 비율(ratio) 컬럼 추가
 *   ★ status 출력에 현재 비율 표시 추가
 *   ★ RATIO_TOLERANCE 상수로 허용 오차 한 줄 조정 가능
 *
 * ── ★중요: 릴레이 폴리티 검증 먼저★ ──
 *   증상(stop 눌러도 릴레이 안 꺼짐 / 부팅 시 켜짐)은 보통 "액티브 LOW 모듈"인데
 *   코드를 액티브 HIGH로 가정했을 때 나타난다.
 *   → 펌프 부하 떼고(릴레이 딸깍 소리만 확인) 시리얼에 'test' 입력.
 *     "ON이라고 말할 때 딸깍 켜지면" 현재 RELAY_ACTIVE_HIGH 설정이 맞는 것.
 *     반대면 아래 RELAY_ACTIVE_HIGH 를 false 로 바꿔서 재업로드.
 *
 *   부팅 시 자동 ON은 부트로더 구간(setup 이전)이라 코드로 못 막음 → 하드웨어 처리:
 *     - 액티브 HIGH 모듈: 핀→GND 10k 풀다운
 *     - 액티브 LOW  모듈: 핀→5V  10k 풀업   (※ 인계 처방과 반대 주의)
 *
 * ── 실험 설정 (6월 1차) ──
 *   알긴산 탱크: 알긴산 나트륨 1.0 wt% + 피트산 0.5 wt%
 *   CaCl2  탱크: CaCl2 2.0 wt%   |   부피 혼합비 1:1 (탱크에 미리 세팅)
 *
 * ── 명령어 ──
 *   fill / stopfill / fire / stopfire / stop / status / vol X / test
 */

// ══════════════════════════════════════════
// 핀 설정
// ══════════════════════════════════════════
const int PUMP_LEFT_PIN  = 4;
const int PUMP_RIGHT_PIN = 5;
const int PUMP_FIRE_PIN  = 6;
const int FLOW_LEFT_PIN  = 3;  // INT1
const int FLOW_RIGHT_PIN = 2;  // INT0

// ══════════════════════════════════════════
// 릴레이 폴리티 — 여기 한 줄만 바꾸면 전체 반전
//   true  = 액티브 HIGH (HIGH=ON)
//   false = 액티브 LOW  (LOW=ON)  ← stop이 안 먹으면 이쪽일 확률 높음
// ══════════════════════════════════════════
const bool RELAY_ACTIVE_HIGH = false;  // 실측: 액티브 LOW (부팅 ON + stop 무효 → LOW=ON 확정)
const int RELAY_ON  = RELAY_ACTIVE_HIGH ? HIGH : LOW;
const int RELAY_OFF = RELAY_ACTIVE_HIGH ? LOW  : HIGH;

const bool LEFT_IS_ALGINATE = true;  // TODO: 실험 전 확인

// ══════════════════════════════════════════
// 실험 파라미터
// ══════════════════════════════════════════
float TANK_CAPACITY_ML  = 5000.0; // 자동 주입 정지 기준 (mL) — 2.5L × 2 = 5000mL
float PULSES_PER_ML     = 0.45;   // 유량계 1mL당 펄스수 (실측 보정 필요)

// ══════════════════════════════════════════
// 비율 제어 파라미터
//   RATIO_TOLERANCE : 허용 오차 (0.05 = ±5%)
//   RATIO_MIN_VOL   : 비율 체크 시작 최소 누적량 (mL) — 초기 노이즈 방지
// ══════════════════════════════════════════
const float RATIO_TOLERANCE = 0.05;   // ±5% 허용
const float RATIO_MIN_VOL   = 10.0;  // 양쪽 합계 10mL 이상부터 비율 체크

// ══════════════════════════════════════════
// 상태
// ══════════════════════════════════════════
bool isFilling = false;
bool isFiring  = false;

// 비율 제어에 의해 개별 펌프가 일시 정지됐는지 추적
bool pumpLeftActive  = false;
bool pumpRightActive = false;

// ══════════════════════════════════════════
// 계측 변수
// ══════════════════════════════════════════
volatile long pulseCountLeft  = 0;
volatile long pulseCountRight = 0;

long  lastPulseLeft  = 0, lastPulseRight = 0;
float flowRateLeft   = 0, flowRateRight  = 0;
float totalVolLeft   = 0, totalVolRight  = 0;
float tankVolume     = 0;

unsigned long lastCalcTime = 0;
const int CALC_INTERVAL_MS = 200;

String serialBuffer = "";

// ══════════════════════════════════════════
// 인터럽트 핸들러
// ══════════════════════════════════════════
void countLeft()  { pulseCountLeft++;  }
void countRight() { pulseCountRight++; }

// 인터럽트 보호 원자적 읽기
long readPulse(volatile long &src) {
  long v;
  noInterrupts();
  v = src;
  interrupts();
  return v;
}

// ══════════════════════════════════════════
// 초기화
// ══════════════════════════════════════════
void setup() {
  Serial.begin(9600);

  // digitalWrite(OFF) → pinMode(OUTPUT) 순서: OUTPUT 전환 직후 바로 OFF 레벨 구동(글리치 최소화)
  digitalWrite(PUMP_LEFT_PIN,  RELAY_OFF);
  digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF);
  digitalWrite(PUMP_FIRE_PIN,  RELAY_OFF);
  pinMode(PUMP_LEFT_PIN,  OUTPUT);
  pinMode(PUMP_RIGHT_PIN, OUTPUT);
  pinMode(PUMP_FIRE_PIN,  OUTPUT);
  // 한 번 더 확실히 OFF
  digitalWrite(PUMP_LEFT_PIN,  RELAY_OFF);
  digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF);
  digitalWrite(PUMP_FIRE_PIN,  RELAY_OFF);

  // 유량계: 홀센서 오픈컬렉터 → 내부 풀업 필수 (없으면 펄스 미검출/노이즈)
  pinMode(FLOW_LEFT_PIN,  INPUT_PULLUP);
  pinMode(FLOW_RIGHT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FLOW_LEFT_PIN),  countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_RIGHT_PIN), countRight, RISING);

  Serial.println("====================================");
  Serial.println(" Bio-hydrogel 발사 시스템 v6");
  Serial.println(" 1:1 비율 피드백 제어 적용");
  Serial.println("====================================");
  Serial.print("[릴레이] 폴리티 = ");
  Serial.println(RELAY_ACTIVE_HIGH ? "액티브 HIGH (HIGH=ON)" : "액티브 LOW (LOW=ON)");
  Serial.println("[검증] 'test' 입력해서 ON/OFF 딸깍 맞는지 확인하세요.");
  Serial.println("[농도] 알긴산 1.0wt% + 피트산 0.5wt%  |  CaCl2 2.0wt%");
  Serial.print("[라인] 왼쪽=");
  Serial.print(LEFT_IS_ALGINATE ? "알긴산" : "CaCl2");
  Serial.print(" / 오른쪽=");
  Serial.println(LEFT_IS_ALGINATE ? "CaCl2" : "알긴산");
  Serial.print("[비율] 목표 1:1  |  허용 오차 ±");
  Serial.print(RATIO_TOLERANCE * 100, 0);
  Serial.print("%  |  체크 시작 ");
  Serial.print(RATIO_MIN_VOL, 0);
  Serial.println("mL 이후");
  Serial.print("[탱크] 자동정지 기준 = "); Serial.print(TANK_CAPACITY_ML, 0); Serial.println(" mL");
  Serial.println("------------------------------------");
  Serial.println("fill / stopfill / fire / stopfire / stop / status / vol X / test");
  Serial.println("====================================\n");
}

// ══════════════════════════════════════════
// 메인 루프
// ══════════════════════════════════════════
void loop() {
  readSerial();
  if (isFilling) controlLoop();
}

// ══════════════════════════════════════════
// 시리얼 입력
// ══════════════════════════════════════════
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuffer.trim();
      if (serialBuffer.length() > 0) handleCommand(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

void handleCommand(String cmd) {
  cmd.toLowerCase();

  if (cmd == "fill") {
    if (isFilling) { Serial.println("[!] 이미 주입 중."); return; }
    startFilling();

  } else if (cmd == "stopfill") {
    if (!isFilling) { Serial.println("[!] 주입 중이 아님."); return; }
    isFilling = false;
    pumpLeftActive  = false;
    pumpRightActive = false;
    digitalWrite(PUMP_LEFT_PIN,  RELAY_OFF);
    digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF);
    Serial.println("[주입 정지] 수동 정지.");
    printStatus();

  } else if (cmd == "fire") {
    if (isFiring) { Serial.println("[!] 이미 발사 중."); return; }
    isFiring = true;
    digitalWrite(PUMP_FIRE_PIN, RELAY_ON);
    Serial.print("[발사 시작] 탱크: "); Serial.print(tankVolume, 0); Serial.println(" mL");

  } else if (cmd == "stopfire") {
    if (!isFiring) { Serial.println("[!] 발사 중이 아님."); return; }
    isFiring = false;
    digitalWrite(PUMP_FIRE_PIN, RELAY_OFF);
    Serial.println("[발사 정지]");

  } else if (cmd == "stop") {
    isFilling       = false;
    isFiring        = false;
    pumpLeftActive  = false;
    pumpRightActive = false;
    digitalWrite(PUMP_LEFT_PIN,  RELAY_OFF);
    digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF);
    digitalWrite(PUMP_FIRE_PIN,  RELAY_OFF);
    Serial.println("[전체 정지]");
    printStatus();

  } else if (cmd == "status") {
    printStatus();

  } else if (cmd == "test") {
    relayTest();

  } else if (cmd.startsWith("vol ")) {
    float val = cmd.substring(4).toFloat();
    if (val > 0) {
      TANK_CAPACITY_ML = val;
      Serial.print("[설정] 탱크 자동정지 = "); Serial.print(TANK_CAPACITY_ML, 0); Serial.println(" mL");
    } else {
      Serial.println("[오류] 예: vol 5000");
    }

  } else {
    Serial.print("[?] 모르는 명령: "); Serial.println(cmd);
  }
}

// ══════════════════════════════════════════
// 릴레이 폴리티 검증 — 펌프 부하 떼고 딸깍 소리로 확인
// ══════════════════════════════════════════
void relayTest() {
  Serial.println("\n=== 릴레이 테스트 (딸깍 소리로 확인) ===");
  Serial.print("현재 설정: ");
  Serial.println(RELAY_ACTIVE_HIGH ? "액티브 HIGH" : "액티브 LOW");
  int pins[3] = { PUMP_LEFT_PIN, PUMP_RIGHT_PIN, PUMP_FIRE_PIN };
  const char* names[3] = { "왼쪽펌프(4)", "오른쪽펌프(5)", "발사펌프(6)" };
  for (int i = 0; i < 3; i++) {
    Serial.print("["); Serial.print(names[i]); Serial.println("] ON 이라고 말함 → 지금 딸깍 켜져야 정상");
    digitalWrite(pins[i], RELAY_ON);
    delay(1200);
    Serial.println("   OFF → 지금 꺼져야 정상");
    digitalWrite(pins[i], RELAY_OFF);
    delay(800);
  }
  Serial.println("=== 테스트 끝 ===");
  Serial.println("ON/OFF가 반대로 동작했다면 코드 상단 RELAY_ACTIVE_HIGH 를 false 로 바꿔 재업로드.\n");
}

// ══════════════════════════════════════════
// 주입 시작
// ══════════════════════════════════════════
void startFilling() {
  tankVolume      = 0;
  totalVolLeft    = 0;
  totalVolRight   = 0;
  pumpLeftActive  = true;
  pumpRightActive = true;

  noInterrupts();
  pulseCountLeft  = 0;
  pulseCountRight = 0;
  interrupts();

  lastPulseLeft  = 0;
  lastPulseRight = 0;
  lastCalcTime   = millis();

  isFilling = true;
  digitalWrite(PUMP_LEFT_PIN,  RELAY_ON);
  digitalWrite(PUMP_RIGHT_PIN, RELAY_ON);

  Serial.println("[주입 시작]");
  Serial.println("시간(ms)\t유량L(mL/s)\t유량R(mL/s)\t탱크(mL)\t비율(L/R)");
}

// ══════════════════════════════════════════
// 제어 루프 (200ms 주기)
//   1) 유량 계산 및 누적
//   2) ★ 1:1 비율 피드백 제어 ★
//   3) 탱크 자동 정지
// ══════════════════════════════════════════
void controlLoop() {
  unsigned long now = millis();
  if (now - lastCalcTime < CALC_INTERVAL_MS) return;

  float dt = (now - lastCalcTime) / 1000.0;
  lastCalcTime = now;

  long pL = readPulse(pulseCountLeft);
  long pR = readPulse(pulseCountRight);

  long dL = pL - lastPulseLeft;
  long dR = pR - lastPulseRight;
  lastPulseLeft  = pL;
  lastPulseRight = pR;

  float addedLeft  = dL / PULSES_PER_ML;
  float addedRight = dR / PULSES_PER_ML;
  flowRateLeft  = addedLeft  / dt;
  flowRateRight = addedRight / dt;

  totalVolLeft  += addedLeft;
  totalVolRight += addedRight;
  tankVolume    += addedLeft + addedRight;

  // ── ★ 1:1 비율 피드백 제어 ★ ──────────────────────────
  // 초기 소량 구간(노이즈)은 건너뜀
  if (totalVolLeft + totalVolRight >= RATIO_MIN_VOL && totalVolRight > 0) {
    float ratio = totalVolLeft / totalVolRight;

    if (ratio > 1.0 + RATIO_TOLERANCE) {
      // 왼쪽 과다 → 왼쪽 일시 정지
      if (pumpLeftActive) {
        digitalWrite(PUMP_LEFT_PIN, RELAY_OFF);
        pumpLeftActive = false;
        Serial.println("[비율] 왼쪽 과다 → 왼쪽 펌프 일시 정지");
      }
      if (!pumpRightActive) {
        digitalWrite(PUMP_RIGHT_PIN, RELAY_ON);
        pumpRightActive = true;
      }

    } else if (ratio < 1.0 - RATIO_TOLERANCE) {
      // 오른쪽 과다 → 오른쪽 일시 정지
      if (pumpRightActive) {
        digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF);
        pumpRightActive = false;
        Serial.println("[비율] 오른쪽 과다 → 오른쪽 펌프 일시 정지");
      }
      if (!pumpLeftActive) {
        digitalWrite(PUMP_LEFT_PIN, RELAY_ON);
        pumpLeftActive = true;
      }

    } else {
      // 정상 범위 → 양쪽 모두 복귀
      if (!pumpLeftActive) {
        digitalWrite(PUMP_LEFT_PIN, RELAY_ON);
        pumpLeftActive = true;
        Serial.println("[비율] 복귀 → 왼쪽 펌프 재가동");
      }
      if (!pumpRightActive) {
        digitalWrite(PUMP_RIGHT_PIN, RELAY_ON);
        pumpRightActive = true;
        Serial.println("[비율] 복귀 → 오른쪽 펌프 재가동");
      }
    }
  }
  // ────────────────────────────────────────────────────────

  // 유량 로그 (비율 컬럼 추가)
  float currentRatio = (totalVolRight > 0) ? totalVolLeft / totalVolRight : 0.0;
  Serial.print(now);             Serial.print("\t");
  Serial.print(flowRateLeft, 2); Serial.print("\t\t");
  Serial.print(flowRateRight,2); Serial.print("\t\t");
  Serial.print(tankVolume,   0); Serial.print("\t\t");
  Serial.println(currentRatio,  3);

  // 탱크 자동 정지
  if (tankVolume >= TANK_CAPACITY_ML) {
    isFilling       = false;
    pumpLeftActive  = false;
    pumpRightActive = false;
    digitalWrite(PUMP_LEFT_PIN,  RELAY_OFF);
    digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF);

    float finalRatio = (totalVolRight > 0) ? totalVolLeft / totalVolRight : 0.0;

    Serial.println("\n====================================");
    Serial.println("[탱크 FULL] 주입 자동 정지");
    Serial.print("충전량: ");        Serial.print(tankVolume,    0); Serial.println(" mL");
    Serial.print("왼쪽 누적: ");     Serial.print(totalVolLeft,  1); Serial.println(" mL");
    Serial.print("오른쪽 누적: ");   Serial.print(totalVolRight, 1); Serial.println(" mL");
    Serial.print("최종 혼합 비율: "); Serial.print(finalRatio,   3);
    Serial.println(abs(finalRatio - 1.0) <= RATIO_TOLERANCE ? "  ✓ 정상" : "  ✗ 편차 초과");
    Serial.print("발사 펌프: ");      Serial.println(isFiring ? "가동 중" : "정지 중");
    Serial.println("'fire' 발사 / 'fill' 재충전");
    Serial.println("====================================");
  }
}

// ══════════════════════════════════════════
// 상태 출력
// ══════════════════════════════════════════
void printStatus() {
  float currentRatio = (totalVolRight > 0) ? totalVolLeft / totalVolRight : 0.0;

  Serial.println("-------- STATUS --------");
  Serial.print("주입 펌프 왼쪽: "); Serial.println(pumpLeftActive  ? "가동 중" : "정지");
  Serial.print("주입 펌프 오른쪽: "); Serial.println(pumpRightActive ? "가동 중" : "정지");
  Serial.print("발사 펌프: ");     Serial.println(isFiring ? "가동 중" : "정지");
  Serial.print("릴레이 폴리티: "); Serial.println(RELAY_ACTIVE_HIGH ? "액티브 HIGH" : "액티브 LOW");
  Serial.print("라인: 왼쪽=");
  Serial.print(LEFT_IS_ALGINATE ? "알긴산" : "CaCl2");
  Serial.print(" / 오른쪽=");
  Serial.println(LEFT_IS_ALGINATE ? "CaCl2" : "알긴산");
  Serial.print("탱크 현재: ");      Serial.print(tankVolume,      0); Serial.println(" mL");
  Serial.print("왼쪽 누적: ");      Serial.print(totalVolLeft,    1); Serial.println(" mL");
  Serial.print("오른쪽 누적: ");    Serial.print(totalVolRight,   1); Serial.println(" mL");
  Serial.print("현재 비율(L/R): "); Serial.print(currentRatio,    3);
  Serial.println(abs(currentRatio - 1.0) <= RATIO_TOLERANCE ? "  ✓ 정상" : "  ✗ 편차 초과");
  Serial.print("탱크 자동정지: ");  Serial.print(TANK_CAPACITY_ML, 0); Serial.println(" mL");
  Serial.print("비율 허용 오차: ±"); Serial.print(RATIO_TOLERANCE * 100, 0); Serial.println("%");
  Serial.println("------------------------");
}
