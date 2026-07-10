/**
 * Bio-hydrogel 발사 장치 v7 (한쪽만 나오는 문제 해결판)
 *
 * ── 하드웨어 구조 ──
 *   [왼쪽 탱크] → 왼쪽 펌프(핀4) → 왼쪽 유량계(핀3) → ─┐
 *                                                          Y합류 → [저장 탱크] → 앞쪽 펌프(핀6) → 발사
 *   [오른쪽 탱크] → 오른쪽 펌프(핀5) → 오른쪽 유량계(핀2) → ─┘
 *
 * ── ★ v7 변경 (한쪽만 물 나오던 문제 수정) ──
 *   원인: 한쪽 유량계가 펄스를 못 읽으면(L/R=0.000) 비율 제어가
 *         "한쪽 과다"로 오판 → 멀쩡한 쪽 펌프를 꺼버림 → 한쪽만 분사.
 *
 *   해결:
 *   1) ENABLE_RATIO_CONTROL 플래그 추가 (기본 false = 양쪽 무조건 동시 구동)
 *      → 펌프 특성이 비슷하면 이 모드로 1:1 충분히 맞음. 가장 안전.
 *   2) 유량계 STALL 감지: 펌프는 도는데 한쪽 펄스가 STALL_TIMEOUT 동안
 *      안 들어오면 = 유량계 고장으로 판단 → 비율 제어 자동 해제 +
 *      양쪽 펌프 강제 ON (펌프가 꺼진 채 멈추는 일 없음).
 *   3) 비율 제어가 켜져 있어도 펌프를 OFF로 두지 않고, 듀티比 조절식이 아닌
 *      "느린 쪽 보조"가 아니라 절대 양쪽 중 하나는 항상 켜져 있게 보장.
 *
 *   ※ 그래도 한쪽 유량계가 0이면 그건 배선/센서 문제다.
 *     핀3(왼쪽) 또는 핀2(오른쪽) 신호선, 풀업, GND 공통 연결 확인 권장.
 *     단, 펌프 분사 자체는 v7에서 양쪽 정상 동작한다.
 *
 * ── 릴레이 폴리티 ──
 *   stop이 안 먹히면 RELAY_ACTIVE_HIGH = false 로 변경 후 재업로드
 *
 * ── 명령어 ──
 *   fill / stopfill / fire / stopfire / stop / status / vol X / test / ratio
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
// 릴레이 폴리티
// ══════════════════════════════════════════
const bool RELAY_ACTIVE_HIGH = false;
const int  RELAY_ON  = RELAY_ACTIVE_HIGH ? HIGH : LOW;
const int  RELAY_OFF = RELAY_ACTIVE_HIGH ? LOW  : HIGH;

const bool LEFT_IS_ALGINATE = true;

// ══════════════════════════════════════════
// 실험 파라미터
// ══════════════════════════════════════════
float TANK_CAPACITY_ML = 5000.0;  // 2.5L × 2
float PULSES_PER_ML    = 0.45;    // 실측 보정 필요

// ══════════════════════════════════════════
// ★ 비율 제어 설정 ★
//   ENABLE_RATIO_CONTROL : false = 양쪽 무조건 동시 구동 (안전, 기본값)
//                          true  = 유량 기반 1:1 피드백 (유량계 둘 다 정상일 때만)
//   런타임에 'ratio' 명령으로 토글 가능
// ══════════════════════════════════════════
bool        enableRatioControl = false;   // ← 기본 OFF (한쪽만 나오는 문제 회피)
const float RATIO_TOLERANCE    = 0.05;    // ±5%
const float RATIO_MIN_VOL      = 10.0;    // 10mL 이후부터 체크

// 유량계 STALL 감지: 펌프 도는데 이 시간(ms) 동안 펄스 0이면 센서 고장 판단
const unsigned long STALL_TIMEOUT_MS = 3000;

// ══════════════════════════════════════════
// 상태
// ══════════════════════════════════════════
bool isFilling       = false;
bool isFiring        = false;
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
const int     CALC_INTERVAL = 200;  // ms

// STALL 감지용: 마지막으로 펄스가 증가한 시각
unsigned long lastPulseTimeLeft  = 0;
unsigned long lastPulseTimeRight = 0;

// ══════════════════════════════════════════
// 시리얼 버퍼 (String 미사용)
// ══════════════════════════════════════════
const int SERIAL_BUF = 24;
char      serialBuf[SERIAL_BUF];
int       serialIdx = 0;

// ══════════════════════════════════════════
// 인터럽트
// ══════════════════════════════════════════
void countLeft()  { pulseCountLeft++;  }
void countRight() { pulseCountRight++; }

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

  digitalWrite(PUMP_LEFT_PIN,  RELAY_OFF);
  digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF);
  digitalWrite(PUMP_FIRE_PIN,  RELAY_OFF);
  pinMode(PUMP_LEFT_PIN,  OUTPUT);
  pinMode(PUMP_RIGHT_PIN, OUTPUT);
  pinMode(PUMP_FIRE_PIN,  OUTPUT);
  digitalWrite(PUMP_LEFT_PIN,  RELAY_OFF);
  digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF);
  digitalWrite(PUMP_FIRE_PIN,  RELAY_OFF);

  pinMode(FLOW_LEFT_PIN,  INPUT_PULLUP);
  pinMode(FLOW_RIGHT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_LEFT_PIN),  countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_RIGHT_PIN), countRight, RISING);

  Serial.println(F("=================================="));
  Serial.println(F(" Bio-hydrogel v7"));
  Serial.println(F("=================================="));
  Serial.print(F("[릴레이] "));
  Serial.println(RELAY_ACTIVE_HIGH ? F("ACTIVE HIGH") : F("ACTIVE LOW"));
  Serial.print(F("[비율제어] "));
  Serial.println(enableRatioControl ? F("ON") : F("OFF (양쪽 동시구동)"));
  Serial.print(F("[탱크] "));
  Serial.print(TANK_CAPACITY_ML, 0);
  Serial.println(F(" mL"));
  Serial.println(F("fill/stopfill/fire/stopfire/stop/status/vol X/test/ratio"));
  Serial.println(F("==================================\n"));
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
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        for (int i = 0; i < serialIdx; i++)
          if (serialBuf[i] >= 'A' && serialBuf[i] <= 'Z') serialBuf[i] += 32;
        handleCommand(serialBuf);
        serialIdx = 0;
      }
    } else if (serialIdx < SERIAL_BUF - 1) {
      serialBuf[serialIdx++] = c;
    }
  }
}

void handleCommand(const char* cmd) {

  if (strcmp(cmd, "fill") == 0) {
    if (isFilling) { Serial.println(F("[!] 이미 주입 중")); return; }
    startFilling();

  } else if (strcmp(cmd, "stopfill") == 0) {
    if (!isFilling) { Serial.println(F("[!] 주입 중 아님")); return; }
    stopAllPumps();
    Serial.println(F("[주입 정지] 수동"));
    printStatus();

  } else if (strcmp(cmd, "fire") == 0) {
    if (isFiring) { Serial.println(F("[!] 이미 발사 중")); return; }
    isFiring = true;
    digitalWrite(PUMP_FIRE_PIN, RELAY_ON);
    Serial.print(F("[발사] 탱크 ")); Serial.print(tankVolume, 0); Serial.println(F(" mL"));

  } else if (strcmp(cmd, "stopfire") == 0) {
    if (!isFiring) { Serial.println(F("[!] 발사 중 아님")); return; }
    isFiring = false;
    digitalWrite(PUMP_FIRE_PIN, RELAY_OFF);
    Serial.println(F("[발사 정지]"));

  } else if (strcmp(cmd, "stop") == 0) {
    stopAllPumps();
    isFiring = false;
    digitalWrite(PUMP_FIRE_PIN, RELAY_OFF);
    Serial.println(F("[전체 정지]"));
    printStatus();

  } else if (strcmp(cmd, "status") == 0) {
    printStatus();

  } else if (strcmp(cmd, "test") == 0) {
    relayTest();

  } else if (strcmp(cmd, "ratio") == 0) {
    enableRatioControl = !enableRatioControl;
    Serial.print(F("[비율제어] "));
    Serial.println(enableRatioControl ? F("ON") : F("OFF (양쪽 동시구동)"));
    // 켜고 끌 때 양쪽 펌프 상태 안전하게 정렬
    if (isFilling) {
      digitalWrite(PUMP_LEFT_PIN,  RELAY_ON);
      digitalWrite(PUMP_RIGHT_PIN, RELAY_ON);
      pumpLeftActive = pumpRightActive = true;
    }

  } else if (strncmp(cmd, "vol ", 4) == 0) {
    float val = atof(cmd + 4);
    if (val > 0) {
      TANK_CAPACITY_ML = val;
      Serial.print(F("[설정] 탱크 = ")); Serial.print(TANK_CAPACITY_ML, 0); Serial.println(F(" mL"));
    } else {
      Serial.println(F("[오류] 예: vol 5000"));
    }

  } else {
    Serial.print(F("[?] ")); Serial.println(cmd);
  }
}

// 양쪽 주입 펌프만 정지 (발사 펌프는 건드리지 않음)
void stopAllPumps() {
  isFilling = pumpLeftActive = pumpRightActive = false;
  digitalWrite(PUMP_LEFT_PIN,  RELAY_OFF);
  digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF);
}

// ══════════════════════════════════════════
// 릴레이 폴리티 검증
// ══════════════════════════════════════════
void relayTest() {
  Serial.println(F("\n=== 릴레이 테스트 ==="));
  Serial.println(RELAY_ACTIVE_HIGH ? F("ACTIVE HIGH") : F("ACTIVE LOW"));
  const int pins[3] = { PUMP_LEFT_PIN, PUMP_RIGHT_PIN, PUMP_FIRE_PIN };
  for (int i = 0; i < 3; i++) {
    Serial.print(F("핀")); Serial.print(pins[i]); Serial.println(F(" ON→딸깍"));
    digitalWrite(pins[i], RELAY_ON);  delay(1200);
    Serial.println(F("  OFF→꺼짐"));
    digitalWrite(pins[i], RELAY_OFF); delay(800);
  }
  Serial.println(F("반대면 RELAY_ACTIVE_HIGH 변경 후 재업로드\n"));
}

// ══════════════════════════════════════════
// 주입 시작
// ══════════════════════════════════════════
void startFilling() {
  tankVolume = totalVolLeft = totalVolRight = 0;
  pumpLeftActive = pumpRightActive = true;

  noInterrupts();
  pulseCountLeft = pulseCountRight = 0;
  interrupts();

  lastPulseLeft = lastPulseRight = 0;
  unsigned long now = millis();
  lastCalcTime = now;
  lastPulseTimeLeft = lastPulseTimeRight = now;
  isFilling = true;

  // ★ 항상 양쪽 동시 ON으로 시작
  digitalWrite(PUMP_LEFT_PIN,  RELAY_ON);
  digitalWrite(PUMP_RIGHT_PIN, RELAY_ON);

  Serial.println(F("[주입 시작]"));
  Serial.println(F("ms\tL(mL/s)\tR(mL/s)\t탱크(mL)\tL/R"));
}

// ══════════════════════════════════════════
// 제어 루프 (200ms)
// ══════════════════════════════════════════
void controlLoop() {
  unsigned long now = millis();
  if (now - lastCalcTime < CALC_INTERVAL) return;

  float dt = (now - lastCalcTime) / 1000.0;
  lastCalcTime = now;

  long pL = readPulse(pulseCountLeft);
  long pR = readPulse(pulseCountRight);

  long dPulseL = pL - lastPulseLeft;
  long dPulseR = pR - lastPulseRight;

  // STALL 감지: 펄스가 들어온 쪽은 타임스탬프 갱신
  if (dPulseL > 0) lastPulseTimeLeft  = now;
  if (dPulseR > 0) lastPulseTimeRight = now;

  float addedLeft  = dPulseL / PULSES_PER_ML;
  float addedRight = dPulseR / PULSES_PER_ML;
  lastPulseLeft  = pL;
  lastPulseRight = pR;

  flowRateLeft  = addedLeft  / dt;
  flowRateRight = addedRight / dt;
  totalVolLeft  += addedLeft;
  totalVolRight += addedRight;
  tankVolume    += addedLeft + addedRight;

  // ── 유량계 STALL 감지 ──────────────────────────────────
  // 펌프는 켜져있는데 한쪽 펄스가 STALL_TIMEOUT 동안 0이면 센서 고장 판단
  bool stallLeft  = pumpLeftActive  && (now - lastPulseTimeLeft  > STALL_TIMEOUT_MS);
  bool stallRight = pumpRightActive && (now - lastPulseTimeRight > STALL_TIMEOUT_MS);

  if (enableRatioControl && (stallLeft || stallRight)) {
    // 유량계가 죽었으면 비율 제어 신뢰 불가 → 자동 해제 + 양쪽 강제 ON
    enableRatioControl = false;
    digitalWrite(PUMP_LEFT_PIN,  RELAY_ON);
    digitalWrite(PUMP_RIGHT_PIN, RELAY_ON);
    pumpLeftActive = pumpRightActive = true;
    Serial.print(F("[STALL] "));
    Serial.print(stallLeft ? F("L") : F("R"));
    Serial.println(F(" 유량계 무응답 → 비율제어 해제, 양쪽 강제ON"));
  }
  // ────────────────────────────────────────────────────────

  // ── 1:1 비율 피드백 (켜져 있고, 양쪽 유량계 다 살아있을 때만) ──
  if (enableRatioControl &&
      totalVolLeft + totalVolRight >= RATIO_MIN_VOL &&
      totalVolLeft > 0 && totalVolRight > 0) {

    float ratio = totalVolLeft / totalVolRight;

    if (ratio > 1.0 + RATIO_TOLERANCE) {
      // 왼쪽 과다 → 왼쪽만 잠깐 정지 (오른쪽은 반드시 ON 유지)
      digitalWrite(PUMP_LEFT_PIN,  RELAY_OFF); pumpLeftActive  = false;
      digitalWrite(PUMP_RIGHT_PIN, RELAY_ON);  pumpRightActive = true;
    } else if (ratio < 1.0 - RATIO_TOLERANCE) {
      // 오른쪽 과다 → 오른쪽만 잠깐 정지 (왼쪽은 반드시 ON 유지)
      digitalWrite(PUMP_RIGHT_PIN, RELAY_OFF); pumpRightActive = false;
      digitalWrite(PUMP_LEFT_PIN,  RELAY_ON);  pumpLeftActive  = true;
    } else {
      // 정상 → 양쪽 복귀
      digitalWrite(PUMP_LEFT_PIN,  RELAY_ON);  pumpLeftActive  = true;
      digitalWrite(PUMP_RIGHT_PIN, RELAY_ON);  pumpRightActive = true;
    }
  } else if (!enableRatioControl) {
    // 비율 제어 OFF → 무조건 양쪽 동시 ON 보장
    if (!pumpLeftActive)  { digitalWrite(PUMP_LEFT_PIN,  RELAY_ON); pumpLeftActive  = true; }
    if (!pumpRightActive) { digitalWrite(PUMP_RIGHT_PIN, RELAY_ON); pumpRightActive = true; }
  }

  // 로그
  float ratio = (totalVolRight > 0) ? totalVolLeft / totalVolRight : 0.0;
  Serial.print(now);              Serial.print(F("\t"));
  Serial.print(flowRateLeft,  2); Serial.print(F("\t"));
  Serial.print(flowRateRight, 2); Serial.print(F("\t"));
  Serial.print(tankVolume,    0); Serial.print(F("\t\t"));
  Serial.println(ratio,       3);

  // 탱크 자동 정지
  if (tankVolume >= TANK_CAPACITY_ML) {
    stopAllPumps();
    float finalRatio = (totalVolRight > 0) ? totalVolLeft / totalVolRight : 0.0;
    Serial.println(F("\n=================================="));
    Serial.println(F("[탱크 FULL] 자동 정지"));
    Serial.print(F("충전: "));   Serial.print(tankVolume,    0); Serial.println(F(" mL"));
    Serial.print(F("L 누적: ")); Serial.print(totalVolLeft,  1); Serial.println(F(" mL"));
    Serial.print(F("R 누적: ")); Serial.print(totalVolRight, 1); Serial.println(F(" mL"));
    Serial.print(F("최종 L/R: ")); Serial.println(finalRatio, 3);
    Serial.println(F("fire=발사 / fill=재충전"));
    Serial.println(F("=================================="));
  }
}

// ══════════════════════════════════════════
// 상태 출력
// ══════════════════════════════════════════
void printStatus() {
  float ratio = (totalVolRight > 0) ? totalVolLeft / totalVolRight : 0.0;
  Serial.println(F("------- STATUS -------"));
  Serial.print(F("L펌프: ")); Serial.println(pumpLeftActive  ? F("ON") : F("OFF"));
  Serial.print(F("R펌프: ")); Serial.println(pumpRightActive ? F("ON") : F("OFF"));
  Serial.print(F("발사 : ")); Serial.println(isFiring        ? F("ON") : F("OFF"));
  Serial.print(F("비율제어: ")); Serial.println(enableRatioControl ? F("ON") : F("OFF"));
  Serial.print(F("탱크 : ")); Serial.print(tankVolume,    0); Serial.println(F(" mL"));
  Serial.print(F("L누적: ")); Serial.print(totalVolLeft,  1); Serial.println(F(" mL"));
  Serial.print(F("R누적: ")); Serial.print(totalVolRight, 1); Serial.println(F(" mL"));
  Serial.print(F("L/R  : ")); Serial.println(ratio, 3);
  Serial.print(F("한계 : ")); Serial.print(TANK_CAPACITY_ML, 0); Serial.println(F(" mL"));
  Serial.println(F("----------------------"));
}
