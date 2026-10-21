#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12
#define PIN_ECHO  13
#define PIN_SERVO 10

// configurable parameters for sonar
#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 180.0   // mm
#define _DIST_MAX 360.0   // mm

#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)

#define _EMA_ALPHA 0.3

// 서보에 맞게 보정할 값
#define _DUTY_MIN 1000   // 0도
#define _DUTY_NEU 1500   // 90도
#define _DUTY_MAX 2000   // 180도

float dist_ema, dist_prev;
unsigned long last_sampling_time;

Servo myservo;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_LED, HIGH);  // LED 끄기: active-low

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  // 초기 서보 위치인 90도에 대응하는 거리로 초기화
  dist_prev = (_DIST_MIN + _DIST_MAX) / 2.0;
  dist_ema = dist_prev;

  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_filtered;

  if (millis() - last_sampling_time < INTERVAL)
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 범위 필터: 범위 밖이거나 측정 실패이면 이전 유효값 유지
  if ((dist_raw == 0.0) ||
      (dist_raw < _DIST_MIN) ||
      (dist_raw > _DIST_MAX)) {
    dist_filtered = dist_prev;
    digitalWrite(PIN_LED, HIGH);  // 범위 밖: LED 끄기
  } else {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, LOW);   // 범위 안: LED 켜기
  }

  // EMA 필터
  dist_ema = _EMA_ALPHA * dist_filtered
           + (1.0 - _EMA_ALPHA) * dist_ema;

  // 180~360mm를 서보의 0~180도에 대응하는 펄스 폭으로 변환
  float ratio = (dist_ema - _DIST_MIN)
              / (_DIST_MAX - _DIST_MIN);
  ratio = constrain(ratio, 0.0, 1.0);

  int duty = (int)(_DUTY_MIN
                  + ratio * (_DUTY_MAX - _DUTY_MIN)
                  + 0.5);

  myservo.writeMicroseconds(duty);

  // 사진의 시리얼 플로터 출력 형식
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",ema:");   Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(myservo.read());
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");

  last_sampling_time += INTERVAL;
}

// 초음파 거리 측정: mm 단위 반환
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
