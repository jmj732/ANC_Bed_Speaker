# ANC 용어집 (KOR / ENG)

간단한 정의 위주로 정리했습니다. 더 자세한 설명이 필요하면 말씀해주세요.

---

## ANC 기본 개념

- **능동 소음 제어 / Active Noise Control (ANC)**: 마이크와 스피커로 소음을 상쇄하는 기술
- **1차 경로 / Primary Path**: 소음원 → 에러 마이크까지의 경로
- **2차 경로 / Secondary Path**: 제어 스피커 → 공간/스피커/마이크 → 에러 마이크까지의 경로
- **2차 경로 모델 / Ŝ (S-hat)**: 2차 경로를 추정한 임펄스 응답(필터 탭)

---

## 적응 알고리즘

- **적응 필터 / Adaptive Filter**: 오차에 따라 가중치를 계속 갱신하는 필터
- **LMS / Least Mean Squares**: 오차를 최소화하도록 가중치를 업데이트하는 기본 알고리즘
- **NLMS / Normalized LMS**: 입력 에너지로 정규화해 안정성을 높인 LMS
- **FxLMS / Filtered-x LMS**: 2차 경로를 고려한 LMS 적응 알고리즘

---

## 하드웨어 / 센서

- **레퍼런스 마이크 / Reference Mic**: 소음을 먼저 측정하는 입력 마이크
- **에러 마이크 / Error Mic**: 상쇄 결과(잔차)를 측정하는 마이크

---

## 신호 처리

- **임펄스 응답 / Impulse Response**: 시스템의 시간영역 특성(필터 탭)
- **FIR 필터 / Finite Impulse Response Filter**: 유한 길이 임펄스 응답 필터
- **탭/가중치 / Tap/Weight**: FIR/적응필터의 계수
- **대역통과 필터 / Bandpass Filter**: 특정 주파수 대역만 통과

---

## 구현 관련

- **샘플레이트 / Sample Rate**: 초당 샘플 수(Hz)
- **블록 크기 / Block Size**: 한 번에 처리하는 샘플 수
- **블록 처리 / Block Processing**: 샘플을 묶어서 처리하는 방식
- **링 버퍼 / Ring Buffer**: 히스토리를 쉬프트 없이 순환 저장
- **리미터 / Limiter**: 출력이 너무 커지지 않게 제한(예: tanh)

---

## 측정 / 평가

- **PSD / Power Spectral Density**: 주파수별 파워 분포
- **감쇠량 / Attenuation**: 소음이 줄어든 정도(dB)
- **노이즈 플로어 / Noise Floor**: 시스템의 바닥 잡음 수준

---

## 신호 흐름 다이어그램

```
소음원
  | (1차 경로)
  v
에러 마이크 ------------------------+
  |                                 |
  v                                 |
 ADC --------> 오차 신호 e(n) ------+----> 가중치 업데이트 (LMS/NLMS/FxLMS)
                                    |                 ^
                                    |                 |
레퍼런스 마이크                      |                 |
  |                                 |                 |
  v                                 |                 |
 ADC -> 대역통과/전처리 -> 참조 x(n) -> 링버퍼/블록처리 -> 적응 FIR W(z)
                                                    |
                                                    v
                                           제어 신호 u(n)
                                                    |
                                                    v
                                                DAC/출력
                                                    |
                                                    v
                                               제어 스피커
                                                    |
                                                    v
                                            (2차 경로 S(z))
                                                    |
                                                    +--------> 에러 마이크

보조 경로(Filtered-x):
참조 x(n) -> 2차 경로 모델 Ŝ(z) -> Filtered-x x̂(n) -> 가중치 업데이트
```
