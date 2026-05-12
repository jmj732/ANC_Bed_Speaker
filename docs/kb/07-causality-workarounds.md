# §07 인과성 우회 — ref_lead = 0 해결 후보

> 본 시스템의 **본질적 병목**: ref mic과 error mic이 가까워 `ref_lead < sec_path_delay`.
> Feedforward 광대역 FxLMS가 인과적으로 불가능 (`§02 §6`, `§01 §6`).
> 현재는 narrowband(가상 reference)로 우회 중. 본 절은 **다른 우회 경로** 전부 정리.

---

## A. 피드백 ANC 계열 (No reference mic 필요)

### A1. Feedback ANC (FB-ANC) — **최우선 후보**
- 원리: ref mic 없이 error mic만 사용. 1차 disturbance를 e에서 역추정.
- 식:
  - `d̂(n) = e(n) - ŷ(n)` (e에서 anti-noise 영향 제거 → estimated disturbance)
  - `ŷ(n) = Ŝ(z) · y(n)` (sec_path 모델 통과한 출력 예측)
  - LMS reference로 `d̂` 사용
- 이론 근거: **Internal Model Control (IMC)**, Kuo & Morgan §3
- **이론적 추가 이득**: 광대역 +5~15dB 가능 (사례 보고)
- **본 시스템 적용성**: ✅ Tier 3 — **ref_lead=0 문제 직접 해결**
- **차단 요인**:
  - sec_path 정확도 의존도 매우 큼 (5%만 어긋나도 발산)
  - 닫힌 루프 안정성 분석 필요 (Nyquist criterion)
  - 본 sec_path measure는 통과했으므로 가능성 있음
- **구현 비용**: 새 모드 `fb`, anc_fb.c 분리, 약 200줄
- **위험**: 발산 시 광대역 잡음 폭주 (silent safety 더 보수적으로 + soft start)

### A2. Hybrid Feedforward + Feedback
- 원리: ref mic (현 nb) + FB-ANC 병렬
- 식: `y = y_FF + y_FB`, 각각 독립 LMS
- 본 시스템 적용성: ✅ Tier 3
- **차단**: ref_lead=0이라 FF 기여 미미 → 단독 FB와 큰 차이 없음

### A3. Internal Model Control (IMC) ANC
- 원리: A1의 정식화. control loop를 `1 / (1 - Ĝ·G⁻¹)` 형태로 명시.
- 적용성: A1과 등가, 단 설계 명확

---

## B. 합성 Reference (Synthesize what's missing)

### B1. Linear Predictor (LP)
- 원리: e(n)의 K 샘플 미래를 LP로 예측, 가상 reference로 사용
- 식: `e_pred(n+k) = Σ a_i · e(n-i)`, AR 모델
- 본 시스템 적용성: ⚠️ Tier 2
- **이득**:
  - 협대역: 미미 (oscillator가 이미 정확한 미래 예측)
  - 광대역: 예측 가능한 부분만 cancel
- **단점**: K가 sec_path_delay (~140 samples)이라 예측 오차 위상 누적 → 광대역 미효율

### B2. Adaptive Line Enhancer (ALE)
- 원리: e(n) → delay → LP → tonal 성분 추출 → reference
- 본 시스템 적용성: ⚠️ Tier 2
- **현 nb 대비 차별점**: f0 명시 검출 없이 tonal 성분 자동 추출
- **이득**: f0 변동 빈번한 burst에서 nb 대비 robust 가능성
- **차단**: ALE adaptation rate 튜닝 까다로움

### B3. Cepstral Prediction
- 원리: cepstrum 도메인에서 미래 추정
- 적용성: ❌ — FFT 인프라 부재, 이득 불명

---

## C. 주기 신호 특화 (Periodic Disturbance)

### C1. Iterative Learning Control (ILC)
- 원리: 동일 task가 반복될 때 prior period error로 control signal 정제
- 식: `u_{k+1}(t) = u_k(t) + L · e_k(t)`
- **이론적 근거**: 코골이 burst 내 1~10 사이클 주기 구조
- 본 시스템 적용성: ⚠️ Tier 3
- **차단**: burst마다 길이가 달라서 task 정의 불명확

### C2. Repetitive Control (RC)
- 원리: Internal model `1/(1-z^{-N})` (period N) → harmonics에 infinite gain
- 본 시스템 적용성: ✅ Tier 3
- **이론적 이득**: 정확한 주기 신호에서 perfect cancellation
- **차단**: burst 내 f0 변동 시 cancellation 깨짐
- **결합 가능**: §06 G3 (MFxLMS)와

### C3. Phase-Locked Loop (PLL) Reference
- 원리: error 신호에서 PLL로 위상 추출 → 동기 oscillator 생성
- 본 시스템 적용성: ✅ Tier 2 (현 autocorr 대체)
- **차이점 vs 현 nb**: f0 detection 후 oscillator를 free-running 시키는 현 구조와 달리, error에 동기화 (closed-loop)
- **이득**: f0 drift에 자동 추종, jitter ↓
- **차단**: 본 시스템 nb_step 구조 재설계 필요

---

## D. 외란 관측기 (Observer-based)

### D1. Disturbance Observer (DOB)
- 원리: plant 출력에서 외란 역추정
- 본 시스템 적용성: ⚠️ Tier 3
- **현 sec_path FIR 모델로 등가 효과 일부 달성** — DOB 추가 이득 불명
- 사례: 산업 정현파 제거에 표준

### D2. Kalman Filter 외란 추정
- state: [w_c, w_s, f0, df0/dt]
- 본 시스템 적용성: ✅ Tier 3 (§08 K1 참조)

### D3. H∞ Robust Control
- 모델 불확실성 하에서 최적
- 적용성: ❌ — 이론 무거움, 본 sec_path 정적이라 이득 미미

### D4. Sliding Mode Control
- 적용성: ❌ — ANC 적용 사례 드뭄

---

## E. 공간 / 가상 센싱 (Spatial)

### E1. Virtual Sensing
- 원리: 다수 mic → 가상 quiet zone에 가상 error mic 합성
- 본 시스템 적용성: ❌ — HW 추가 필요 (mic ≥ 3)
- **사례**: 헤드레스트 ANC, Forward-difference prediction

### E2. Acoustic Energy Density Control
- mic + 가속도계 결합
- 적용성: ❌ — HW

### E3. Multi-Channel ANC (M×K)
- M reference, K error
- 적용성: ❌ — HW

---

## F. 물리적 해결 (Physical)

### F1. Reference Mic 재배치
- ref mic을 스피커로부터 멀리, 코골이 음원에 가깝게
- **본 시스템 적용성**: ✅ — `§02 §6`에서 "유일한 실제 해결" 명시
- **예상 이득**: +10~15dB (broadband 부활)
- **비용**: 물리 작업, 와이어 정리
- **현 KB §05 §D Card C** = 이것

### F2. Sec_path 단축
- error mic을 스피커에 가깝게 → sec_path delay ↓ → 인과 마진 ↑
- 적용성: ⚠️ — 사용자 청취 위치와 충돌

### F3. 스피커 방향 변경
- 직접 청취자 향함 → null 형성
- 적용성: ⚠️ — 본 침대 시나리오 특화 검토 필요

---

## G. 모드 분리 / 하이브리드 정책

### G1. 광대역 FB + 협대역 FF Hybrid
- 광대역은 A1 (FB), 협대역은 현 nb (FF) — 각자 강한 영역만
- 본 시스템 적용성: ✅ Tier 3 — **A1 구현 후 합성**

### G2. Burst-가중 모드 스위칭
- 코골이 burst 내: nb FF (현재 +5dB)
- burst 사이: FB로 정적 잡음 흡수
- 적용성: ⚠️ Tier 3

---

## 본 절 종합 — 다음 카드 우선순위

| 순위 | 기법 | Tier | ref_lead=0 해결? | 예상 이득 | 비고 |
|---|---|---|---|---|---|
| 1 | **F1. ref mic 재배치 (물리)** | 4 | ✅ 완전 해결 | +10~15dB | KB §05 Card C, 가장 큰 이득 |
| 2 | **A1. Feedback ANC** | 3 | ✅ 우회 | +5~15dB | SW 한도 내 최대치, 안정성 가드 필요 |
| 3 | **A2/G1. Hybrid FF+FB** | 3 | ✅ 우회 | A1 +2~5dB | A1 검증 후 |
| 4 | **B2. ALE** | 2 | ⚠️ 부분 | 현 nb 대비 미미 | nb robust 보조 |
| 5 | **C2. Repetitive Control** | 3 | ⚠️ 부분 | 변동 적은 burst만 | 실효성 검증 어려움 |
| 6 | **C3. PLL Reference** | 2 | ⚠️ 부분 | jitter ↓ | nb_step 재설계 |
| 7 | **D2. Kalman 외란** | 3 | ⚠️ 부분 | dip ↓ | §08과 함께 |

**최우선 액션**: F1 (물리) 또는 A1 (Feedback ANC) — 둘 다 **본질적 천장 깨기**.
나머지는 현 +5dB 근처에서 ±2dB 튜닝.
