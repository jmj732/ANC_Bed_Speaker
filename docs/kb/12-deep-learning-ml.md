# §12 Deep Learning / ML 기반 ANC

본 시스템 제약: Pi5 (ARM Cortex-A76, NEON), C 단일파일, period=32 @ 96kHz, budget=0.333ms/period, no GPU. Python/외부 ML 런타임은 사용자 원칙상 금지(추론까지 C 또는 NEON intrinsics).

---

## 12.1 Deep ANC (Zhang & Wang, 2021)

**원리**: end-to-end CNN/CRN이 reference에서 anti-noise를 직접 출력. 비선형/비정상 잡음에 강함.

**구성**: encoder-decoder, 8~16 layer, 약 1~5M parameter, STFT 도메인 처리.

**적용성**:
| 항목 | 값 | 평가 |
|---|---|---|
| Pi5 CPU 추론 (5M param, STFT block 1024) | ~30~80ms/block | ❌ budget 100× 초과 |
| Tiny variant (32k param) | ~2~5ms/block | ⚠️ period=32(0.33ms) 불가, period=512(5.3ms) 가능 |
| 학습 데이터 | 수십 시간 코골이 필요 | 사용자 없음 |
| 일반화 | 화자 의존성 큼 | 가족용 1인 한정이면 OK |

**판정**: ❌ 본 시스템 직접 적용 불가. period 큰 모드(>=256) + 사전 학습된 모델 + NEON 최적화 시에만 가능성.

**참고**: Zhang, H. & Wang, D. (2021). "Deep ANC: A Deep Learning Approach to Active Noise Control." *Neural Networks* 141, 1-10.

---

## 12.2 FxNLMS + DNN 하이브리드

**원리**: DNN이 FxLMS의 step size 또는 reference 예측을 보조. DNN inference는 저주파(수 Hz~수십 Hz)로 호출.

**변형 A — DNN learning rate scheduler**:
- 입력: 최근 100ms err RMS, conf, f0, weight norm
- 출력: μ_t (per-harmonic)
- 모델: 3-layer MLP, ~5k param
- 호출 주기: 10~50ms (period당이 아님)
- Pi5 inference cost: ~0.05~0.2ms (NEON dot product)

**변형 B — DNN predictor-as-reference**:
- 입력: error mic 최근 N 샘플
- 출력: 다음 P 샘플 예측 (P > sec_path delay)
- 모델: TCN 또는 LSTM, ~50k param
- ref_lead 인위 생성 → broadband 부활 가능 (§07 §7.4와 유사)

**적용성**: 변형 A는 ★★ (장기 카드, 데이터 수집 후), 변형 B는 ★ (구현 복잡, 학습 데이터 필요).

---

## 12.3 Tiny RNN / LSTM (Embedded)

**원리**: 32~128 cell 단일층 LSTM/GRU으로 f0 또는 envelope 예측. quantized int8 추론.

**도구**:
- TensorFlow Lite Micro (C++, 임베디드용)
- CMSIS-NN (ARM 공식, NEON 활용)
- Custom NEON kernels (가장 가벼움, 본 코드 정책 부합)

**적용 예 — f0 trajectory smoother**:
- 입력: 최근 20개 f0 후보 + conf
- 출력: 다음 f0 추정 + 변화율
- 모델: GRU 32 cell, ~5k param
- 추론: 50ms 주기, NEON dot product → 약 0.1ms
- 효과 예상: f0 jitter ↓, dip 완화. §08 Kalman과 유사하지만 비선형 burst 패턴 학습.

**판정**: ★★ (Phase 3+). Kalman을 §08에서 먼저 시도 → 부족하면 GRU.

---

## 12.4 Convolutional NN 이차 경로 식별

**원리**: 통계적 LMS 식별 대신 CNN이 sec_path 비선형 모델(스피커 왜곡 포함)을 학습.

**입력/출력**: input=백색잡음 driving signal, output=error mic response.

**모델**: 1D CNN 4-layer, kernel=64, ~10k param.

**장점**: 스피커 클리핑/IM 왜곡까지 포함하여 NLMS 발산 마진 ↑.

**현재 시스템 점수**: 본 시스템은 OUTPUT_LIMIT=0.35로 클리핑 회피 중이고 sec_path FIR이 합리적 동작 → ★ (필요성 낮음).

---

## 12.5 Reinforcement Learning (RL) ANC

**원리**: PPO/SAC agent가 mu/leak/n_harm을 시간별로 조정. Reward = -err_rms.

**문제점**:
- 학습에 수만 episode 필요 (실기에선 수십 시간)
- exploration 단계의 발산이 스피커 소음으로 직결 → §04 #5 silent safety 위반
- Bandit 형태로 단순화하면 응용 가능 (Card 단위 선택)

**판정**: ❌ 실기 학습 불가. 시뮬레이션 학습 후 transfer만 고려 가능. 우선순위 매우 낮음.

---

## 12.6 GAN 기반 noise generator

**원리**: GAN으로 코골이 다양성 합성 → 다른 알고리즘 학습/검증용 데이터 augmentation.

**용도**: 알고리즘 검증 dataset 확장. 본 시스템 inference에는 미사용.

**판정**: ★ (test pipeline 보조용으로만).

---

## 12.7 On-device 적응 학습 (Federated / Continual)

**원리**: 사용자별 패턴을 nightly 누적 학습. Edge에서 weight delta만 갱신.

**활용처**:
- 화자(사용자)별 f0 분포 학습 → §08 prior로 사용
- snore intensity threshold 개인화
- sec_path 시간 변화 추적 (§06 §6.7과 유사)

**판정**: ★ (장기 비전, Phase 5+).

---

## 12.8 본 시스템 ML 도입 의사결정 체크리스트

새 ML 기법 제안 시 다음 통과 필수:
1. **C/NEON 추론 가능?** Python/TF/PyTorch 런타임은 거절 (사용자 원칙)
2. **Period 0.333ms 또는 50~100ms 주기 호출 가능?** budget 초과면 거절
3. **사용자 데이터 ≥ 1시간 확보?** 없으면 합성 데이터로 PoC
4. **silent safety 보장?** RL은 본질적으로 안전성 미보장 → 거절
5. **§04 카탈로그 비교** — 단순 알고리즘으로 동등 효과 가능하면 ML 거절 (Occam)

---

## 12.9 우선순위

| 카드 | 비고 |
|---|---|
| FxNLMS+DNN scheduler (변형 A) | ★★ (Phase 3+, 단순한 ML) |
| Tiny GRU f0 smoother | ★★ (Kalman 부족 시) |
| Deep ANC (Zhang) end-to-end | ❌ (budget 미달) |
| DNN reference predictor | ★ (변형 B, 복잡) |
| RL ANC | ❌ (silent safety) |
| GAN data augmentation | ★ (검증 보조) |
| Federated/continual | ★ (장기) |

ML 본격 도입은 §22 Grand Matrix의 Phase 4 이후.
