# §09 코골이 도메인 특화 기법

> §03 신호 프로파일 기반. 본 시스템의 **target = 코골이**이므로 일반 ANC와 다른 정책 가능.
> 코골이 구조적 특성: burst (1~3초) + pause (0.5~3초), burst 내 f0 ramp, 다중 고조파.

---

## A. 활성 구간 검출 (Activity Detection)

### A1. f0_active flag — **현재 부분 구현**
- 코드: `anc_nb.c:131` (`f0_active = (nb.f0_hz > 0.0f && nb.f0_conf >= NB_F0_CONF_THR)`)
- 정책: SILENT 시 baseline/best/divergence 계산 skip
- **확장 후보**:
  - energy threshold 추가 (`||x|| > E_min`)
  - hysteresis (활성 진입/이탈 임계 차등)
  - hold time (1 frame high conf → adapt 시작 금지)

### A2. Snore Voice Activity Detector (SVAD)
- 일반 VAD가 아닌 코골이 전용
- features: spectral centroid, spectral flatness, zero-crossing rate, periodicity score
- 결정: heuristic threshold or 2-3 feature linear classifier
- **본 시스템 적용성**: ✅ Tier 2
- **이득**:
  - false adapt 제거 (말소리, 옷 stuck, 호흡 등)
  - dip 구간 안정성 ↑
- **비용**: 샘플당 cheap (running stat)

### A3. Burst Onset Detection
- 에너지 envelope의 derivative
- onset 직후 mu↑ (warm start) — §06 B1 (VSS) 연계
- **본 시스템 적용성**: ✅ Tier 2

### A4. Burst Offset Detection
- 에너지 envelope decay
- offset 시 weights freeze (현 페이드아웃 메커니즘 `anc_algo.h:627-632`와 결합)
- **본 시스템 적용성**: ✅ Tier 2

---

## B. Burst-Aware 적응 정책 (Burst-Aware Adaptation)

### B1. Phase-Aware Mu Schedule
| Burst 단계 | mu 정책 | 이유 |
|---|---|---|
| Pre-burst (silent) | mu = 0, weights frozen | false adapt 방지 |
| Onset (0~200ms) | mu ↑ (×3~5) | 빠른 수렴, dip 단축 |
| Steady (200ms~end) | mu = baseline | misadjustment ↓ |
| Offset / decay | mu = 0, leak 강화 | 잔향 제거 |
- **본 시스템 적용성**: ✅ Tier 2
- **이득**: dip -3dB → -1dB 가능 (§06 B1 VSS와 시너지)
- **함정**: onset 오검출 시 mu↑로 인한 발산 위험. §04 #5 회피 게이트 필수.

### B2. Inter-Burst Weight Persistence
- burst 종료 시 weights freeze (`out_env` 페이드 후)
- 다음 burst 시작 시: 직전 burst의 weights를 초기값으로 (reset 대신 warm start)
- **현 코드 대비 변화**: `nb_set_f0` (anc_algo.h:480-502)의 30% reset 임계를 조건부로
- **본 시스템 적용성**: ✅ Tier 2
- **이득**: onset dip 단축 (수렴 0 → 직전 burst 위치)
- **차단**: 두 burst의 f0가 너무 다르면 오히려 해로움 — f0 거리 게이트 필수

### B3. Inter-Burst Sec-Path Compensation
- 직전 burst f0 weights를 새 f0의 sec_path 위상 차이로 회전
- 5/12 시도한 **phase rotation (A안)**, 폐기. `§04 #8` 참조.
- 재시도 시: scale clamp 제거, phase만 회전. 검증 후 채택 여부 결정.

---

## C. 다중 톤 / 고조파 추적 (Multi-Tonal Tracking)

### C1. n_harm 확장 (현재 1)
- 시도 이력: `§04 #6, #7` — 단순 확장은 dip 악화
- 원인: h≥1 weights가 burst마다 다른 에너지 분포에 잘못 학습

### C2. Dynamic mu_scale (SNR-based)
- 각 harmonic 별 instantaneous SNR 추정
- SNR 높은 harmonic만 학습
- 식: `mu_scale[h] = clip(SNR_h / SNR_h_max, 0, 1)`
- SNR 추정: sliding DFT bin energy / total energy 등
- **본 시스템 적용성**: ✅ Tier 3
- **이득**: n_harm=2 실효화, best +1~2dB
- **차단**: SNR 추정 정확도가 핵심, 미세 조정 필요
- **현 mu_scale 인프라**: `anc_algo.h:471-477` (b18cf63 commit) — 동적화하면 됨

### C3. 독립 톤 추적 (Non-Harmonic Multi-Tonal)
- f0와 무관하게 강한 peak 2~3개를 독립 추적
- 코골이 spectrogram dominant 평균 434Hz가 fundamental 70Hz의 정확한 정수배 아닐 수 있음 (formant)
- **본 시스템 적용성**: ✅ Tier 3
- **이득**: 본 시스템 천장 +5dB 깰 잠재력
- **구현**: §08 C2 (Cascade ANF) 또는 §08 B5 (Sliding DFT) 기반

### C4. Formant-Aware Tracking
- 코골이 formant 구조 (구강/비강 resonance)
- F1 (~500Hz), F2 (~1500Hz) 추적
- **본 시스템 적용성**: ⚠️ Tier 3 — formant 추정 인프라 큼

### C5. Period-Period Pitch Ramp 보상
- burst 내에서 f0가 monotonically 변화 (rising or falling)
- LP/polynomial fit으로 f0(t) trajectory 예측
- **본 시스템 적용성**: ✅ Tier 3
- **이득**: f0 lag 보상 → dip ↓

---

## D. 신호 모델 (Signal Modeling)

### D1. Periodic + Aperiodic Decomposition
- 코골이 = 주기 성분 + 난류 (turbulence)
- 주기 성분만 cancel, 난류는 baseline
- **본 시스템 적용성**: ⚠️ — 현 nb가 사실상 이렇게 동작
- **확장**: aperiodic 성분 spectral subtraction (헤드폰 모드만 가능, 자유공간 불가)

### D2. Source-Filter Model
- glottal source (반복 펄스) + vocal tract filter (formant)
- ANC: source 부분만 cancel
- **본 시스템 적용성**: ⚠️ Tier 3 — 모델 식별 어려움

### D3. Snore Template Bank
- pre-recorded snore templates
- 매칭된 template의 anti-noise 적용
- **본 시스템 적용성**: ❌ — 일반화 안 됨, 사용자별 캘리브레이션 필요

### D4. Statistical Snore Model
- HMM/GMM으로 snore state 모델
- 적용성: ⚠️ Tier 3

---

## E. 시스템 정책 (System-Level)

### E1. Coarse Spectral Gate
- 80-300Hz 대역 밖 출력 0
- 현 OUTPUT_SAFETY_LPF 800Hz보다 더 좁게
- **본 시스템 적용성**: ✅ Tier 1
- **이득**: 안전 (코골이 외 톤 발진 방지)
- **차단**: high-harmonic cancel 능력 손실

### E2. Watchdog Reset
- 일정 시간 best_db < 0 지속 시 weights 0 reset + 학습 일시 중지
- 발산 누적 방지
- **본 시스템 적용성**: ✅ Tier 2
- **현 divergence reset**과 유사 (`anc_nb.c` 라인 약 243-)

### E3. User Calibration Mode
- 새 환경에서 5~10분 학습 후 best 파라미터 저장
- 적용성: ⚠️ Tier 3

### E4. Long-Term Memory
- 사용자 코골이 패턴 누적
- 적용성: ⚠️ Tier 3 — privacy 고려

---

## F. 평가 / 측정 정책 (Measurement Methodology)

### F1. 다중 신호 평균
- 단일 mollyroselee 8s만이 아닌 29개 신호 평균
- 코드: `--snore-file=` 옵션 확장하여 directory 지원
- **본 시스템 적용성**: ✅ Tier 1~2
- **이득**: burst 정렬 lottery 제거 (§04 #10 advisor 지적)

### F2. Per-Burst Analysis
- 단순 dB 평균이 아닌 burst별 dB 분포
- p10/median/p90, dip의 max
- **본 시스템 적용성**: ✅ Tier 1
- **현 logger 확장**

### F3. Listening Test
- 객관적 dB 외 주관적 인지 (loudness, sharpness)
- 적용성: ⚠️ — 사용자 시간 의존

---

## 본 절 종합 — 다음 카드 우선순위

| 순위 | 기법 | Tier | 영역 | 예상 이득 |
|---|---|---|---|---|
| 1 | **B1. Phase-Aware Mu Schedule (Burst-aware)** | 2 | dip 단축 | dip -3 → -1dB |
| 2 | **A2. Snore VAD** | 2 | false adapt 제거 | 안전성 ↑, +0.5dB |
| 3 | **F1. 다중 신호 평균 측정** | 1 | 측정 노이즈 제거 | 측정 신뢰도 (튜닝 효율) |
| 4 | **C2. Dynamic mu_scale (SNR-based)** | 3 | n_harm≥2 실효화 | best +1~2dB |
| 5 | **B2. Inter-Burst Weight Persistence** | 2 | onset dip | onset 200ms 단축 |
| 6 | **C3. 독립 톤 추적** | 3 | formant 영역 cancel | best +2~4dB 잠재력 |
| 7 | **C5. Pitch Ramp 보상** | 3 | burst 내 f0 lag | 미세 |

**즉시 시도**: F1 (측정 신뢰 회복, 5/12 advisor 지적) → B1+A2 (burst-aware 패키지).
