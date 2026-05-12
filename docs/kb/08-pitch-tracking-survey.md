# §08 피치 추적 서베이 — f0 추정 대안

> 본 시스템: 4× 데시메이션 자기상관 (`anc_algo.h:nb_detect_f0`, line 504-552).
> 한계: 옥타브 슬립, BUF_LEN 의존 floor (`§02 §4`, `§01 §6`), burst간 f0 점프 시 weight reset.
> 본 절은 대안 알고리즘 및 안정화 기법.

---

## A. 시간 도메인 (Time-Domain)

### A1. ACF (Autocorrelation) — **현재 사용 중**
- 식: `r(k) = Σ x(n)·x(n+k) / (||·|| · ||·||)` (normalized)
- 코드: `nb_detect_f0` (anc_algo.h:504)
- 비용: O(N·lag_range)
- **장점**: 단순, robust to phase
- **단점**: 옥타브 모호성, BUF_LEN-derived floor (`fs/(BUF/2)`)

### A2. YIN Algorithm
- 식:
  - `d(k) = Σ (x(n) - x(n+k))²` (squared difference)
  - `d'(k) = d(k) / [(1/k) · Σ_{j=1..k} d(j)]` (cumulative mean normalized)
  - 첫 local minimum with `d'(k) < threshold`
- 원리: ACF의 옥타브 모호성을 cumulative normalization으로 해결
- **본 시스템 적용성**: ✅ Tier 2
- **이득**: 옥타브 슬립 빈도 -50%~-80% (사례 보고)
- **비용**: ACF와 동급 (squared diff도 N·lag)
- **함정**: threshold 튜닝, 본 코골이 신호 미검증
- **참조**: de Cheveigné & Kawahara, 2002

### A3. pYIN (Probabilistic YIN)
- YIN + HMM smoothing over time
- **적용성**: ⚠️ Tier 3 — HMM forward-backward 추가, 본 system 무겁

### A4. AMDF (Average Magnitude Difference Function)
- 식: `D(k) = Σ |x(n) - x(n+k)|` (no square)
- **적용성**: ⚠️ Tier 1 — ACF의 곱셈을 abs로 대체, FP 비용 비슷
- **이득**: 적분 연산 회피 사례, 본 시스템 미미

### A5. SDF (Squared Difference Function)
- AMDF의 squared 버전, YIN의 1단계
- 적용성: A4와 동일

---

## B. 주파수 도메인 (Frequency-Domain)

### B1. Cepstrum
- 식: `c(τ) = IFFT(log|FFT(x)|²)`, peak in c at τ = pitch period
- **본 시스템 적용성**: ⚠️ Tier 3
- **비용**: 2× FFT (현 BUF_LEN=2048이면 2048-point FFT 2회)
- **차단**: FFT 인프라 없음, autocorr 이미 동급 성능

### B2. HPS (Harmonic Product Spectrum)
- 식: `H(f) = Π_{k=1..K} |X(k·f)|`
- 원리: f0의 정수배에 에너지 → 곱하면 f0 peak 강조
- **본 시스템 적용성**: ⚠️ Tier 2
- **장점**: 옥타브 robustness 우수
- **비용**: 1× FFT + 곱셈
- **이득**: 옥타브 슬립 거의 제거 (사례 보고)
- **차단**: FFT 비용, period budget

### B3. SHS (Subharmonic Summation)
- 식: `S(f) = Σ |X(k·f)| · h^(k-1)`, h=0.84 typical
- HPS의 sum version
- **적용성**: B2와 동일

### B4. SWIPE / SWIPE'
- 톱니파 weighting을 strength-normalized spectrum과 inner product
- **적용성**: ⚠️ Tier 3 — 복잡, ANC 적용 사례 적음

### B5. Sliding DFT
- 샘플당 1개 bin 갱신, recursive
- 식: `X_k(n) = (X_k(n-1) - x(n-N) + x(n))·e^{j2πk/N}`
- **본 시스템 적용성**: ✅ Tier 2
- **활용**: f0 직접 추정 대신 **harmonic 시리즈를 직접 트래킹**
- **이득**: nb의 oscillator를 sliding DFT bin으로 대체, sec_path 평가도 즉시
- **차단**: 본 nb 구조 재설계, 기대 이득 불확실

---

## C. 적응 노치 (Adaptive Notch Filter, ANF)

### C1. IIR ANF for f0
- 2차 IIR notch: `H(z) = (1 - 2·cos(ω₀)·z⁻¹ + z⁻²) / (1 - 2·ρ·cos(ω₀)·z⁻¹ + ρ²·z⁻²)`
- ω₀ adapt: notch 후 power minimize
- **본 시스템 적용성**: ✅ Tier 2
- **장점**: 샘플당 cheap, 별도 BUF_LEN 불필요 → §02 §4 floor 한계 회피
- **이득**:
  - 50Hz 이하 burst (median 70Hz) 검출 가능 → §03 §2와 정합
  - 옥타브 슬립 없음 (continuous tracking)
- **단점**: 초기 수렴 ~수십 ms, 다중 톤 시 trap
- **참조**: Nehorai & Porat, IEEE TSP 1986

### C2. Cascade ANF (Multi-Tonal)
- 직렬 ANF 다수, 각 다른 freq
- 적용성: ✅ Tier 3

### C3. LMS-ANF
- ω₀를 LMS로 직접 갱신
- 적용성: ✅ Tier 2

### C4. Constrained Pole-Zero ANF
- 극점/영점을 unit circle 제약 하에 adapt
- 적용성: ⚠️ Tier 3

---

## D. 통계 / 베이지안 (Statistical & Bayesian)

### D1. Kalman Filter f0 Smoother — **추천**
- state: `x_k = [f0, df0/dt]ᵀ`
- transition: `x_{k+1} = F·x_k + w`, `F = [[1, T],[0,1]]`
- observation: `z_k = f0_meas + v` (ACF/YIN 출력)
- **본 시스템 적용성**: ✅ Tier 2
- **이득**:
  - f0 jitter -50%
  - burst 사이 f0 점프를 smooth → dip 구간 -3dB → -1.5dB 예상
- **비용**: 2×2 matrix ops per update (저비용)
- **차단**: process noise Q 튜닝
- **결합**: §07 D2와 동일 아이디어

### D2. Particle Filter
- 비선형/비가우시안 f0 동역학
- **적용성**: ⚠️ Tier 3 — 비용 무거움, 본 시스템 이득 불명

### D3. HMM f0 Tracker
- discrete f0 grid + transition probabilities
- 적용성: ⚠️ Tier 3 (pYIN과 등가)

### D4. Bayesian Online f0
- prior · likelihood update
- 적용성: ⚠️ Tier 3

---

## E. 결합 / 후처리 (Smoothing & Post-Processing)

### E1. Median Filter on f0 stream
- N개 윈도우의 median
- **본 시스템 적용성**: ✅ Tier 1 — 코드 한 줄 추가
- **이득**: 옥타브 슬립 outlier 제거
- **차단**: latency N·NB_F0_UPDATE_SAMPLES (4800)

### E2. Hysteresis
- f0 변경 임계를 방향별 다르게 (rising vs falling)
- **적용성**: ✅ Tier 1
- **현 구현 일부**: `anc_algo.h:585-596` (15% hold + octave-lock)
- **확장**: 임계를 conf에 비례

### E3. Confidence-Weighted Average
- f0 = Σ conf_i · f0_i / Σ conf_i over window
- 적용성: ✅ Tier 1

### E4. Vibrato Compensation
- f0가 sinusoidal modulation 있으면 mean + envelope 분리
- **적용성**: ⚠️ — 코골이 vibrato 없음, 무의미

---

## F. 다중 가설 (Multi-Hypothesis)

### F1. 다중 lag candidate 동시 추적
- top-N peaks of ACF/YIN → each as candidate
- best가 burst마다 다를 수 있음
- **본 시스템 적용성**: ⚠️ Tier 3

### F2. 옥타브 ambiguity Voting
- 현재 KB에 부분 구현: ratio 1.7-2.3 / 0.43-0.59 → 폴드 (`nb_f0_update` line 594)
- 확장: 1.5×, 3× 등 추가 ratio

---

## G. 음성 처리 도메인 기법 (Speech-Origin)

### G1. RAPT (Talkin)
- ACF 변형 + dynamic programming
- 적용성: ⚠️ Tier 3

### G2. PEFAC
- spectral peak + log-spectrum filtering
- 적용성: ⚠️ Tier 3

### G3. CREPE (CNN 기반)
- 적용성: ❌ — DL inference 무거움

---

## 본 절 종합 — 다음 카드 우선순위

| 순위 | 기법 | Tier | 해결 영역 | 예상 이득 |
|---|---|---|---|---|
| 1 | **D1. Kalman f0 smoother** | 2 | burst간 f0 점프 dip | dip -3 → -1.5dB |
| 2 | **C1. IIR ANF** | 2 | 저주파(50Hz 이하) 검출, 옥타브 슬립 | best +1~2dB, 옥타브 슬립 -90% |
| 3 | **A2. YIN** | 2 | 옥타브 슬립 | 슬립 -50% |
| 4 | **E1+E2+E3. f0 후처리 (median+hyst+conf weight)** | 1 | jitter | 즉시 적용 가능, ±0.5dB |
| 5 | **B2. HPS** | 3 | 옥타브 robustness 최강 | 슬립 거의 0 |
| 6 | **B5. Sliding DFT 재구조화** | 3 | 전체 nb 아키텍처 | 미검증, 이득 불확실 |
| 7 | **C2. Multi-tonal ANF** | 3 | 코골이 multi-peak | §09와 결합 |

**즉시 시도 후보**: E1+E2+E3 (Tier 1, 1 commit) → D1 (Tier 2) → C1 (Tier 2 별도 트랙).
