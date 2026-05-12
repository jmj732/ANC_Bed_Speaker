# §10 주파수 도메인 / 멀티레이트 / 비선형 기법

> 본 시스템은 시간 도메인 NB-FxLMS. 본 절은 **다른 도메인** 또는 **비선형 모델** 접근.
> 대부분 broadband 부활 (§07 F1 물리 배치) 또는 무거운 보조 인프라 전제.

---

## A. 블록 / 주파수 도메인 (Frequency-Domain Adaptive Filter)

### A1. FDAF (Frequency-Domain Adaptive Filter)
- 원리: N 샘플을 FFT → bin별 LMS → IFFT → output
- 식 (Overlap-Save):
  - `Y(k) = W(k) · X(k)` (per bin product)
  - `W(k) += μ_k · E(k) · X*(k)` (NLMS in freq)
- 비용: O(N·log N) per block vs O(N²) time-domain
- **본 시스템 적용성**: ⚠️ Tier 3
- **장점**: bin별 독립 mu (자동 multi-band)
- **단점**: period=32 작아서 FFT overhead가 sample-by-sample보다 큼
- **이득 시점**: broadband + 긴 sec_path 활성화 시
- **참조**: Shynk 1992, Frequency-Domain Adaptive Filtering

### A2. Constrained FDAF
- circular convolution 보정 (overlap-save)
- A1 표준 구현

### A3. PBFDAF (Partitioned Block FDAF)
- 긴 sec_path (현 1024 taps)을 K개 partition (각 N) FFT
- delay-line 처리, low-latency
- **본 시스템 적용성**: ✅ Tier 3 — broadband 재시도 시 표준 선택
- **이득**: 광대역 적응을 budget=0.333ms에 fit
- **참조**: Soo & Pang 1990

### A4. Multi-Delay Block FDAF (MDF)
- PBFDAF의 발전형, latency-throughput tradeoff
- 적용성: A3과 등가

### A5. RDFT-LMS (Real-DFT)
- complex FFT 대신 RDFT, 약 ½ 비용
- **적용성**: ⚠️ Tier 3 — broadband 부활 시

---

## B. 서브밴드 (Subband)

### B1. Subband FxLMS (M-band)
- 원리:
  1. 입력을 M개 대역으로 filter bank 분해
  2. 각 대역 downsample × M
  3. 대역별 독립 FxLMS
  4. 출력을 다시 filter bank로 합성
- **본 시스템 적용성**: ✅ Tier 3
- **장점**:
  - per-band sec_path 짧음 → ref_lead 조건 부분 완화 (band별 인과성)
  - per-band 수렴 빠름 (eigenvalue spread ↓)
  - per-band mu 차등 가능
- **단점**:
  - filter bank 설계 (QMF, cosine-modulated)
  - aliasing 보정
  - cross-band coupling
- **이득**: 광대역 +3~7dB 가능 (사례 보고), 단 본 시스템 ref_lead=0 한계 일부 남음
- **참조**: Morgan & Thi 1995

### B2. Adaptive Subband
- filter bank 자체를 학습
- 적용성: ⚠️ Tier 3 — 복잡

### B3. Wavelet Subband
- non-uniform 분해
- **적용성**: ⚠️ — 학술적, 실용 ANC 사례 적음

### B4. Polyphase Subband
- B1의 효율적 구현
- 적용성: B1과 동일

### B5. DCT Subband
- 적용성: ❌

---

## C. 변환 도메인 (Transform-Domain)

### C1. DCT-LMS
- input을 DCT 변환 후 LMS
- 적용성: ❌ — ANC 적용 사례 적음

### C2. DST-LMS
- 적용성: ❌

### C3. Hartley Transform LMS
- 적용성: ❌

### C4. Walsh-Hadamard LMS
- binary 입력 신호에 효율적
- 적용성: ❌ — 본 시스템 무관

---

## D. 비선형 / 함수 링크 (Nonlinear)

### D1. Volterra FxLMS
- 2차/3차 Volterra kernel: `y = Σ h_1(i)·x(i) + Σ h_2(i,j)·x(i)·x(j) + ...`
- 비선형 스피커/마이크 보정
- **본 시스템 적용성**: ⚠️ Tier 3
- **이득 가능성**: TPA3116D2 (D-class) 비선형 일부, +0.5~2dB
- **비용**: 2차 kernel은 O(N²) tap 수, prohibitive

### D2. FsLMS (Functional Link)
- x를 비선형 함수 expand: `[x, sin(x), cos(x), x², ...]`
- 선형 LMS를 expanded space에 적용
- **본 시스템 적용성**: ⚠️ Tier 3
- **참조**: Das & Panda 2004

### D3. Bilinear FxLMS
- bilinear plant: `y = a·u + b·u·d`
- 적용성: ❌ — 본 sec_path FIR로 충분

### D4. Neural-Network ANC
- CNN, RNN, Transformer 기반
- **본 시스템 적용성**: ❌
- **차단 요인**:
  - C 단일파일 + 표준 lib 위배
  - Pi5 추론 budget (period=32, 0.333ms) 위반
  - 학습 데이터 부재
  - 발산 시 안전성 분석 불가
- **이론적 이득**: 사례 보고 +10~20dB (헤드폰), 자유공간 미지수

### D5. Echo-State Network ANC
- RNN의 일종, sparse fixed
- 적용성: ❌ — 동일

### D6. Adaptive Lattice Filter
- 비선형 변형이라기보다 구조 변형
- 적용성: ⚠️ — 본 NB-FxLMS와 직교

---

## E. 멀티레이트 (Multirate)

### E1. Multirate FxLMS
- 입력 rate ≠ 출력 rate
- **본 시스템 적용성**: ⚠️ — 현 96kHz fixed, 변경 시 sec_path 재측정

### E2. Two-Stage Multirate
- 1단: low rate (subband FxLMS), 2단: high rate (output)
- 적용성: ⚠️ Tier 3 — 본 시스템 latency budget 분석 필요

### E3. Polyphase Decomposition
- E2의 구현 기법
- 적용성: ⚠️

### E4. Sample-Rate Conversion in Loop
- input/output 다른 rate
- 적용성: ❌ — HW 고정

---

## F. 직접 합성 (Direct Synthesis)

### F1. DXHS (Delayed-X Harmonic Synthesizer)
- 원리: f0와 sec_path 위상을 미리 알고 직접 anti-noise 합성
- LMS는 amplitude만 갱신
- **본 시스템 적용성**: ⚠️ Tier 2
- **차이점 vs 현 nb**: 현 nb는 이미 거의 등가 (cos/sin oscillator + sec_path phase pre-compute)
- **이득**: cosmetic
- **참조**: Hu & Hsieh

### F2. Phase-Locked Oscillator Bank
- 다중 oscillator 각자 phase-lock to harmonic
- §07 C3 PLL과 결합
- **본 시스템 적용성**: ✅ Tier 3

### F3. Cosine-Sine Generator Recursive
- `cos((n+1)θ) = 2cos(θ)·cos(nθ) - cos((n-1)θ)`
- 매 샘플 `sincosf` 호출 회피
- **본 시스템 적용성**: ✅ Tier 1
- **이득**: compute ↓ (5/12 측정 0.008ms에서 더 줄어듦)
- **차단**: 수치 drift (long-running), 주기적 normalize 필요

---

## G. 모델 예측 제어 (Model Predictive)

### G1. MPC ANC
- finite horizon prediction
- **본 시스템 적용성**: ⚠️ Tier 3 — 이론 무거움
- **이득**: 사례 보고 +2~5dB
- **차단**: 실시간 QP solver 필요

### G2. Receding Horizon ANC
- 적용성: G1과 등가

---

## H. 강건 제어 (Robust)

### H1. H∞ ANC
- worst-case 모델 불확실성
- **적용성**: ⚠️ Tier 3 — 본 sec_path 정적, 이득 미미

### H2. μ-Synthesis
- 구조적 불확실성
- 적용성: ❌

### H3. LPV (Linear Parameter Varying)
- sec_path가 환경 변수(위치, 온도)에 의존
- 적용성: ⚠️ Tier 3 — 사용자 위치 변할 때

---

## 본 절 종합 — 다음 카드 우선순위

| 순위 | 기법 | Tier | 전제 조건 | 예상 이득 |
|---|---|---|---|---|
| 1 | **F3. Recursive Cos-Sin** | 1 | 없음 | compute ↓ (즉시 적용) |
| 2 | **A3. PBFDAF** | 3 | 광대역 부활 (§07 F1) | broadband ANC 가능 |
| 3 | **B1. Subband FxLMS** | 3 | 광대역 부활 부분 | +3~7dB (광대역) |
| 4 | **D1. Volterra FxLMS** | 3 | nb 안정화 후 | +0.5~2dB (비선형 보정) |
| 5 | **G1. MPC ANC** | 3 | QP solver 인프라 | +2~5dB |
| 6 | **D2. FsLMS** | 3 | nb 안정화 후 | 미검증 |

**즉시 적용 가능**: F3 만. 나머지는 광대역 재활성화 또는 알고리즘 인프라 큰 추가 전제.

**중요 메모**: 본 절의 기법들은 대부분 ref_lead=0 천장을 깨지 못함. 본질적 해결은 **§07 (인과성 우회)** 또는 **§02 §6 (물리 배치)**. 본 절은 천장 깬 후 추가 튜닝 영역.
