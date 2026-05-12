# §06 적응 알고리즘 서베이 — LMS 패밀리 및 변형

> 본 시스템(narrowband FxLMS + autocorr pitch tracker) 외에 시도 가능한 적응 필터 알고리즘.
> 각 항목 끝의 **적용성**은 ref_lead=0, period=32 budget=0.333ms, C 단일파일, Pi5 ARM Cortex-A76 기준.

---

## A. 정규화/누설 변형 (Normalization & Leakage)

### A1. LMS (기본형)
- 식: `w(n+1) = w(n) - μ·e(n)·x(n)`
- 본 시스템 미사용. NLMS가 표준.

### A2. NLMS (Normalized LMS) — **현재 사용 중**
- 식: `μ_n = μ / (||x||² + ε)`
- 코드: `anc_algo.h:661` (`mu_n = nb->mu / power`, `power = sx_mag² + NB_NLMS_EPS`)
- 장점: 입력 레벨 변화에 강건
- 단점: 분모 추정 노이즈 → μ_n 튐 (NB_NLMS_EPS로 완화)

### A3. Leaky LMS — **현재 사용 중**
- 식: `w(n+1) = (1-α)·w(n) - μ·e(n)·x(n)`
- 본 시스템: `leak = 0.9999` (`anc_algo.h:665-666`)
- 장점: weight runaway 방지, DC drift 억제
- 단점: 무한 SNR에서도 손실 (steady-state misadjustment ↑)

### A4. Regularized NLMS
- 식: `μ_n = μ / (||x||² + δ·σ²)`, δ는 SNR 의존 정규화
- **적용성**: ⚠️ Tier 2 — SNR 추정 추가, 본 NB_NLMS_EPS와 등가

---

## B. 가변 스텝 사이즈 (Variable Step Size, VSS)

### B1. Kwong VSS-LMS
- 식: `μ(n) = α·μ(n-1) + γ·e²(n)`, clamp `[μ_min, μ_max]`
- 원리: 큰 에러 → μ↑ (빠른 수렴), 작은 에러 → μ↓ (낮은 misadjustment)
- **적용성**: ✅ Tier 2
- **본 시스템 매핑**: burst 시작 시 μ 자동 ↑ → dip 구간 수렴 가속
- **예상 이득**: dip -3dB → -1dB (이론치, 미검증)
- **함정**: 코골이 burst 같은 비정상 신호에서 μ 폭주 — clamp 필수
- **연관**: §04 #5 (μ=0.003 발산)와 충돌 — μ_max는 0.001 이하 권장

### B2. NPVSS-LMS (Non-Parametric VSS)
- Benesty et al., μ가 잡음 분산 추정 기반
- 식: `μ(n) = μ_max · (1 - σ_v² / σ_e²)`
- **적용성**: ✅ Tier 2 — 본 시스템 baseline_err_rms 측정 인프라 재활용 가능
- **장점**: 파라미터 1개(μ_max)만 튜닝
- **이득**: B1과 동급, 더 robust

### B3. Robust VSS (RVSS)
- Outlier-resistant, e(n) 대신 sign 또는 Huber loss
- **적용성**: ⚠️ Tier 2 — 본 NLMS에 sign 혼합 가능

### B4. Mathews-Xie VSS
- `μ(n+1) = μ(n) + ρ·∇e²(n)` (gradient on μ itself)
- **적용성**: ⚠️ Tier 2 — convergence 분석 복잡

---

## C. 고차 / 다중 입력 (Higher-Order Adaptive)

### C1. APA (Affine Projection Algorithm)
- N×P 입력 행렬, P=2~4 typical
- 식: `w(n+1) = w(n) - μ·X(n)·(X(n)ᵀX(n) + δI)⁻¹·e(n)`
- 원리: 과거 P개 입력 vector projection → 수렴 가속
- **적용성**: ⚠️ Tier 2
- **본 NB-FxLMS 한정**: harmonic당 N=2 이미 작아서 APA 추가 이득 미미
- **비용**: P×P 역행렬, period=32 budget 압박
- **broadband 재활성화 시**: ✅ — 효과 큼

### C2. FxAPA (Filtered-x APA)
- C1의 ANC 버전. sec_path 통과 후 적용.
- 적용성: C1과 동일

### C3. RLS (Recursive Least Squares)
- 식: `w(n+1) = w(n) + K(n)·e(n)`, K는 inverse covariance 갱신
- **장점**: 이론적 최적 수렴 (지수 가중 LSE)
- **비용**: O(N²) per sample
- **적용성**: ❌ — N=2 (per-harmonic)이면 가능하나 NB-FxLMS에서 이득 cosmetic. Broadband (N>100)는 budget 미충족.

### C4. FxRLS (Filtered-x RLS)
- C3의 ANC 버전
- **적용성**: ❌ same

### C5. QR-RLS / Fast RLS
- 수치 안정성↑, 비용 ↓
- **적용성**: ⚠️ — broadband 부활 후 검토

---

## D. 희소화 / 조건부 갱신 (Sparse & Conditional Update)

### D1. SM-NLMS (Set-Membership NLMS)
- 갱신 조건: `|e(n)| > γ` 일 때만
- 원리: error가 acceptable bound 이내면 갱신 skip
- **적용성**: ✅ Tier 2
- **본 시스템 적용 시점**: 코골이 burst 종료 후 (f0_active=0) compute 절감
- **이득**: dB 변화 없음, CPU 절감 → 다른 곳에 budget 활용

### D2. Sign-LMS / Sign-Error LMS
- 식: `w(n+1) = w(n) - μ·sign(e(n))·x(n)`
- 원리: 곱셈 제거 (HW 단순화)
- **적용성**: ⚠️ — ARM Cortex-A76 FP multiply 1 cycle, no benefit
- **단점**: 수렴 ↓, misadjustment ↑

### D3. Sign-Sign LMS
- 식: `w(n+1) = w(n) - μ·sign(e(n))·sign(x(n))`
- 적용성: D2와 동일

### D4. Proportionate NLMS (PNLMS)
- 각 tap의 weight 크기에 비례한 step
- 원리: sparse impulse response에 효율적
- **적용성**: ❌ — NB-FxLMS는 N=2로 sparse 개념 무의미

---

## E. 블록 / 주파수 도메인 (Block & Frequency-Domain)

> 상세는 §10 참조.

### E1. FDAF
- FFT 기반 block LMS
- 본 시스템 period=32 작아서 FFT overhead가 더 큼
- broadband 부활 시 검토

### E2. PBFDAF (Partitioned Block FDAF)
- 긴 sec_path(1024 taps)를 분할 FFT
- broadband 활성화 시 표준 선택

### E3. Block LMS (time-domain)
- N 샘플 모아서 1회 갱신
- **적용성**: ⚠️ — 본 nb_step은 이미 샘플당 cheap

---

## F. 비선형 / 함수 링크 (Nonlinear)

> 상세는 §10의 advanced 절 참조.

### F1. Volterra FxLMS
- 2차/3차 Volterra kernel
- 비선형 스피커/마이크 보정
- **적용성**: ⚠️ Tier 3 — 본 TPA3116D2는 D-class, 어느 정도 비선형
- **이득**: 본 시스템 미검증, 광대역 +1~3dB 사례 보고

### F2. FsLMS (Functional Link LMS)
- x를 비선형 함수(cos·sin, tanh)로 expand
- **적용성**: ⚠️ Tier 3

### F3. Bilinear FxLMS
- bilinear plant model
- **적용성**: ❌ — 본 sec_path FIR 모델로 충분

### F4. Neural-Network ANC
- CNN / RNN / Transformer
- **적용성**: ❌ — C 단일파일 / Pi5 budget 위배. 별도 추론 가속기 없으면 불가.

---

## G. Filtered-x 변형 (Filtered-x family)

### G1. FxLMS (표준) — **현재 사용 중**
- sec_path 통과 후 x를 reference로 사용
- 코드: `anc_algo.h:659-660` (덧셈 정리로 trig 1회)

### G2. Filtered-u (FuLMS) — IIR ANC
- ANC 컨트롤러를 IIR로
- 적은 tap으로 긴 impulse response
- **적용성**: ⚠️ Tier 3 — 안정성 어려움, sec_path FIR로 충분

### G3. Modified FxLMS (MFxLMS)
- Bjarnason의 sec_path 가중 LMS 변형
- 수렴 속도 ↑
- **적용성**: ⚠️ Tier 2 — 본 NB-FxLMS는 per-harmonic sec_path 평가로 등가 효과

### G4. Delayed FxLMS (DXLMS)
- sec_path 추정 오차 보상
- **적용성**: ⚠️ Tier 2

### G5. FxLMS with Online Secondary-Path Modeling
- 보조 잡음 주입으로 sec_path 실시간 추정
- **적용성**: ⚠️ Tier 3 — 본 시스템 sec_path 정적, 변화 적음. 침대 위치 바뀌면 검토.

---

## H. 안정성 / Constrain

### H1. Weight Cap — **현재 사용 중**
- `||w|| ≤ 0.3` (`anc_algo.h:667-672`)
- 발산 방지 최후 가드

### H2. Projection NLMS
- weight를 admissible set에 project
- weight cap의 정식 일반화

### H3. Stochastic Gradient with Lyapunov
- 수렴 보장 가능한 step size 도출
- 적용성: 이론 분석용

---

## 본 시스템 즉시 시도 후보 (이 파일 기준)

| 순위 | 기법 | Tier | 이유 |
|---|---|---|---|
| 1 | **VSS-LMS (B1 또는 B2)** | 2 | burst 수렴 dip 직접 공략. §04 #5 회피 가능 (μ_max clamp). |
| 2 | **SM-NLMS (D1)** | 2 | f0_active=0 구간 compute 절감 → §02 §4의 BUF_LEN=4096 budget 마진 확보 |
| 3 | **Modified FxLMS (G3)** | 2 | per-harmonic sec_path 가중치 정식화, 미세 수렴 개선 |
| 4 | **APA (C1)** | 2 | broadband 재활성화 시 |
| 5 | **Online sec_path (G5)** | 3 | 침대/배치 변경 후 정적 measure 의존도 ↓ |

**§05 결정트리 §D에 진입할 후보**: VSS-LMS, SM-NLMS.
