# §14 Bayesian / 통계적 추론 기반 ANC

확률 모델로 f0 / weight / sec_path / state를 추정. 본 시스템 §08 Kalman은 시작점, 본 §14는 그 확장.

---

## 14.1 Wiener Filter (Frequency-Domain)

**원리**: optimal MMSE 추정. `H(f) = S_xd(f) / S_xx(f)`. ANC의 이론적 lower bound.

**용도**: 본 시스템 best_db 천장의 **이론적 한계 비교** 기준. mollyroselee 파일에 Wiener 계산하면 +X dB 가능한지 사전 확인.

**Pi5 inference**: FFT block 1024 ≈ 0.5ms (NEON FFT). period=32에선 sub-block 처리 필요.

**판정**: ★★★ (검증 도구). 알고리즘으로는 안 쓰더라도 **천장 추정 reference**로 1회 계산하면 §11 매트릭스 결정에 결정적.

**구현 안**: 오프라인 sim_compare에 Wiener bound 계산 추가.

---

## 14.2 Kalman 변형

§08 §8.4가 기본 Kalman f0 smoother. 변형:

| 변형 | 차이 | 효과 |
|---|---|---|
| **Extended KF (EKF)** | 비선형 f(state) 선형화 | f0 + amplitude 동시 추정 |
| **Unscented KF (UKF)** | sigma point 통과 | EKF보다 정확, 비용 ×5 |
| **Cubature KF (CKF)** | sigma point 대안 | UKF와 유사 |
| **Square-Root KF** | 수치 안정성 ↑ | float 환경에서 의미 |
| **IMM (Interacting Multiple Model)** | 여러 motion 모델 병렬 | burst on/off 같은 모드 전환 잘 잡음 |

**Pi5 cost** (state dim 2~4, 50ms 주기):
- 일반 KF: ~0.01ms
- EKF: ~0.02ms
- UKF: ~0.05ms

**판정**: 
- 기본 KF (§08 §8.4) ★★★
- IMM (burst on/off 명시 모델) ★★ — VAD(§09 §9.2)와 결합
- EKF/UKF ★ (필요성 낮음, narrowband는 선형 가정 충분)

---

## 14.3 Particle Filter (Sequential Monte Carlo)

**원리**: state posterior를 weighted samples (particles)로 표현. 비선형/non-Gaussian에 강함.

**ANC 응용**: f0 + harmonic amplitude jointly 추정 (Christensen et al., 2008).

**Pi5 비용**: N=100 particles, 50ms 주기, 추정 차원 4 → ~0.5ms (NEON으로 0.1ms 가능).

**판정**: ★★ (Kalman 부족 시 대안). f0 분포가 multi-modal일 때 (예: 두 사람 동시 코골이) 유용.

**참고**: Christensen, M.G. et al. (2008). "Sinusoidal order estimation using angles..." *EURASIP*.

---

## 14.4 HMM (Hidden Markov Model) for snore states

**원리**: 코골이 burst를 hidden state 시퀀스로 모델. State = {silent, inhale-onset, peak, exhale}.

**Observation**: err RMS, f0 conf, spectral centroid.

**Pi5 cost**: forward algorithm O(N²T), 4 state, 50ms 주기 → 무시할 만함.

**활용**: 
- adapt on/off scheduling (silent state엔 adapt 끄기)
- mu_scale gating (state별 다른 mu)
- §09 §9.4 burst onset detection을 HMM으로 통합

**판정**: ★★ (Phase 2 카드, VAD 강화).

---

## 14.5 GMM (Gaussian Mixture Model) for noise classification

**원리**: 마이크 입력을 GMM으로 분류 — {snore, breathing, fan, talking, silence}.

**구성**: MFCC 13차원 + ΔMFCC, K=5 컴포넌트.

**ANC 응용**: snore 클래스에서만 adapt ON. 다른 잡음은 mute.

**Pi5 cost**: MFCC ~0.2ms/frame (FFT 포함), GMM eval ~0.05ms.

**판정**: ★★ (Phase 3, 환경 잡음 거부). VAD(§09 §9.2)의 일반화.

---

## 14.6 Bayesian Online Learning

**원리**: posterior p(w | data)를 매 sample 갱신. Variational inference 또는 conjugate prior.

**ANC 적용**: weight uncertainty 추정 → 불확실 영역에서 step size ↓.

**구현**: Recursive Bayesian estimator = Kalman의 일반화. 본 시스템 weight 차원이 작아(=2n_harm) 가능.

**판정**: ★★ (Phase 3, §06 §6.5 RLS와 유사 효과).

---

## 14.7 Variational Bayes for sec_path drift

**원리**: sec_path를 시변 random variable로 두고 posterior 갱신.

**경쟁**: §06 §6.7 online sec_path identification (LMS).

**판정**: ★ (복잡도 ↑ 효과 ↓).

---

## 14.8 EM Algorithm for joint f0/amp estimation

**원리**: E-step에 f0 posterior, M-step에 amplitude/phase 갱신.

**참고**: Nielsen, J. K. et al. (2014). "Bayesian model comparison with the g-prior..." 

**Pi5**: 수렴 5~10 iter, 각 ~0.5ms → 50ms 주기로 OK.

**판정**: ★★ (n_harm=2+ 안정화의 통계적 대안).

---

## 14.9 우선순위

| 카드 | 점수 | 어디서 활용 |
|---|---|---|
| Wiener bound 계산 (§14.1) | ★★★ | **천장 검증** (Phase 0) |
| Kalman f0 smoother (§14.2) | ★★★ | §08과 동일, Phase 1 |
| IMM Kalman | ★★ | burst 모드 전환 |
| HMM scheduler | ★★ | adapt on/off Phase 2 |
| GMM noise classifier | ★★ | 환경 잡음 거부 Phase 3 |
| Particle filter | ★★ | KF 부족 시 |
| EM joint estimator | ★★ | n_harm=2+ 안정화 |
| VB sec_path | ★ | 복잡도 대비 효과 낮음 |

**최우선**: §14.1 Wiener bound. 이를 통해 **n_harm=1로 도달 가능한 이론 최대치**를 알면 §11/§22 카드 우선순위가 결정적으로 정리된다.
