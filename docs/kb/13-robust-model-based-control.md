# §13 Robust / Model-Based 제어 기법

LMS 패밀리(§06)와 별개로, **모델 기반 제어 이론**에서 ANC에 적용되는 기법들. 본 시스템 제약(C, period=32, no QP solver, sec_path FIR 알려짐) 기준으로 평가.

---

## 13.1 H∞ Control for ANC

**원리**: secondary path 불확실성 ΔS(z)을 가정하고 최악 disturbance를 최소화하는 controller K(z) 설계. weighted sensitivity:
```
||W_s · S||∞ + ||W_u · K·S||∞ < γ
```

**구현**: MATLAB `hinfsyn` → controller K(z), 보통 IIR. 본 시스템에 옮기려면 FIR truncation 또는 SOS biquad cascade.

**장점**: sec_path drift / 화자 위치 변화에 robust.

**단점**: design-time 작업이며 online adaptive 아님. NB-FxLMS와 결합 시 controller weight를 H∞로 초기화 후 LMS로 미세 조정.

**판정**: ★ (Tier 4 카드, 오프라인 design 도구 필요). 본 시스템 narrowband에서는 §06 NLMS+leak이 충분.

---

## 13.2 LQG / MPC (Model Predictive Control)

**원리**: state-space 모델 + cost 최소화. MPC는 매 step QP 풀어 anti-noise 결정.

**Pi5에서**: 
- QP solver (e.g. OSQP) per period = 1~5ms → ❌ budget 100× 초과
- Explicit MPC (offline LUT) = 가능하나 narrowband 한정 시 LMS 대비 이점 적음

**판정**: ❌ 본 budget에서 불가.

---

## 13.3 Sliding Mode Control (SMC)

**원리**: state space에서 sliding surface s=0 위로 강제 수렴. Chattering 위험.

**ANC 응용**: Hu & Zhang (2015), Niedźwiecki. 빠른 수렴 + robust.

**문제**:
- Chattering이 스피커에서 high-freq noise로 출력될 위험 → §04 silent safety 위반 우려
- continuous SMC (super-twisting) 사용 필요, 구현 복잡

**판정**: ★ (research only).

---

## 13.4 Signed-LMS / Sign-Error / Sign-Sign

**원리**: 곱셈 대신 sign() 사용. 
- Sign-error: `w += μ · sign(e) · x_fx`
- Sign-data: `w += μ · e · sign(x_fx)`
- Sign-sign: `w += μ · sign(e) · sign(x_fx)`

**Pi5 비용**: NEON에서 곱셈도 단일 사이클 → 속도 이점 없음. **고정소수점 + 시프트 연산** 환경에선 의미.

**노이즈 특성**: impulsive noise(코골이 burst 시작)에 강함. err 폭주 시 step size 한정.

**판정**: ★★ (Phase 2 후보). 본 시스템 발산 마진 ↑ 가능. §16 fixed-point와 결합 시 시너지.

**참고 코드 매핑**: 현재 NLMS step (`anc_algo.h:660-664`)에 분기 추가:
```c
#ifdef NB_SIGN_ERROR
    p->w_c -= mu_n * copysignf(fx_cos, err);  /* err sign × fx_cos */
#else
    p->w_c -= mu_n * err * fx_cos;
#endif
```

---

## 13.5 Robust LMS — Median / Trimmed / Huber

**원리**: outlier(코골이 burst onset 시 큰 err)에 강한 통계량.

| 방법 | 식 | 비용 |
|---|---|---|
| Median-LMS | err 윈도우 median으로 대체 | sort N개 |
| Huber-LMS | |err|<δ면 quadratic, 이상이면 linear | 분기 1개 |
| Trimmed-LMS | err 최상위/최하위 X% 제거 | partial sort |

**Pi5**: Huber가 단순 (`fminf/fmaxf` 한 번). 비용 거의 0.

**판정**: ★★★ (Phase 1, 손쉬운 카드). burst onset dip 완화 기대. §04 #6 #7 (n_harm=2 발산)에도 도움.

**구현**: `nb_step`에 1줄 추가:
```c
float err_h = fmaxf(fminf(err, NB_HUBER_DELTA), -NB_HUBER_DELTA);
p->w_c -= mu_n * err_h * fx_cos;
```
`NB_HUBER_DELTA` ≈ 3× baseline err RMS (e.g., 0.03).

---

## 13.6 μ-synthesis / D-K iteration

**원리**: H∞의 일반화, structured uncertainty까지 처리.

**판정**: ❌ Pi5 inference 불가, design-time도 본 프로젝트 scope 초과.

---

## 13.7 Adaptive Robust Control (Slotine)

**원리**: parameter adaptation + sliding surface. ANC에 직접 적용 사례 적음.

**판정**: ★ (research).

---

## 13.8 Internal Model Principle (IMP)

**원리**: disturbance 모델을 controller 안에 내장. 본 시스템의 **NB-FxLMS 그 자체가 IMP의 특수 경우** (주기 신호 모델 = cos/sin 발진기).

**시사**: §07 §7.1 Feedback ANC와 결합 시 IMP-IMC controller로 일반화 가능. coupling이 잘 풀리면 broadband 부활 효과.

**참고**: Bodson, M. & Douglas, S. (1997). "Adaptive algorithms for the rejection of sinusoidal disturbances..." *Automatica* 33, 2213-2221.

**판정**: ★★ (§07 §7.1과 결합 검토).

---

## 13.9 Lyapunov 안정성 분석

**원리**: V(w,e) > 0, dV/dt ≤ 0 증명으로 발산 마진 결정.

**활용**: 본 시스템 mu 상한 도출 — 이론적으로 `0 < μ < 2/(λ_max(R))`, R은 reference autocorrelation. 본 시스템 NLMS는 `μ < 2/(sx_mag² + ε)` 자동 정규화.

**시사**: §04 #5 mu=0.003 발산은 NLMS 조건 만족하지만 **f0 변화 시 reference가 시변(non-stationary)**이어서 정적 분석 부족. Lyapunov-based gain scheduling이 더 안전.

**판정**: ★★ (이론적 가드, 새 mu 변경 시 사전 검증).

---

## 13.10 우선순위

| 카드 | 점수 | 즉시? |
|---|---|---|
| Huber-LMS (§13.5) | ★★★ | Phase 1, 코드 1줄 |
| Sign-error LMS (§13.4) | ★★ | Phase 2 (fixed-point와 결합) |
| IMP + Feedback ANC (§13.8) | ★★ | §07 §7.1과 묶음 |
| Lyapunov 사전 검증 (§13.9) | ★★ | 새 mu 후보 검토 시 |
| H∞ (§13.1) | ★ | 오프라인 설계 도구 |
| SMC / MPC / μ-syn | ❌ | budget/안전성 |

Phase 1 즉시 카드: **Huber-LMS** — 코드 1줄, 발산 마진 큰 폭 ↑.
