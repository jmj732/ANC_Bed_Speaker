# 01. 이론

본 시스템에서 ANC를 결정하는 5개 식과 그 식이 깨지는 조건. 코드 위치는 모두 `src/anc_algo.h` 기준 (라인은 근사치, `nb_*` 함수명 우선 참조).

---

## 1.1 FxLMS (Filtered-x LMS) 기본식

| 기호 | 정의 |
|---|---|
| `x(n)` | reference 신호 (NB-FxLMS에선 내부 cos/sin 발진기) |
| `d(n)` | 1차 경로(primary)를 거친 disturbance, 에러 mic에서 측정 |
| `y(n)` | 안티-노이즈 출력, `y = w·x` |
| `s(n)` | secondary path 임펄스 응답 |
| `e(n)` | error mic 신호 = `d(n) − s(n)*y(n)` |
| `x'(n)` | filtered-x = `s(n)*x(n)` |

업데이트:
```
w(n+1) = leak · w(n) − μ · e(n) · x'(n)
```

본 코드는 NLMS 변형(아래 1.2)을 사용. → `nb_step()` 안의 `p->w_c`, `p->w_s` 업데이트 블록 (~line 655-664).

---

## 1.2 NB-FxLMS: 가상 reference

코골이는 협대역(준-주기) 신호. ref mic의 인과성 문제(ref_lead=0, §02)를 우회하기 위해 **내부에서 cos/sin을 생성**해 reference로 사용.

각 harmonic `h` (h=0이 fundamental):
```
phase += 2π · (h+1) · f0 / fs      ; nb_set_f0() & nb_step()
y_h    = w_c · cos(phase) + w_s · sin(phase)
```

filtered-x는 secondary path 응답 `S(f_h) = |S| ∠φ_s` 를 cos/sin에 곱해 만든다. 덧셈정리로 trig 호출을 최소화:
```
fx_cos = |S| · (cos_p · cos_sx − sin_p · sin_sx)    ; = |S|·cos(p+φ_s)
fx_sin = |S| · (sin_p · cos_sx + cos_p · sin_sx)    ; = |S|·sin(p+φ_s)
```
(`p->cos_sx`, `p->sin_sx` 는 `nb_set_f0()`에서 1회 사전계산. ~line 487-488)

코드 위치:
- `nb_eval_sec()` (~line 443-454): single-bin DFT로 `|S|`, `∠S` 추출
- `nb_set_f0()` (~line 480-502): f0 변경 시 phase_step, sx_mag, sx_phase, cos_sx, sin_sx, 그리고 large jump 시 weights reset
- `nb_step()` (~line 632-681): 샘플당 y, e, weight update

---

## 1.3 NLMS 정규화 + 안전장치

```
mu_n   = μ / (|S|² + ε)              ; ε = NB_NLMS_EPS = 3e-4 (anc_defs.h)
w_c   -= mu_n · e · fx_cos
w_s   -= mu_n · e · fx_sin
w_c, w_s ← leak · (w_c, w_s)         ; leak = 0.9999
‖w‖ ≤ 0.3 (clipped)                   ; weight cap
```

코드: `nb_step()` ~line 661-672.

### 안정성 조건 (이론)
- NLMS 일반: μ ∈ (0, 2). 본 시스템은 추가로 weight cap 때문에 cap에 도달하면 그래디언트가 잘려 위상이 어긋날 수 있음.
- **실험상 안전 마진**: `μ ≤ 0.001`. μ=0.003 시 발산 + FROZEN 반복(스피커 소음 발생) — 04 카탈로그 #5.
- leak < 1 이면 정상상태 weights는 disturbance 크기에 비례 축소되어 미세한 bias 존재. 실용상 무시 가능.

---

## 1.4 Pitch tracking (autocorrelation, 4× decimated)

f0 추정:
```
r(k) = Σ_i  buf[i] · buf[i+k]          ; k ∈ [lag_min, lag_max]
ρ(k) = r(k) / √(Σ buf[i]² · Σ buf[i+k]²)   ; 정규화
k*   = argmax_k ρ(k)
f0   = fs_dec / k* (포물선 보간 후)
```
- decimation: 3-tap 평균 LPF 후 4× → fs_dec = fs/4 = 24kHz
- lag 범위는 `NB_F0_MIN_HZ`, `NB_F0_MAX_HZ` (anc_defs.h)에서 환산
- 옥타브 모호성: r(k)와 r(2k)가 모두 큰 경우 → 후처리 가드(§1.5)

코드: `nb_detect_f0()` (~line 504-552), `nb_f0_update()` (~line 563-629).

### 검출 한계 (중요)
```
min_f0_detectable = fs_dec / (BUF_LEN_dec / 2)
                  = (fs / 4) / (BUF_LEN / 4 / 2)
                  = 2 · fs / BUF_LEN
```
@ fs=96kHz:
| BUF_LEN | min_f0 |
|---|---|
| 2048 | **94 Hz** ← 현재 |
| 4096 | 47 Hz |
| 8192 | 23 Hz |

> **함의:** `NB_F0_MIN_HZ` 매크로를 50Hz로 낮춰도 BUF_LEN=2048이면 검출 floor는 94Hz. 04 카탈로그 #9.

---

## 1.5 옥타브락 + hold

`nb_f0_update()` 내부에 두 단계 가드:

1. **옥타브락** (~line 594-596): `new_f0 / old_f0` 비율이 (1.7, 2.3)이면 0.5× 폴드, (0.43, 0.59)면 2× 폴드. conf<0.60 + 30% 점프는 거부.
2. **홀드** (~line 605-621): 15% 이내 2회 연속 후보 검출되어야 적용. 적용 후 weight reset 임계는 5% (위상 연속성 유지).

큰 점프 시 weights reset 임계(현재 15%, `nb_set_f0` ~line 497)와의 상호작용: §05 결정트리에서 다룬다.

---

## 1.6 인과성 (causality)

broadband FxLMS가 동작하려면:
```
ref_lead ≥ sec_path_delay
```
- ref_lead = ref mic이 disturbance를 error mic보다 먼저 감지하는 샘플 수
- sec_path_delay = anti-noise가 스피커→공기→error mic 도달하는 데 걸리는 샘플 수

본 시스템: ref mic이 스피커 옆 → **ref_lead = 0** (xcorr lag=0 모든 측정). 인과성 미충족 → broadband 불가. NB-FxLMS는 내부 발진기를 쓰므로 인과성 제약 우회.

→ 04 #1, 02 §HW 제약, 05 Card C.

---

## 1.7 핵심 식 요약 (한눈에)

| 항목 | 식 / 값 | 코드 위치 |
|---|---|---|
| y(n) | `Σ_h w_c·cos(p_h) + w_s·sin(p_h)` | nb_step ~644 |
| weight update | `w ← leak·w − (μ/|S|²)·e·fx` | nb_step ~655-664 |
| filtered-x | `|S|·exp(j(p+φ_s))` 의 실/허 | nb_step ~651-652 |
| f0 추정 | `argmax_k ρ(k)`, 4× decimated | nb_detect_f0 ~504 |
| 검출 floor | `2·fs / BUF_LEN` | (이론, anc_defs.h NB_F0_BUF_LEN) |
| weight cap | 0.3 | nb_step ~666 |
| leak | 0.9999 (NB_LEAK_DEFAULT) | anc_defs.h:229 |
| μ 안전상한 | **0.001** (실험) | 04 #4, #5 |

---

## Source

- 코드: `src/anc_algo.h`, `src/anc_defs.h`
- 측정/실험: §04 실패 카탈로그
- 관련 KB: §02 HW 제약, §03 신호 프로파일
