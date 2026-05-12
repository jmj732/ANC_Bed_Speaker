# 02. HW / 시스템 제약

"바꿀 수 있는 것"과 "바꿀 수 없는 것"을 분리한다. SW에서 갈아탈 수 있는 영역의 한계를 알면 의미 없는 시도가 줄어든다.

---

## 2.1 하드웨어

| 부품 | 사양 / 모델 |
|---|---|
| SBC | Raspberry Pi 5 |
| DAC/ADC | HiFiBerry DAC2 ADC Pro (40핀 직결) |
| 앰프 | TPA3116D2 |
| 스피커 | 삼미 CW-100B25 |
| 전원 | CEDC-65-6012 (12V 5A) |
| 케이블 | 3.5mm AUX → 2RCA |
| 마이크 | MAX4466 ×2 (**L = error mic, R = reference mic**) |

ALSA device: `hw:sndrpihifiberry,0` (Makefile `DEVICE` 매크로).

---

## 2.2 ALSA 설정 (실측 동작점)

| 항목 | 값 |
|---|---|
| sample rate `fs` | 96000 Hz |
| period | **32** 샘플 |
| buffer | 384 (= 12 periods) |
| channels | 2 (interleaved int16) |
| linked | yes |
| RT priority | SCHED_FIFO 49 |
| CPU affinity | CPU3 (anc-rt가 pinned) |
| `cpu_dma_latency` | 0 (C-states 비활성) |

### Budget
```
period_budget_ms = 32 / 96000 = 0.333 ms
```
이 시간 안에 [read → DSP → write]가 끝나야 xrun 없음.

---

## 2.3 Secondary path 측정 이력

`memory/project_anc_latency_goals.md` 출처, 4/9 측정.

| period | output safety LPF | sec_path peak (sample) | total (ms) |
|---|---|---|---|
| 128 | 400 Hz | 346 | 7.21 |
| 64 | 400 Hz | 217 | 4.52 |
| 32 | 800 Hz | **142** | **2.96** ← 현재 동작점 |

### period=128 기준 분해
| 구성 요소 | 샘플 | 시간 |
|---|---|---|
| ALSA capture alignment | ~128 | 2.67 ms |
| ALSA playback alignment | ~128 | 2.67 ms |
| Output safety LPF (2×400Hz IIR) | ~36 | 0.75 ms |
| DAC LL IIR | ~17 | 0.35 ms |
| Amp + speaker + air + mic + ADC | ~37 | 0.77 ms |

> SW로 줄일 수 있는 한계: period=32 → 2.96 ms. 이 이상은 **error mic ↔ 스피커 거리 단축(물리)** 만 가능.

---

## 2.4 Compute budget allocation (5/12 측정)

샘플당 / 주기당:

| 작업 | 평균 (ms) | 최대 (ms) | 비고 |
|---|---|---|---|
| ALSA read (blocking) | 0.328 | 0.36 | budget의 ~98% |
| DSP per-sample 합산 | 0.002~0.008 | 0.083 | period당 32 sample |
| ALSA write | 0.004 | 0.012 | |
| autocorr (NB_F0_BUF_LEN=2048) | **0.08** | 0.083 | update 시점만 |
| autocorr (BUF_LEN=4096) | ~0.32 | > budget | 분산 없이는 1주기에 집중 |
| autocorr (BUF_LEN=8192) | ~1.0 | **1.0** | xrun 폭주 (실측 219/12s) |

> **autocorr 비용 ∝ BUF_LEN²** (이중 루프). 4096 이상은 **여러 period에 분산** 또는 **별도 스레드** 필요.

---

## 2.5 ref_lead = 0 (인과성 미충족)

- xcorr lag = 0 (모든 측정에서 동일)
- ref mic이 스피커 옆에 있어 disturbance가 ref mic과 error mic에 거의 동시 도달
- broadband FxLMS는 인과성(`ref_lead ≥ sec_path_delay`) 필요 → 본 시스템은 **SW 해결 불가**
- 우회: NB-FxLMS는 내부 cos/sin 발진기를 reference로 써서 인과성 제약 회피 (§01 §1.6)
- 본 해결: **ref mic 위치 변경** (스피커 반대편 / 환자 머리쪽). → 05 Card C.

---

## 2.6 변경 가능 / 불가 분류

| 항목 | 가능? | 위험 / 비용 | 참조 |
|---|---|---|---|
| `μ` (NB_MU_DEFAULT 또는 `--nb-mu`) | ✅ CLI | μ>0.001은 발산 위험 | §01 §1.3, §04 #4 #5 |
| `leak` (NB_LEAK_DEFAULT 또는 `--nb-leak`) | ✅ CLI | 1.0 근접 시 누적 bias | §01 §1.3 |
| `n_harm` (NB_MAX_HARM 또는 `--n-harm`) | ✅ CLI | n>1 시 mu_scale 조정 필수 | §04 #6 #7 |
| `NB_F0_MIN/MAX_HZ` | ✅ 매크로 | BUF_LEN floor가 우선 | §04 #9 |
| `NB_F0_CONF_THR` | ✅ 매크로 | 너무 낮으면 silent safety 깨짐 | feedback_anc_noise.md |
| `NB_F0_UPDATE_SAMPLES` | ✅ 매크로 | 빠른 추적 vs autocorr 빈도 | §04 #12 |
| `NB_F0_BUF_LEN` | ⚠️ 매크로 | 4096+는 autocorr 분산 없이 불가 | §04 #2 #3 |
| `period`, `buffer-mult` | ⚠️ CLI | sec_path 재측정 필요 | §02 §2.3 |
| `OUTPUT_SAFETY_LPF_HZ` | ⚠️ 빌드플래그 | sec_path 변경 → 재측정 | §02 §2.3 |
| sec_path 길이 (스피커-err mic 거리) | ❌ 물리 | 침대/스피커 재배치 | §05 Card C |
| `ref_lead` | ❌ 물리 | ref mic 위치 변경 | §05 Card C |
| 부품 교체 (스피커, 앰프 등) | ❌ (원칙) | feedback_anc_style.md 금지 | — |

---

## 2.7 빌드

```bash
cd /home/admin/anc-rt
make                              # 보통 빌드
make clean && make                # 매크로 변경 시 필수 (의존성 추적 안 됨, §04 #11)
```
Makefile은 `OPTFLAGS ?= -O3 -mcpu=cortex-a76` 사용. 매크로 임시 오버라이드:
```bash
make clean && OPTFLAGS="-O3 -mcpu=cortex-a76 -DBASELINE_SECS=8" make
```

---

## Source

- 측정: `memory/project_anc_latency_goals.md`, `memory/project_anc.md`
- 설정: `src/anc_defs.h`, `Makefile`
- 관련 KB: §01 검출 floor, §04 카탈로그, §05 Tier 분류
