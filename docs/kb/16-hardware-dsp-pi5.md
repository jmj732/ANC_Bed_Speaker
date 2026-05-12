# §16 Pi5 / ARM DSP 최적화 카드

본 시스템 budget 0.333ms / period 32 / fs 96kHz / Cortex-A76. SW 한도 내 latency·CPU 추가 마진 확보 카드.

---

## 16.1 NEON SIMD intrinsics

**현 상태**: gcc `-O3 -mcpu=cortex-a76`로 auto-vectorize 일부만. 명시적 NEON 미사용.

**핫스팟 (perf 프로파일 5/12):**
| 영역 | 비용 | NEON 잠재 절감 |
|---|---|---|
| autocorr inner loop (nb_detect_f0) | 50ms마다 ~0.08ms | 4× (float32x4) |
| sec_path FIR convolve | per sample | 4× (1024 tap → 0.02ms) |
| nb_step inner | per sample × n_harm | 2× (cos/sin 동시) |

**Intrinsics 예** (autocorr):
```c
#include <arm_neon.h>
float32x4_t v_sum = vdupq_n_f32(0.0f);
for (int i = 0; i + 4 <= n; i += 4) {
    float32x4_t a = vld1q_f32(&buf[i]);
    float32x4_t b = vld1q_f32(&buf[i + lag]);
    v_sum = vmlaq_f32(v_sum, a, b);
}
sum = vaddvq_f32(v_sum) + tail_scalar;
```

**예상 효과**: autocorr 0.08ms → 0.02ms → BUF_LEN=8192도 한 period에 가능 (~0.3ms).

**판정**: ★★★ (Phase 2 결정적 카드). BUF_LEN 확장 차단의 본질적 해법.

---

## 16.2 Fixed-Point Arithmetic (Q15 / Q31)

**원리**: float → int16/int32 변환. NEON 정수 SIMD는 float SIMD와 동등 속도, 일부 명령(`vqdmulh`) 더 빠름.

**문제**: NLMS의 mu_n = mu/(sx²+ε) 정밀도 손실 우려. saturation 처리 필요.

**ABI**: ALSA buffer는 int16 이미 사용 → float 변환 비용 제거 가능.

**판정**: ★★ (Phase 3, NEON float가 충분히 빠르면 우선순위 낮음).

---

## 16.3 ALSA mmap / direct DMA

**현 상태**: `snd_pcm_readi/writei` blocking 호출. read=0.33ms 평균 ← 사실 budget의 100%.

**대안**:
- `snd_pcm_mmap_readi/writei` — kernel copy 1회 절감
- `snd_pcm_avail_update` + 직접 buffer 접근

**예상 절감**: read/write 합 ~0.05~0.1ms (memcpy 1회 분).

**판정**: ★★ (Phase 2).

---

## 16.4 isolcpus / CPU affinity / IRQ steering

**현 상태**: CPU3 affinity (`SCHED_FIFO 49`). isolcpus 미설정.

**개선**:
- `isolcpus=3` boot param → CPU3 격리, scheduler 다른 task 안 할당
- `rcu_nocbs=3` → RCU callback CPU3 안 함
- IRQ steering: `i2s` IRQ를 CPU3 외부로 (BCM2712 시스템 IRQ ↔ DMA IRQ)

**예상 효과**: read jitter ↓, period miss 확률 ↓ (xrun 1회/14s → 0).

**판정**: ★★★ (Phase 1, boot param 변경 1회).

---

## 16.5 PREEMPT_RT 커널

**금지**: 사용자 메모 `feedback_anc_style.md` — "baseline 성공 전 PREEMPT_RT 금지".

**판정**: 현재 단계에서 ❌. baseline +5dB 안정화 + xrun 0 달성 후 검토.

---

## 16.6 CPU governor / DVFS

**현 상태**: 미확인.

**확인**: `cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor`

**권장**: `performance` (2.4GHz 고정), 또는 `schedutil` + min_freq=2.4G.

**효과**: 첫 wake-up jitter 제거 (idle → wake ramp 수십 µs).

**판정**: ★★★ (Phase 1, 1줄 변경).

---

## 16.7 Cache prefetch / data locality

**원리**: `__builtin_prefetch(&buf[i+64], 0, 1)`로 다음 cache line 미리 가져옴.

**적용처**: autocorr inner loop (long stride access).

**예상 절감**: A76 L1D 4-cycle latency 가림. ~10% improvement on cache-miss-bound loop.

**판정**: ★★ (Phase 2, NEON 적용 후).

---

## 16.8 Memory locking (mlockall)

**현 상태**: 미확인.

**적용**: `mlockall(MCL_CURRENT | MCL_FUTURE)` — swap-out 차단.

**RT 필수**: page fault 발생 시 ms 단위 지연 → xrun.

**판정**: ★★★ (Phase 1, 1줄).

---

## 16.9 Branch prediction / inline hints

**적용**: `__builtin_expect(f0_active, 1)` — hot path 최적화.

**효과**: 미미하나 무비용.

**판정**: ★ (Phase 3 micro-opt).

---

## 16.10 LTO (Link-Time Optimization)

**현 상태**: 미사용 (`make` 기본).

**적용**: Makefile `CFLAGS += -flto`.

**효과**: cross-file inline → nb_step 인라인 → ~5~15% 속도 개선.

**판정**: ★★ (Phase 2, build flag 1줄).

---

## 16.11 SDR (Same-Data-Rate) 또는 sample-rate 검토

**현 상태**: fs=96kHz. 코골이 80~500Hz 대역에 비해 과샘플링.

**가설**: fs=48kHz로 내리면 sec_path 동일 ms에 절반 tap → autocorr/FIR cost 절반. 단 sec_path 재측정.

**판정**: ★ (장기, 큰 변경).

---

## 16.12 Compute graph 재배치

**현 상태**: nb_step → output limit → safety LPF → clip 순.

**개선**: NEON-friendly로 inline + branch 제거.

**판정**: ★★ (Phase 2, profiler 결과 따라).

---

## 16.13 우선순위 (즉시 가능 카드 우선)

| 카드 | 비용 | 효과 | Phase |
|---|---|---|---|
| **CPU governor performance** (§16.6) | 1 line | jitter ↓ | 1 |
| **mlockall** (§16.8) | 1 line | page fault 0 | 1 |
| **isolcpus=3** (§16.4) | boot param | scheduler 간섭 ↓ | 1 |
| **NEON autocorr** (§16.1) | 1 함수 | BUF_LEN 확장 가능 | 2 |
| **ALSA mmap** (§16.3) | API 교체 | read 0.1ms 절감 | 2 |
| **LTO** (§16.10) | 1 flag | 5~15% | 2 |
| **Cache prefetch** (§16.7) | hint 추가 | autocorr 10% | 2 |
| **Fixed-point** (§16.2) | 큰 변경 | 정밀도 trade | 3 |
| **PREEMPT_RT** | 정책 차단 | 큼 | (사용자 허가 후) |
| **fs 변경** | 큼 | autocorr 절반 | (장기) |

**Phase 1 패키지**: §16.4 + §16.6 + §16.8 — 모두 1~5줄 변경. RT 안정성 즉시 ↑.

**Phase 2 본 카드**: §16.1 NEON autocorr → §04 #2 #3 (BUF_LEN 확장) 차단 해소.
