/*
 * anc_defs.h — 모든 컴파일 타임 상수 (코드 없음).
 *
 * snore_anc.c는 이 파일을 직접 include하기 전에 매크로를 오버라이드한다.
 * #ifndef 가드가 있으므로 오버라이드가 항상 우선한다.
 */
#ifndef ANC_DEFS_H
#define ANC_DEFS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <limits.h>
#include <sched.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#if defined(__aarch64__) || defined(__ARM_NEON)
#include <arm_neon.h>
#define ANC_HAVE_NEON 1
#else
#define ANC_HAVE_NEON 0
#endif
#include <alsa/asoundlib.h>

/* ===== ALSA / 타이밍 ===== */
#ifndef DEVICE
#define DEVICE          "hw:0,0"
#endif
#ifndef SAMPLE_RATE
#define SAMPLE_RATE     96000
#endif
#ifndef CHANNELS
#define CHANNELS        2
#endif
#ifndef REQ_PERIOD
#define REQ_PERIOD      32
#endif
#ifndef REQ_BUFFER
#define REQ_BUFFER      (REQ_PERIOD * 12)
#endif
#ifndef START_FILL_PERIODS
#define START_FILL_PERIODS 2
#endif
#ifndef XRUN_FILL_PERIODS
#define XRUN_FILL_PERIODS 2
#endif

/* ===== FxNLMS 알고리즘 ===== */
#ifndef W_LEN_DEFAULT
#define W_LEN_DEFAULT   1024
#endif
#ifndef S_LEN_DEFAULT
#define S_LEN_DEFAULT   1024
#endif
#ifndef MU_DEFAULT
#define MU_DEFAULT      0.010f
#endif
#ifndef VSS_ALPHA
#define VSS_ALPHA       0.9999f   /* Akhtar VSS 망각 인수 */
#endif
#ifndef VSS_GAMMA
#define VSS_GAMMA       1e-5f     /* Akhtar VSS e² → mu 게인 */
#endif
#ifndef MU_FLOOR
#define MU_FLOOR        0.001f
#endif
#ifndef MU_CEIL
#define MU_CEIL         0.008f    /* VSS raw mu 상한 */
#endif
#ifndef MU_N_FLOOR
#define MU_N_FLOOR      0.012f
#endif
#ifndef MU_N_MAX
#define MU_N_MAX        0.03f
#endif
#ifndef EPSILON_ABS
#define EPSILON_ABS     2e-4f
#endif
#ifndef EPSILON_REL
#define EPSILON_REL     0.010f    /* NLMS 상대 정규화: denom += ε_rel * ||fx||² */
#endif
#ifndef POWER_EMA_ALPHA
#define POWER_EMA_ALPHA 0.95f
#endif
#ifndef LEAKAGE_DEFAULT
#define LEAKAGE_DEFAULT 0.999999f
#endif
#ifndef UPDATE_POW_FLOOR
#define UPDATE_POW_FLOOR 1e-8f
#endif
#ifndef ERR_CLIP
#define ERR_CLIP        1.0f
#endif
#ifndef GRAD_CLIP
#define GRAD_CLIP       0.2f
#endif

/* ===== 채널 배선 ===== */
#ifndef REF_CH
#define REF_CH          1         /* R 채널 = 기준 마이크 */
#endif
#ifndef ERR_CH
#define ERR_CH          0         /* L 채널 = 오차 마이크 */
#endif

/* ===== 출력 안전 ===== */
#ifndef OUTPUT_LIMIT
#define OUTPUT_LIMIT    0.35f
#endif
#ifndef OUTPUT_SAFETY_LPF_HZ
#define OUTPUT_SAFETY_LPF_HZ 200.0f
#endif

/* ===== 발산 감지 ===== */
#ifndef W_NORM_MAX
#define W_NORM_MAX      50.0f
#endif
#ifndef ERR_DIVERGE
#define ERR_DIVERGE     3.0f
#endif
#ifndef CLIP_WARN_FRAC
#define CLIP_WARN_FRAC  0.01f
#endif
#ifndef CLIP_DIVERGE_FRAC
#define CLIP_DIVERGE_FRAC 0.05f
#endif

/* ===== 측정 모드 ===== */
#ifndef MEASURE_SECS
#define MEASURE_SECS    60
#endif
#ifndef CAPTURE_BENCH_SECS_DEFAULT
#define CAPTURE_BENCH_SECS_DEFAULT 10
#endif
#ifndef NOISE_AMP
#define NOISE_AMP       0.2f
#endif
#ifndef MU_MEASURE
#define MU_MEASURE      0.02f
#endif
#ifndef MU_MEASURE_MAX
#define MU_MEASURE_MAX  0.05f
#endif
#ifndef MEASURE_EPS_ABS
#define MEASURE_EPS_ABS 1e-4f
#endif
#ifndef MEASURE_EPS_REL
#define MEASURE_EPS_REL 0.01f
#endif
#ifndef MEASURE_EMA_ALPHA
#define MEASURE_EMA_ALPHA 0.95f
#endif
#ifndef SEC_PATH_FILE
#define SEC_PATH_FILE   "sec_path.bin"
#endif

/* ===== 런타임 제어 ===== */
#ifndef BASELINE_SECS
#define BASELINE_SECS   3
#endif
#ifndef STARTUP_WARMUP_SECS
#define STARTUP_WARMUP_SECS 1
#endif
#ifndef AMBIENT_SECS
#define AMBIENT_SECS    2
#endif
#ifndef XCORR_MAX_LAG
#define XCORR_MAX_LAG   1024
#endif
#ifndef TARGET_BAND_LO_HZ
#define TARGET_BAND_LO_HZ 70.0f
#endif
#ifndef TARGET_BAND_HI_HZ
#define TARGET_BAND_HI_HZ 170.0f
#endif
#ifndef MBAND_BANDS_DEFAULT
#define MBAND_BANDS_DEFAULT 2
#endif
#ifndef MBAND_BANDS_MAX
#define MBAND_BANDS_MAX 8
#endif
#ifndef TARGET_REDUCTION_DB
#define TARGET_REDUCTION_DB 10.0
#endif
#ifndef TONE_TRACK_HZ_DEFAULT
#define TONE_TRACK_HZ_DEFAULT 0.0f
#endif
#ifndef TUNE_DOWN_RATE
#define TUNE_DOWN_RATE  0.7f
#endif
#ifndef RECOVER_SECS
#define RECOVER_SECS    2
#endif
#ifndef ADAPT_DELAY_MS_DEFAULT
#define ADAPT_DELAY_MS_DEFAULT 400
#endif
#ifndef STACK_PREFAULT_BYTES
#define STACK_PREFAULT_BYTES (64 * 1024)
#endif

/* ===== 협대역 ANC ===== */
#ifndef NB_MAX_HARM
#define NB_MAX_HARM       6
#endif
#ifndef NB_F0_BUF_LEN
#define NB_F0_BUF_LEN    2048
#endif
#ifndef NB_F0_MIN_HZ
#define NB_F0_MIN_HZ     60.0f
#endif
#ifndef NB_F0_MAX_HZ
#define NB_F0_MAX_HZ     120.0f
#endif
#ifndef NB_F0_CONF_THR
#define NB_F0_CONF_THR    0.75f
#endif
#ifndef NB_F0_UPDATE_SAMPLES
#define NB_F0_UPDATE_SAMPLES 9600  /* ~200ms @ 48kHz */
#endif
#ifndef NB_MU_DEFAULT
#define NB_MU_DEFAULT     0.001f
#endif
#ifndef NB_LEAK_DEFAULT
#define NB_LEAK_DEFAULT   0.9999f
#endif
#ifndef NB_NLMS_EPS
#define NB_NLMS_EPS       3e-4f   /* 협대역 NLMS 분모 하한 */
#endif

/* ===== 오프라인 시뮬레이션 ===== */
#ifndef SIM_WARMUP_SECS
#define SIM_WARMUP_SECS 2
#endif
#ifndef SIM_SECS
#define SIM_SECS        12
#endif
#ifndef SIM_BASELINE_SECS
#define SIM_BASELINE_SECS 2
#endif

/* ===== 런타임 동적 설정 (매크로 대신 구조체로 전달) ===== */
typedef struct {
    float output_limit;
    float clip_warn_frac;
    float clip_diverge_frac;
    float w_norm_max;
    float err_diverge;
    int   baseline_secs;
    int   warmup_secs;
    int   recover_secs;
    float tune_down_rate;
    float target_reduction_db;
} anc_runtime_cfg_t;

static void anc_runtime_cfg_init_default(anc_runtime_cfg_t *c)
{
    c->output_limit        = OUTPUT_LIMIT;
    c->clip_warn_frac      = CLIP_WARN_FRAC;
    c->clip_diverge_frac   = CLIP_DIVERGE_FRAC;
    c->w_norm_max          = W_NORM_MAX;
    c->err_diverge         = ERR_DIVERGE;
    c->baseline_secs       = BASELINE_SECS;
    c->warmup_secs         = STARTUP_WARMUP_SECS;
    c->recover_secs        = RECOVER_SECS;
    c->tune_down_rate      = TUNE_DOWN_RATE;
    c->target_reduction_db = TARGET_REDUCTION_DB;
}

static int anc_runtime_cfg_validate(const anc_runtime_cfg_t *c)
{
    int ok = 1;
    if (c->output_limit <= 0.0f || c->output_limit > 1.0f) {
        fprintf(stderr, "ERROR: output_limit=%.3f\n", c->output_limit);
        ok = 0;
    }
    if (c->baseline_secs < 1 || c->baseline_secs > 30) {
        fprintf(stderr, "ERROR: baseline_secs=%d\n", c->baseline_secs);
        ok = 0;
    }
    if (c->recover_secs < 1 || c->recover_secs > 60) {
        fprintf(stderr, "ERROR: recover_secs=%d\n", c->recover_secs);
        ok = 0;
    }
    if (c->w_norm_max <= 0.0f) {
        fprintf(stderr, "ERROR: w_norm_max=%.3f\n", c->w_norm_max);
        ok = 0;
    }
    if (c->tune_down_rate <= 0.0f || c->tune_down_rate >= 1.0f) {
        fprintf(stderr, "ERROR: tune_down_rate=%.3f\n", c->tune_down_rate);
        ok = 0;
    }
    if (c->target_reduction_db <= 0.0f) {
        fprintf(stderr, "ERROR: target_reduction_db=%.1f\n", c->target_reduction_db);
        ok = 0;
    }
    return ok;
}

#endif /* ANC_DEFS_H */
