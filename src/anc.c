/*
 * anc.c — FxLMS Active Noise Cancellation (single-file prototype)
 *
 * Build:  gcc -O3 -mcpu=cortex-a76 -o anc anc.c -lasound -lm
 * Usage:  ./anc passthrough
 *         ./anc measure
 *         ./anc run [--no-adapt]
 */

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

/* ===== Configuration ===== */
#ifndef DEVICE
#define DEVICE          "hw:0,0"
#endif
#ifndef SAMPLE_RATE
#define SAMPLE_RATE     48000
#endif
#ifndef CHANNELS
#define CHANNELS        2
#endif
#ifndef REQ_PERIOD
#define REQ_PERIOD      128
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
#define VSS_ALPHA       0.9999f   /* forgetting factor: 시정수 ≈ 1/fs/(1-alpha) */
#endif
#ifndef VSS_GAMMA
#define VSS_GAMMA       1e-5f     /* e^2 → mu 반응 게인                        */
#endif
#ifndef MU_FLOOR
#define MU_FLOOR        0.001f
#endif
#ifndef MU_CEIL
#define MU_CEIL         0.008f   /* pure VSS: mu < 2/fx_pow, worst-case 150Hz */
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
#define EPSILON_REL     0.010f
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

#ifndef REF_CH
#define REF_CH          1       /* R channel = reference mic */
#endif
#ifndef ERR_CH
#define ERR_CH          0       /* L channel = error mic     */
#endif

#ifndef OUTPUT_LIMIT
#define OUTPUT_LIMIT    0.35f
#endif
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
#ifndef OUTPUT_SAFETY_LPF_HZ
#define OUTPUT_SAFETY_LPF_HZ 200.0f
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

/* Narrowband ANC */
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
#define NB_F0_UPDATE_SAMPLES 9600  /* ~200ms at 48kHz */
#endif
#ifndef NB_MU_DEFAULT
#define NB_MU_DEFAULT     0.001f
#endif
#ifndef NB_LEAK_DEFAULT
#define NB_LEAK_DEFAULT   0.9999f
#endif
#ifndef NB_NLMS_EPS
#define NB_NLMS_EPS       3e-4f   /* NLMS power floor — ensures mu_n_max = mu/eps < 2 */
#endif
#ifndef SIM_WARMUP_SECS
#define SIM_WARMUP_SECS 2
#endif
#ifndef SIM_SECS
#define SIM_SECS        12
#endif
#ifndef SIM_BASELINE_SECS
#define SIM_BASELINE_SECS 2
#endif

/* ===== Signal handler ===== */
static volatile sig_atomic_t g_running = 1;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ===== Sample helpers ===== */
static inline float to_f(int16_t v)
{
    return v / 32768.0f;
}

static inline int16_t clip16(float f)
{
    int32_t v = (int32_t)(f * 32768.0f);
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return (int16_t)v;
}

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

#if ANC_HAVE_NEON
static inline float hsum_f32x4(float32x4_t v)
{
#if defined(__aarch64__)
    return vaddvq_f32(v);
#else
    float32x2_t sum = vadd_f32(vget_low_f32(v), vget_high_f32(v));
    sum = vpadd_f32(sum, sum);
    return vget_lane_f32(sum, 0);
#endif
}
#endif

static float dot_f32(const float *a, const float *b, int n)
{
    float sum = 0.0f;
#if ANC_HAVE_NEON
    float32x4_t acc = vdupq_n_f32(0.0f);
    int k = 0;
    for (; k + 4 <= n; k += 4) {
        float32x4_t va = vld1q_f32(a + k);
        float32x4_t vb = vld1q_f32(b + k);
        acc = vmlaq_f32(acc, va, vb);
    }
    sum = hsum_f32x4(acc);
    for (; k < n; k++)
        sum += a[k] * b[k];
#else
    for (int k = 0; k < n; k++)
        sum += a[k] * b[k];
#endif
    return sum;
}

static void sumsq2_f32(const float *a, const float *b, int n,
                       float *sum_a, float *sum_b)
{
    float a_acc = 0.0f;
    float b_acc = 0.0f;
#if ANC_HAVE_NEON
    float32x4_t acc_a = vdupq_n_f32(0.0f);
    float32x4_t acc_b = vdupq_n_f32(0.0f);
    int k = 0;
    for (; k + 4 <= n; k += 4) {
        float32x4_t va = vld1q_f32(a + k);
        float32x4_t vb = vld1q_f32(b + k);
        acc_a = vmlaq_f32(acc_a, va, va);
        acc_b = vmlaq_f32(acc_b, vb, vb);
    }
    a_acc = hsum_f32x4(acc_a);
    b_acc = hsum_f32x4(acc_b);
    for (; k < n; k++) {
        a_acc += a[k] * a[k];
        b_acc += b[k] * b[k];
    }
#else
    for (int k = 0; k < n; k++) {
        a_acc += a[k] * a[k];
        b_acc += b[k] * b[k];
    }
#endif
    *sum_a = a_acc;
    *sum_b = b_acc;
}

static void update_weights_f32(float *w, const float *fx, int n,
                               float leakage, float mu_n,
                               float e_use, float grad_clip)
{
#if ANC_HAVE_NEON
    float32x4_t v_leak = vdupq_n_f32(leakage);
    float32x4_t v_mu = vdupq_n_f32(mu_n);
    float32x4_t v_e = vdupq_n_f32(e_use);
    float32x4_t v_clip_hi = vdupq_n_f32(grad_clip);
    float32x4_t v_clip_lo = vdupq_n_f32(-grad_clip);
    int k = 0;
    for (; k + 4 <= n; k += 4) {
        float32x4_t v_w = vld1q_f32(w + k);
        float32x4_t v_fx = vld1q_f32(fx + k);
        float32x4_t v_grad = vmulq_f32(v_e, v_fx);
        v_grad = vmaxq_f32(v_clip_lo, vminq_f32(v_grad, v_clip_hi));
        v_w = vsubq_f32(vmulq_f32(v_leak, v_w), vmulq_f32(v_mu, v_grad));
        vst1q_f32(w + k, v_w);
    }
    for (; k < n; k++) {
        float grad = clampf(e_use * fx[k], -grad_clip, grad_clip);
        w[k] = leakage * w[k] - mu_n * grad;
    }
#else
    for (int k = 0; k < n; k++) {
        float grad = clampf(e_use * fx[k], -grad_clip, grad_clip);
        w[k] = leakage * w[k] - mu_n * grad;
    }
#endif
}

static inline double db_improvement(double baseline_rms, double cur_rms)
{
    return 20.0 * log10((baseline_rms + 1e-20) / (cur_rms + 1e-20));
}

static float white_noise(uint32_t *seed)
{
    *seed = *seed * 1664525u + 1013904223u;
    return (float)((int32_t)*seed) / 2147483648.0f;
}

static double get_time(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void rt_prepare_memory(int lock_future)
{
    int mlock_flags = MCL_CURRENT | (lock_future ? MCL_FUTURE : 0);
    if (mlockall(mlock_flags) < 0)
        fprintf(stderr, "WARNING: mlockall failed: %s\n", strerror(errno));

    struct sched_param sp;
    sp.sched_priority = 49;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0)
        fprintf(stderr, "WARNING: SCHED_FIFO failed: %s (run with sudo or set rtprio)\n",
                strerror(errno));
    else
        fprintf(stderr, "RT: SCHED_FIFO priority %d\n", sp.sched_priority);

    /* Prevent CPU from entering deep C-states between periods.
     * /dev/cpu_dma_latency=0 tells the PM layer to stay in C0.
     * The fd must remain open for the lifetime of the process. */
    static int dma_lat_fd = -1;
    dma_lat_fd = open("/dev/cpu_dma_latency", O_WRONLY);
    if (dma_lat_fd < 0) {
        fprintf(stderr, "WARNING: cannot open /dev/cpu_dma_latency: %s\n",
                strerror(errno));
    } else {
        const int32_t lat = 0;
        if (write(dma_lat_fd, &lat, sizeof(lat)) < 0)
            fprintf(stderr, "WARNING: cpu_dma_latency write failed: %s\n",
                    strerror(errno));
        else
            fprintf(stderr, "RT: cpu_dma_latency set to 0 (C-states disabled)\n");
        /* intentionally not closed — closing reverts the setting */
    }

    volatile unsigned char stack_prefault[STACK_PREFAULT_BYTES];
    for (size_t i = 0; i < sizeof(stack_prefault); i += 4096)
        stack_prefault[i] = 0;
    stack_prefault[sizeof(stack_prefault) - 1] = 0;
}

/* ===== Simple band shaping for configurable ANC target band ===== */
typedef struct {
    float alpha;
    float x_prev;
    float y_prev;
} hpf1_t;

typedef struct {
    float alpha;
    float y;
} lpf1_t;

typedef struct {
    hpf1_t hp;
    lpf1_t lp;
} band_ctl_t;

typedef struct {
    lpf1_t lp1;
    lpf1_t lp2;
} output_safety_t;

static void hpf1_init(hpf1_t *f, float fc_hz, float fs_hz)
{
    float rc = 1.0f / (2.0f * (float)M_PI * fc_hz);
    float dt = 1.0f / fs_hz;
    f->alpha = rc / (rc + dt);
    f->x_prev = 0.0f;
    f->y_prev = 0.0f;
}

static float hpf1_step(hpf1_t *f, float x)
{
    float y = f->alpha * (f->y_prev + x - f->x_prev);
    f->x_prev = x;
    f->y_prev = y;
    return y;
}

static void lpf1_init(lpf1_t *f, float fc_hz, float fs_hz)
{
    float rc = 1.0f / (2.0f * (float)M_PI * fc_hz);
    float dt = 1.0f / fs_hz;
    f->alpha = dt / (rc + dt);
    f->y = 0.0f;
}

static float lpf1_step(lpf1_t *f, float x)
{
    f->y += f->alpha * (x - f->y);
    return f->y;
}

static void band_ctl_init(band_ctl_t *f, float lo_hz, float hi_hz, float fs_hz)
{
    hpf1_init(&f->hp, lo_hz, fs_hz);
    lpf1_init(&f->lp, hi_hz, fs_hz);
}

static void band_ctl_reset(band_ctl_t *f)
{
    f->hp.x_prev = 0.0f;
    f->hp.y_prev = 0.0f;
    f->lp.y = 0.0f;
}

static float band_ctl_step(band_ctl_t *f, float x)
{
    return lpf1_step(&f->lp, hpf1_step(&f->hp, x));
}

static void output_safety_init(output_safety_t *f, float fs_hz)
{
    lpf1_init(&f->lp1, OUTPUT_SAFETY_LPF_HZ, fs_hz);
    lpf1_init(&f->lp2, OUTPUT_SAFETY_LPF_HZ, fs_hz);
}

static float output_safety_step(output_safety_t *f, float x)
{
    return lpf1_step(&f->lp2, lpf1_step(&f->lp1, x));
}

/* ===== Single-tone meter for exact test frequency tracking ===== */
typedef struct {
    float  freq_hz;
    float  phase;
    float  phase_step;
    double sum_i;
    double sum_q;
    int    sample_count;
} tone_meter_t;

static void tone_meter_init(tone_meter_t *m, float freq_hz, float fs_hz)
{
    memset(m, 0, sizeof(*m));
    m->freq_hz = freq_hz;
    if (freq_hz > 0.0f && fs_hz > 0.0f)
        m->phase_step = 2.0f * (float)M_PI * freq_hz / fs_hz;
}

static void tone_meter_step(tone_meter_t *m, float x)
{
    if (m->phase_step <= 0.0f)
        return;

    m->sum_i += (double)x * cosf(m->phase);
    m->sum_q += (double)x * sinf(m->phase);
    m->sample_count++;

    m->phase += m->phase_step;
    if (m->phase >= 2.0f * (float)M_PI)
        m->phase = fmodf(m->phase, 2.0f * (float)M_PI);
}

static double tone_meter_rms(const tone_meter_t *m)
{
    if (m->sample_count <= 0)
        return 0.0;

    double a_i = 2.0 * m->sum_i / m->sample_count;
    double a_q = 2.0 * m->sum_q / m->sample_count;
    double amp = sqrt(a_i * a_i + a_q * a_q);
    return amp * M_SQRT1_2;
}

static void tone_meter_clear_interval(tone_meter_t *m)
{
    m->sum_i = 0.0;
    m->sum_q = 0.0;
    m->sample_count = 0;
}

/* ===== Cross-correlation ===== */
/*
 * Find lag d that maximizes R(d) = (1/N) * sum_i x(i)*y(i+d).
 * Positive return: y is delayed relative to x by that many samples.
 */
static int xcorr_peak_lag(const float *x, const float *y, int n,
                          int max_lag, float *out_peak_val)
{
    int best_lag = 0;
    float best_corr = -1e30f;

    for (int d = -max_lag; d <= max_lag; d++) {
        double sum = 0.0;
        int i_start = d > 0 ? 0 : -d;
        int i_end   = d > 0 ? n - d : n;
        int count   = i_end - i_start;
        if (count <= 0) continue;

        for (int i = i_start; i < i_end; i++)
            sum += (double)x[i] * (double)y[i + d];

        float corr = (float)(sum / count);
        if (corr > best_corr) {
            best_corr = corr;
            best_lag  = d;
        }
    }
    if (out_peak_val) *out_peak_val = best_corr;
    return best_lag;
}

/* ===== Circular Buffer (double-buffer, contiguous read) ===== */
typedef struct {
    float *data;
    int    cap;
    int    pos;
} cbuf_t;

static void cbuf_init(cbuf_t *b, int cap)
{
    b->cap  = cap;
    b->pos  = 0;
    b->data = (float *)calloc(2 * cap, sizeof(float));
}

static void cbuf_free(cbuf_t *b)
{
    free(b->data);
    b->data = NULL;
}

static void cbuf_push(cbuf_t *b, float v)
{
    int pos = b->pos - 1;
    if (pos < 0)
        pos += b->cap;
    b->pos = pos;
    b->data[b->pos]          = v;
    b->data[b->pos + b->cap] = v;
}

static float *cbuf_ptr(cbuf_t *b)
{
    return &b->data[b->pos];
}

static void cbuf_reset(cbuf_t *b)
{
    memset(b->data, 0, 2 * b->cap * sizeof(float));
    b->pos = 0;
}

/* ===== FxLMS Adaptive Filter ===== */
typedef struct {
    int     w_len;
    int     s_len;
    float  *w;
    float  *s;
    cbuf_t  x_buf;
    cbuf_t  fx_buf;
    float   mu;
    float   mu_floor;
    float   mu_ceil;
    float   mu_n_floor;
    float   mu_n_max;
    float   leakage;
    float   epsilon_abs;
    float   epsilon_rel;
    float   ema_alpha;
    float   update_pow_floor;
    float   err_clip;
    float   grad_clip;
    float   fx_pow_ema;
    float   x_pow_ema;
    float   last_mu_n;
    band_ctl_t ref_band;
    band_ctl_t err_band;
    int     adapt;
    /* Akhtar VSS: mu(n+1) = vss_alpha*mu(n) + vss_gamma*e^2 */
    float   vss_alpha;
    float   vss_gamma;
} fxlms_t;

static void fxlms_set_band(fxlms_t *f, float lo_hz, float hi_hz)
{
    band_ctl_init(&f->ref_band, lo_hz, hi_hz, SAMPLE_RATE);
    band_ctl_init(&f->err_band, lo_hz, hi_hz, SAMPLE_RATE);
}

static void fxlms_init(fxlms_t *f, int w_len, int s_len, float mu, float leak)
{
    f->w_len   = w_len;
    f->s_len   = s_len;
    f->mu      = mu;
    f->mu_floor = MU_FLOOR;
    f->mu_ceil  = MU_CEIL;
    f->mu_n_floor = MU_N_FLOOR;
    f->mu_n_max = MU_N_MAX;
    f->leakage = leak;
    f->epsilon_abs = EPSILON_ABS;
    f->epsilon_rel = EPSILON_REL;
    f->ema_alpha = POWER_EMA_ALPHA;
    f->update_pow_floor = UPDATE_POW_FLOOR;
    f->err_clip = ERR_CLIP;
    f->grad_clip = GRAD_CLIP;
    f->fx_pow_ema = 0.0f;
    f->x_pow_ema = 0.0f;
    f->last_mu_n = 0.0f;
    f->adapt     = 1;
    f->vss_alpha = VSS_ALPHA;
    f->vss_gamma = VSS_GAMMA;
    f->w = (float *)calloc(w_len, sizeof(float));
    f->s = (float *)calloc(s_len, sizeof(float));
    int x_cap = w_len > s_len ? w_len : s_len;
    cbuf_init(&f->x_buf,  x_cap);
    cbuf_init(&f->fx_buf, w_len);
    fxlms_set_band(f, TARGET_BAND_LO_HZ, TARGET_BAND_HI_HZ);
}

static void fxlms_free(fxlms_t *f)
{
    free(f->w);
    free(f->s);
    cbuf_free(&f->x_buf);
    cbuf_free(&f->fx_buf);
}

static float fxlms_step(fxlms_t *f, float x_ref, float e)
{
    cbuf_push(&f->x_buf, x_ref);
    float *x = cbuf_ptr(&f->x_buf);

    /* Filtered-x: x'(n) = S_hat^T . X */
    float x_filt = dot_f32(f->s, x, f->s_len);

    cbuf_push(&f->fx_buf, x_filt);
    float *fx = cbuf_ptr(&f->fx_buf);

    /* Anti-noise output: y = W^T . X */
    float y = dot_f32(f->w, x, f->w_len);

    float fx_pow = 0.0f;
    float x_pow  = 0.0f;
    sumsq2_f32(fx, x, f->w_len, &fx_pow, &x_pow);

    if (f->fx_pow_ema == 0.0f) f->fx_pow_ema = fx_pow;
    else f->fx_pow_ema = f->ema_alpha * f->fx_pow_ema + (1.0f - f->ema_alpha) * fx_pow;
    if (f->x_pow_ema == 0.0f) f->x_pow_ema = x_pow;
    else f->x_pow_ema = f->ema_alpha * f->x_pow_ema + (1.0f - f->ema_alpha) * x_pow;

    /* Akhtar VSS: mu(n+1) = alpha*mu(n) + gamma*e^2, clamped to [floor, ceil]. */
    if (f->vss_gamma > 0.0f)
        f->mu = clampf(f->vss_alpha * f->mu + f->vss_gamma * e * e,
                       f->mu_floor, f->mu_ceil);

    /* Leaky FxNLMS weight update.
     * Denominator: ||fx||²*(1+ε_rel) + ε_abs  (standard NLMS regularization).
     * Result clamped to [mu_n_floor, mu_n_max] for stability. */
    if (f->adapt) {
        float mu_n = (f->fx_pow_ema > f->update_pow_floor)
                     ? clampf(f->mu / (f->fx_pow_ema * (1.0f + f->epsilon_rel)
                                       + f->epsilon_abs),
                               f->mu_n_floor, f->mu_n_max)
                     : 0.0f;
        f->last_mu_n = mu_n;
        if (mu_n > 0.0f) {
            float e_use = clampf(e, -f->err_clip, f->err_clip);
            update_weights_f32(f->w, fx, f->w_len, f->leakage,
                               mu_n, e_use, f->grad_clip);
        }
    }

    return y;
}

static float fxlms_w_norm(fxlms_t *f)
{
    float sum = 0.0f;
    for (int k = 0; k < f->w_len; k++)
        sum += f->w[k] * f->w[k];
    return sqrtf(sum);
}

static int fxlms_load_sec_default(fxlms_t *f);

static void fxlms_reset_state(fxlms_t *f)
{
    cbuf_reset(&f->x_buf);
    cbuf_reset(&f->fx_buf);
    band_ctl_reset(&f->ref_band);
    band_ctl_reset(&f->err_band);
    f->fx_pow_ema = 0.0f;
    f->x_pow_ema = 0.0f;
    f->last_mu_n = 0.0f;
}

static void fxlms_reset_w(fxlms_t *f)
{
    memset(f->w, 0, f->w_len * sizeof(float));
    fxlms_reset_state(f);
}

/* ===== Multi-band FxLMS ===== */
typedef struct {
    int n_bands;
    float band_edges[MBAND_BANDS_MAX + 1];
    fxlms_t bands[MBAND_BANDS_MAX];
} mbfxlms_t;

static void mbfxlms_build_edges(float *edges, int n_bands, float lo_hz, float hi_hz)
{
    if (n_bands <= 1) {
        edges[0] = lo_hz;
        edges[1] = hi_hz;
        return;
    }

    float ratio = hi_hz / lo_hz;
    for (int i = 0; i <= n_bands; i++) {
        float t = (float)i / (float)n_bands;
        edges[i] = lo_hz * powf(ratio, t);
    }
    edges[0] = lo_hz;
    edges[n_bands] = hi_hz;
}

static void mbfxlms_init(mbfxlms_t *m, int n_bands, int w_len, int s_len,
                         float mu, float leak, float lo_hz, float hi_hz)
{
    memset(m, 0, sizeof(*m));
    if (n_bands < 1)
        n_bands = 1;
    if (n_bands > MBAND_BANDS_MAX)
        n_bands = MBAND_BANDS_MAX;

    m->n_bands = n_bands;
    mbfxlms_build_edges(m->band_edges, n_bands, lo_hz, hi_hz);

    for (int i = 0; i < n_bands; i++) {
        fxlms_init(&m->bands[i], w_len, s_len, mu, leak);
        fxlms_set_band(&m->bands[i], m->band_edges[i], m->band_edges[i + 1]);
    }
}

static void mbfxlms_free(mbfxlms_t *m)
{
    for (int i = 0; i < m->n_bands; i++)
        fxlms_free(&m->bands[i]);
}

static int mbfxlms_load_sec_default(mbfxlms_t *m)
{
    if (m->n_bands <= 0)
        return -1;
    if (fxlms_load_sec_default(&m->bands[0]) < 0)
        return -1;

    for (int i = 1; i < m->n_bands; i++) {
        memcpy(m->bands[i].s, m->bands[0].s, m->bands[0].s_len * sizeof(float));
        fprintf(stderr, "Loaded secondary path copy -> band%d %.0f~%.0fHz\n",
                i, m->band_edges[i], m->band_edges[i + 1]);
    }
    return 0;
}

static void mbfxlms_set_adapt(mbfxlms_t *m, int adapt)
{
    for (int i = 0; i < m->n_bands; i++)
        m->bands[i].adapt = adapt;
}

static void mbfxlms_scale_mu(mbfxlms_t *m, float scale)
{
    for (int i = 0; i < m->n_bands; i++) {
        fxlms_t *f = &m->bands[i];
        f->mu = clampf(f->mu * scale, f->mu_floor, f->mu_ceil);
    }
}

static void mbfxlms_set_mu_n_max(mbfxlms_t *m, float mu_n_max)
{
    /* FxNLMS 정규화 step 상한 mu_n_max를 설정. clampf 범위: [mu_n_floor, 2.0]
     * (mu_n < 2/||fx||² 이면 이론적 안정; mu_n_max=2가 실용 상한) */
    for (int i = 0; i < m->n_bands; i++)
        m->bands[i].mu_n_max = clampf(mu_n_max, m->bands[i].mu_n_floor, 2.0f);
}

static void mbfxlms_scale_mu_n_max(mbfxlms_t *m, float scale)
{
    /* FxNLMS 정규화 step 상한 mu_n_max를 scale배. 발산 시 낮추고 회복 시 높인다. */
    for (int i = 0; i < m->n_bands; i++) {
        fxlms_t *f = &m->bands[i];
        f->mu_n_max = clampf(f->mu_n_max * scale, f->mu_n_floor, 2.0f);
    }
}

static void mbfxlms_set_eps(mbfxlms_t *m, float eps_abs, float eps_rel)
{
    for (int i = 0; i < m->n_bands; i++) {
        m->bands[i].epsilon_abs = eps_abs;
        m->bands[i].epsilon_rel = eps_rel;
    }
}

static void mbfxlms_reset_w(mbfxlms_t *m)
{
    for (int i = 0; i < m->n_bands; i++)
        fxlms_reset_w(&m->bands[i]);
}

static void mbfxlms_reset_state(mbfxlms_t *m)
{
    for (int i = 0; i < m->n_bands; i++)
        fxlms_reset_state(&m->bands[i]);
}

static float mbfxlms_step_sample(mbfxlms_t *m, float ref, float e)
{
    float anti = 0.0f;
    for (int i = 0; i < m->n_bands; i++) {
        fxlms_t *f = &m->bands[i];
        float ref_band = band_ctl_step(&f->ref_band, ref);
        float e_band   = band_ctl_step(&f->err_band, e);
        anti += fxlms_step(f, ref_band, e_band);
    }
    return anti;
}

static float mbfxlms_w_norm(const mbfxlms_t *m)
{
    float sum = 0.0f;
    for (int i = 0; i < m->n_bands; i++) {
        float wn = fxlms_w_norm((fxlms_t *)&m->bands[i]);
        sum += wn * wn;
    }
    return sqrtf(sum);
}

static float mbfxlms_mu(const mbfxlms_t *m)
{
    return m->n_bands > 0 ? m->bands[0].mu : 0.0f;
}

static float mbfxlms_mu_n_avg(const mbfxlms_t *m)
{
    if (m->n_bands <= 0)
        return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < m->n_bands; i++)
        sum += m->bands[i].last_mu_n;
    return sum / m->n_bands;
}

static float mbfxlms_fx_pow_sum(const mbfxlms_t *m)
{
    float sum = 0.0f;
    for (int i = 0; i < m->n_bands; i++)
        sum += m->bands[i].fx_pow_ema;
    return sum;
}

static float mbfxlms_x_pow_sum(const mbfxlms_t *m)
{
    float sum = 0.0f;
    for (int i = 0; i < m->n_bands; i++)
        sum += m->bands[i].x_pow_ema;
    return sum;
}

static void mbfxlms_print_bands(const mbfxlms_t *m)
{
    for (int i = 0; i < m->n_bands; i++) {
        fprintf(stderr, "  band%d: %.0f~%.0fHz\n",
                i, m->band_edges[i], m->band_edges[i + 1]);
    }
}

/* ===== Narrowband ANC — pitch-tracking harmonic canceller ===== */

typedef struct {
    float w_c, w_s;       /* cos / sin weights */
    float phase;          /* oscillator phase   */
    float phase_step;     /* 2π·f / fs          */
    float sx_mag;         /* |S(f)|             */
    float sx_phase;       /* ∠S(f)              */
    float cos_sx;         /* cosf(sx_phase) — precomputed for addition theorem */
    float sin_sx;         /* sinf(sx_phase) — precomputed for addition theorem */
    float freq_hz;
} nb_harm_t;

typedef struct {
    /* f0 tracker */
    float  f0_buf[NB_F0_BUF_LEN];
    int    f0_pos;
    int    f0_count;      /* total samples fed */
    int    f0_next_update;
    float  f0_hz;
    float  f0_conf;
    int    lag_min;       /* fs / f_max */
    int    lag_max;       /* fs / f_min */
    /* f0 hold: require 2 consecutive consistent detections before switching */
    float  f0_candidate;
    int    f0_cand_count;

    /* harmonics */
    int    n_harm;
    nb_harm_t harm[NB_MAX_HARM];

    /* sec path (borrowed pointer, not owned) */
    const float *sec;
    int    sec_len;
    float  fs;

    /* control */
    float  mu;
    float  leak;
    int    adapt;

    /* fade-out state: smooth transition when f0 is lost */
    float  last_y;    /* last anti-noise output (for fade-out seed) */
    float  out_env;   /* fade-out envelope [1→0] */
} nb_anc_t;

/* Evaluate secondary path transfer function at one frequency */
static void nb_eval_sec(const float *s, int s_len, float freq_hz, float fs_hz,
                        float *out_mag, float *out_phase)
{
    double re = 0, im = 0;
    double w = 2.0 * M_PI * (double)freq_hz / (double)fs_hz;
    for (int k = 0; k < s_len; k++) {
        re += s[k] * cos(w * k);
        im -= s[k] * sin(w * k);
    }
    *out_mag   = (float)sqrt(re * re + im * im);
    *out_phase = (float)atan2(im, re);
}

static void nb_init(nb_anc_t *nb, int n_harm, float mu, float leak,
                    const float *sec, int sec_len, float fs)
{
    memset(nb, 0, sizeof(*nb));
    nb->n_harm  = (n_harm > NB_MAX_HARM) ? NB_MAX_HARM : n_harm;
    nb->mu      = mu;
    nb->leak    = leak;
    nb->sec     = sec;
    nb->sec_len = sec_len;
    nb->fs      = fs;
    nb->lag_min = (int)(fs / NB_F0_MAX_HZ);
    nb->lag_max = (int)(fs / NB_F0_MIN_HZ);
    if (nb->lag_max >= NB_F0_BUF_LEN / 2)
        nb->lag_max = NB_F0_BUF_LEN / 2 - 1;
    nb->f0_next_update = NB_F0_UPDATE_SAMPLES;
}

/* Set all harmonics to multiples of f0 and recompute S(f) */
static void nb_set_f0(nb_anc_t *nb, float f0_hz)
{
    nb->f0_hz = f0_hz;
    for (int h = 0; h < nb->n_harm; h++) {
        float fh = f0_hz * (float)(h + 1);
        if (fh <= 0.0f || fh > nb->fs * 0.45f) {
            nb->harm[h].freq_hz = 0;
            nb->harm[h].w_c = nb->harm[h].w_s = 0;
            continue;
        }
        float old_freq = nb->harm[h].freq_hz;
        nb->harm[h].freq_hz   = fh;
        nb->harm[h].phase_step = 2.0f * (float)M_PI * fh / nb->fs;
        nb_eval_sec(nb->sec, nb->sec_len, fh, nb->fs,
                    &nb->harm[h].sx_mag, &nb->harm[h].sx_phase);
        nb->harm[h].cos_sx = cosf(nb->harm[h].sx_phase);
        nb->harm[h].sin_sx = sinf(nb->harm[h].sx_phase);
        /* Only reset weights if frequency changed substantially */
        if (old_freq <= 0.0f || fabsf(fh - old_freq) > old_freq * 0.15f) {
            nb->harm[h].w_c = 0;
            nb->harm[h].w_s = 0;
        }
    }
}

/* Autocorrelation-based f0 detection with parabolic interpolation */
static float nb_detect_f0(const float *buf, int len, int lag_min, int lag_max,
                          float fs, float *out_conf)
{
    /* normalized autocorrelation in lag range */
    float best_r = -1.0f;
    int best_lag = lag_min;

    for (int lag = lag_min; lag <= lag_max && lag < len; lag++) {
        double sum = 0, e1 = 0, e2 = 0;
        int n = len - lag;
        for (int i = 0; i < n; i++) {
            sum += (double)buf[i] * buf[i + lag];
            e1  += (double)buf[i] * buf[i];
            e2  += (double)buf[i + lag] * buf[i + lag];
        }
        float denom = (float)sqrt(e1 * e2);
        float r = (denom > 1e-10f) ? (float)(sum / (double)denom) : 0.0f;
        if (r > best_r) {
            best_r  = r;
            best_lag = lag;
        }
    }

    *out_conf = best_r;
    if (best_r < NB_F0_CONF_THR)
        return 0.0f;

    /* parabolic interpolation */
    float lag_f = (float)best_lag;
    if (best_lag > lag_min && best_lag < lag_max) {
        double sum_m = 0, e1_m = 0, e2_m = 0;
        double sum_p = 0, e1_p = 0, e2_p = 0;
        int n_m = len - (best_lag - 1);
        int n_p = len - (best_lag + 1);
        for (int i = 0; i < n_m; i++) {
            sum_m += (double)buf[i] * buf[i + best_lag - 1];
            e1_m  += (double)buf[i] * buf[i];
            e2_m  += (double)buf[i + best_lag - 1] * buf[i + best_lag - 1];
        }
        for (int i = 0; i < n_p; i++) {
            sum_p += (double)buf[i] * buf[i + best_lag + 1];
            e1_p  += (double)buf[i] * buf[i];
            e2_p  += (double)buf[i + best_lag + 1] * buf[i + best_lag + 1];
        }
        float d_m = (float)sqrt(e1_m * e2_m);
        float d_p = (float)sqrt(e1_p * e2_p);
        float r_m = (d_m > 1e-10f) ? (float)(sum_m / (double)d_m) : 0.0f;
        float r_p = (d_p > 1e-10f) ? (float)(sum_p / (double)d_p) : 0.0f;
        float denom2 = 2.0f * (2.0f * best_r - r_m - r_p);
        if (fabsf(denom2) > 1e-10f)
            lag_f += (r_m - r_p) / denom2;
    }
    return fs / lag_f;
}

/* Buffer one sample for f0 tracker (cheap — called per sample) */
static void nb_f0_buf_sample(nb_anc_t *nb, float sample)
{
    nb->f0_buf[nb->f0_pos] = sample;
    nb->f0_pos = (nb->f0_pos + 1) % NB_F0_BUF_LEN;
    nb->f0_count++;
}

/* Run f0 detection if due (expensive — call BETWEEN periods, not per sample).
   Returns 1 if f0 changed. */
static int nb_f0_update(nb_anc_t *nb)
{
    if (nb->f0_count < nb->f0_next_update)
        return 0;
    nb->f0_next_update = nb->f0_count + NB_F0_UPDATE_SAMPLES;

    int filled = (nb->f0_count < NB_F0_BUF_LEN) ? nb->f0_count : NB_F0_BUF_LEN;
    int start  = (nb->f0_pos - filled + NB_F0_BUF_LEN) % NB_F0_BUF_LEN;

    /* Decimate by 4 for fast autocorrelation (48kHz→12kHz, plenty for f0) */
    int dec_len = filled / 4;
    float dec[NB_F0_BUF_LEN / 4];
    for (int i = 0; i < dec_len; i++) {
        int b0 = (start + i * 4)     % NB_F0_BUF_LEN;
        int b1 = (start + i * 4 + 1) % NB_F0_BUF_LEN;
        int b2 = (start + i * 4 + 2) % NB_F0_BUF_LEN;
        dec[i] = (nb->f0_buf[b0] + nb->f0_buf[b1] + nb->f0_buf[b2]) * (1.0f / 3.0f);
    }

    int dec_lag_min = nb->lag_min / 4;
    int dec_lag_max = nb->lag_max / 4;
    if (dec_lag_min < 1) dec_lag_min = 1;
    if (dec_lag_max >= dec_len / 2) dec_lag_max = dec_len / 2 - 1;
    float dec_fs = nb->fs / 4.0f;

    float conf;
    float new_f0 = nb_detect_f0(dec, dec_len, dec_lag_min, dec_lag_max, dec_fs, &conf);
    nb->f0_conf = conf;

    if (new_f0 > 0.0f) {
        /* Hold: require 2 consecutive detections within 15% of each other */
        if (nb->f0_candidate > 0.0f &&
            fabsf(new_f0 - nb->f0_candidate) < nb->f0_candidate * 0.15f) {
            nb->f0_cand_count++;
        } else {
            nb->f0_candidate  = new_f0;
            nb->f0_cand_count = 1;
        }
        if (nb->f0_cand_count >= 2) {
            float apply_f0 = nb->f0_candidate;
            nb->f0_cand_count = 0;
            float delta = fabsf(apply_f0 - nb->f0_hz);
            if (nb->f0_hz <= 0.0f || delta > nb->f0_hz * 0.05f) {
                nb_set_f0(nb, apply_f0);
                return 1;
            }
        }
    } else if (nb->f0_hz > 0.0f) {
        /* Lost pitch — clear candidate and decay weights via leakage */
        nb->f0_candidate  = 0.0f;
        nb->f0_cand_count = 0;
        nb->f0_hz = 0;
        return 1;
    }
    return 0;
}

/* Per-sample narrowband FxLMS: returns anti-noise output */
static float nb_step(nb_anc_t *nb, float err)
{
    /* Fade-out when pitch is lost: decay last output rather than hard mute */
    if (nb->f0_hz <= 0.0f) {
        if (nb->out_env <= 0.0f)
            return 0.0f;
        /* ~200ms fade at 48kHz: tau = 9600 samples → alpha ≈ 1 - 1/9600 */
        nb->out_env *= 0.999896f;
        if (nb->out_env < 1e-4f) nb->out_env = 0.0f;
        return nb->last_y * nb->out_env;
    }

    /* Pitch active — reset fade-out envelope */
    nb->out_env = 1.0f;

    float y = 0.0f;
    for (int h = 0; h < nb->n_harm; h++) {
        nb_harm_t *p = &nb->harm[h];
        if (p->freq_hz <= 0.0f) continue;

        /* One sincosf call per harmonic per sample (replaces 2 separate trig calls) */
        float cos_p, sin_p;
        sincosf(p->phase, &sin_p, &cos_p);

        /* anti-noise output */
        y += p->w_c * cos_p + p->w_s * sin_p;

        /* filtered-x weight update (NLMS):
         * fx = sx_mag * [cos(phase+sx_phase), sin(phase+sx_phase)]
         * Using addition theorem to avoid 2 extra trig calls:
         *   cos(p+sx) = cos_p*cos_sx - sin_p*sin_sx
         *   sin(p+sx) = sin_p*cos_sx + cos_p*sin_sx
         */
        if (nb->adapt && p->sx_mag > 1e-6f) {
            float fx_cos = p->sx_mag * (cos_p * p->cos_sx - sin_p * p->sin_sx);
            float fx_sin = p->sx_mag * (sin_p * p->cos_sx + cos_p * p->sin_sx);
            float power  = p->sx_mag * p->sx_mag + NB_NLMS_EPS; /* ||fx||^2 = sx_mag^2 */
            float mu_n   = nb->mu / power;
            p->w_c -= mu_n * err * fx_cos;
            p->w_s -= mu_n * err * fx_sin;
            p->w_c *= nb->leak;
            p->w_s *= nb->leak;
            /* clamp weight vector magnitude */
            float wmag = sqrtf(p->w_c * p->w_c + p->w_s * p->w_s);
            if (wmag > 0.3f) {
                float inv = 0.3f / wmag;
                p->w_c *= inv;
                p->w_s *= inv;
            }
        }

        /* advance oscillator */
        p->phase += p->phase_step;
        if (p->phase > 2.0f * (float)M_PI)
            p->phase -= 2.0f * (float)M_PI;
    }
    nb->last_y = y;
    return y;
}

static float nb_w_norm(const nb_anc_t *nb)
{
    float sum = 0;
    for (int h = 0; h < nb->n_harm; h++) {
        sum += nb->harm[h].w_c * nb->harm[h].w_c;
        sum += nb->harm[h].w_s * nb->harm[h].w_s;
    }
    return sqrtf(sum);
}

static void nb_reset_weights(nb_anc_t *nb)
{
    for (int h = 0; h < nb->n_harm; h++) {
        nb->harm[h].w_c = 0;
        nb->harm[h].w_s = 0;
    }
}

/* ===== End Narrowband ANC ===== */

static void print_sec_path_stats(const float *s, int s_len, unsigned int rate,
                                 const char *tag)
{
    int peak_idx = 0;
    float peak_val = 0.0f;
    double energy = 0.0;
    for (int k = 0; k < s_len; k++) {
        float abs_v = fabsf(s[k]);
        if (abs_v > peak_val) {
            peak_val = abs_v;
            peak_idx = k;
        }
        energy += (double)s[k] * s[k];
    }

    fprintf(stderr,
            "%s: peak=%.4f @ %d (%.2fms)  rms=%.6f  energy=%.6f\n",
            tag,
            peak_val, peak_idx,
            1000.0f * (float)peak_idx / (float)rate,
            sqrt(energy / s_len), energy);
}

static int fxlms_load_sec(fxlms_t *f, const char *path)
{
    FILE *fp = fopen(path, "rb");
    if (!fp) { perror(path); return -1; }
    int32_t hdr[2];
    if (fread(hdr, sizeof(int32_t), 2, fp) != 2) {
        fprintf(stderr, "%s: header read failed\n", path);
        fclose(fp);
        return -1;
    }
    if (hdr[0] != f->s_len) {
        fprintf(stderr, "%s: s_len mismatch (file=%d, expected=%d)\n",
                path, hdr[0], f->s_len);
        fclose(fp);
        return -1;
    }
    if (fread(f->s, sizeof(float), f->s_len, fp) != (size_t)f->s_len) {
        fprintf(stderr, "%s: data read failed\n", path);
        fclose(fp);
        return -1;
    }
    fclose(fp);
    fprintf(stderr, "Loaded secondary path: %s (s_len=%d, rate=%d)\n",
            path, hdr[0], hdr[1]);
    print_sec_path_stats(f->s, f->s_len, (unsigned int)hdr[1], "S_hat");
    return 0;
}

static int fxlms_load_sec_default(fxlms_t *f)
{
    if (fxlms_load_sec(f, SEC_PATH_FILE) == 0)
        return 0;
#ifdef SEC_PATH_FALLBACK
    fprintf(stderr, "WARNING: falling back to secondary path: %s\n",
            SEC_PATH_FALLBACK);
    return fxlms_load_sec(f, SEC_PATH_FALLBACK);
#else
    return -1;
#endif
}

/* ===== Secondary path file save (standalone) ===== */
static int save_sec_path(const float *s, int s_len, unsigned int rate,
                         const char *path)
{
    FILE *fp = fopen(path, "wb");
    if (!fp) { perror(path); return -1; }
    int32_t hdr[2] = { s_len, (int32_t)rate };
    fwrite(hdr, sizeof(int32_t), 2, fp);
    fwrite(s, sizeof(float), s_len, fp);
    fclose(fp);
    return 0;
}

/* ===== ALSA Context ===== */
typedef struct {
    snd_pcm_t          *cap;
    snd_pcm_t          *play;
    snd_pcm_uframes_t   period;
    snd_pcm_uframes_t   buf_size;
    unsigned int         rate;
    int                  linked;
} alsa_ctx_t;

static void alsa_start(alsa_ctx_t *a);

static int set_hw_params(snd_pcm_t *pcm, unsigned int *rate,
                         snd_pcm_uframes_t *period, snd_pcm_uframes_t *buf)
{
    snd_pcm_hw_params_t *p;
    int err;

    snd_pcm_hw_params_alloca(&p);
    snd_pcm_hw_params_any(pcm, p);
    snd_pcm_hw_params_set_rate_resample(pcm, p, 0);
    snd_pcm_hw_params_set_access(pcm, p, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm, p, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcm, p, CHANNELS);
    snd_pcm_hw_params_set_periods_integer(pcm, p);
    snd_pcm_hw_params_set_rate_near(pcm, p, rate, 0);
    snd_pcm_hw_params_set_period_size_near(pcm, p, period, 0);
    snd_pcm_hw_params_set_buffer_size_near(pcm, p, buf);

    if ((err = snd_pcm_hw_params(pcm, p)) < 0) {
        fprintf(stderr, "hw_params: %s\n", snd_strerror(err));
        return err;
    }

    snd_pcm_hw_params_get_rate(p, rate, 0);
    snd_pcm_hw_params_get_period_size(p, period, 0);
    snd_pcm_hw_params_get_buffer_size(p, buf);

    return 0;
}

static int set_sw_params(snd_pcm_t *pcm, snd_pcm_uframes_t period,
                         snd_pcm_uframes_t buf,
                         snd_pcm_uframes_t start_thr)
{
    snd_pcm_sw_params_t *p;
    int err;

    snd_pcm_sw_params_alloca(&p);
    if ((err = snd_pcm_sw_params_current(pcm, p)) < 0) {
        fprintf(stderr, "sw_params_current: %s\n", snd_strerror(err));
        return err;
    }

    if ((err = snd_pcm_sw_params_set_avail_min(pcm, p, period)) < 0) {
        fprintf(stderr, "sw_params avail_min: %s\n", snd_strerror(err));
        return err;
    }
    if ((err = snd_pcm_sw_params_set_start_threshold(pcm, p, start_thr)) < 0) {
        fprintf(stderr, "sw_params start_threshold: %s\n", snd_strerror(err));
        return err;
    }
    if ((err = snd_pcm_sw_params_set_stop_threshold(pcm, p, buf)) < 0) {
        fprintf(stderr, "sw_params stop_threshold: %s\n", snd_strerror(err));
        return err;
    }

    if ((err = snd_pcm_sw_params(pcm, p)) < 0) {
        fprintf(stderr, "sw_params: %s\n", snd_strerror(err));
        return err;
    }
    return 0;
}

static int pcm_write_full(snd_pcm_t *pcm, const int16_t *buf,
                          snd_pcm_uframes_t frames)
{
    const int16_t *ptr = buf;
    snd_pcm_uframes_t left = frames;

    while (left > 0) {
        snd_pcm_sframes_t n = snd_pcm_writei(pcm, ptr, left);
        if (n < 0)
            return (int)n;
        if (n == 0)
            return -EIO;
        ptr  += n * CHANNELS;
        left -= (snd_pcm_uframes_t)n;
    }
    return 0;
}

static int pcm_read_full(snd_pcm_t *pcm, int16_t *buf, snd_pcm_uframes_t frames)
{
    int16_t *ptr = buf;
    snd_pcm_uframes_t left = frames;

    while (left > 0) {
        snd_pcm_sframes_t n = snd_pcm_readi(pcm, ptr, left);
        if (n < 0)
            return (int)n;
        if (n == 0)
            return -EIO;
        ptr  += n * CHANNELS;
        left -= (snd_pcm_uframes_t)n;
    }
    return 0;
}

static int alsa_prepare_streams(alsa_ctx_t *a)
{
    int err;

    if (a->linked || !a->play)
        return snd_pcm_prepare(a->cap);

    if ((err = snd_pcm_prepare(a->play)) < 0)
        return err;
    return snd_pcm_prepare(a->cap);
}

static int alsa_prefill_playback(alsa_ctx_t *a, const int16_t *buf, int periods)
{
    if (!a->play || !buf || periods <= 0)
        return 0;

    for (int i = 0; i < periods; i++) {
        int err = pcm_write_full(a->play, buf, a->period);
        if (err < 0)
            return err;
    }
    return 0;
}

static int alsa_resync(alsa_ctx_t *a, int16_t *silence_buf, int fill_periods)
{
    int err;

    if (a->play && silence_buf)
        memset(silence_buf, 0, a->period * CHANNELS * sizeof(int16_t));
    if (a->play)
        snd_pcm_drop(a->play);
    if (a->cap)
        snd_pcm_drop(a->cap);

    if ((err = alsa_prepare_streams(a)) < 0) {
        fprintf(stderr, "prepare after xrun failed: %s\n", snd_strerror(err));
        return err;
    }
    if ((err = alsa_prefill_playback(a, silence_buf, fill_periods)) < 0) {
        fprintf(stderr, "prefill after xrun failed: %s\n", snd_strerror(err));
        return err;
    }
    alsa_start(a);
    return 0;
}

static int alsa_handle_io_error(alsa_ctx_t *a, snd_pcm_t *pcm, const char *stream,
                                int err, int16_t *silence_buf,
                                int xrun_fill_periods)
{
    fprintf(stderr, "xrun (%s): %s\n", stream, snd_strerror(err));
    err = snd_pcm_recover(pcm, err, 0);
    if (err < 0) {
        fprintf(stderr, "recover (%s) failed: %s\n", stream, snd_strerror(err));
        return err;
    }
    return alsa_resync(a, silence_buf, xrun_fill_periods);
}

static int alsa_init(alsa_ctx_t *a, const char *dev, unsigned int req_rate,
                     snd_pcm_uframes_t req_period, snd_pcm_uframes_t req_buf)
{
    int err;
    memset(a, 0, sizeof(*a));

    if ((err = snd_pcm_open(&a->cap, dev, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf(stderr, "capture open: %s\n", snd_strerror(err));
        return err;
    }
    if ((err = snd_pcm_open(&a->play, dev, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "playback open: %s\n", snd_strerror(err));
        snd_pcm_close(a->cap);
        return err;
    }

    unsigned int cap_rate = req_rate, play_rate = req_rate;
    snd_pcm_uframes_t cap_period = req_period, play_period = req_period;
    snd_pcm_uframes_t cap_buf = req_buf, play_buf = req_buf;

    if (set_hw_params(a->cap,  &cap_rate,  &cap_period,  &cap_buf)  < 0) goto fail;
    if (set_hw_params(a->play, &play_rate, &play_period, &play_buf) < 0) goto fail;
    if (set_sw_params(a->cap, cap_period, cap_buf, cap_period) < 0) goto fail;
    if (set_sw_params(a->play, play_period, play_buf, play_buf) < 0) goto fail;

    /* --- Strict parameter validation --- */
    if (cap_rate != play_rate) {
        fprintf(stderr, "FATAL: rate mismatch  capture=%u  playback=%u\n",
                cap_rate, play_rate);
        goto fail;
    }
    if (cap_period != play_period) {
        fprintf(stderr, "FATAL: period mismatch  capture=%lu  playback=%lu\n",
                (unsigned long)cap_period, (unsigned long)play_period);
        goto fail;
    }
    if (cap_buf != play_buf) {
        fprintf(stderr, "FATAL: buffer mismatch  capture=%lu  playback=%lu\n",
                (unsigned long)cap_buf, (unsigned long)play_buf);
        goto fail;
    }

    a->rate     = cap_rate;
    a->period   = cap_period;
    a->buf_size = cap_buf;

    /* Link capture and playback for synchronous start */
    if ((err = snd_pcm_link(a->cap, a->play)) < 0) {
        fprintf(stderr, "WARNING: snd_pcm_link failed: %s (continuing unlinked)\n",
                snd_strerror(err));
        a->linked = 0;
    } else {
        a->linked = 1;
    }

    return 0;

fail:
    snd_pcm_close(a->play);
    snd_pcm_close(a->cap);
    return -1;
}

static int alsa_init_capture_only(alsa_ctx_t *a, const char *dev,
                                  unsigned int req_rate,
                                  snd_pcm_uframes_t req_period,
                                  snd_pcm_uframes_t req_buf)
{
    int err;
    memset(a, 0, sizeof(*a));

    if ((err = snd_pcm_open(&a->cap, dev, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf(stderr, "capture open: %s\n", snd_strerror(err));
        return err;
    }

    unsigned int cap_rate = req_rate;
    snd_pcm_uframes_t cap_period = req_period;
    snd_pcm_uframes_t cap_buf = req_buf;

    if (set_hw_params(a->cap, &cap_rate, &cap_period, &cap_buf) < 0)
        goto fail;
    if (set_sw_params(a->cap, cap_period, cap_buf, cap_period) < 0)
        goto fail;

    a->rate = cap_rate;
    a->period = cap_period;
    a->buf_size = cap_buf;
    a->linked = 0;
    return 0;

fail:
    snd_pcm_close(a->cap);
    return -1;
}

static void alsa_print(alsa_ctx_t *a)
{
    fprintf(stderr, "ALSA: rate=%u  period=%lu  buffer=%lu  linked=%d\n",
            a->rate, (unsigned long)a->period, (unsigned long)a->buf_size,
            a->linked);
}

static void alsa_close(alsa_ctx_t *a)
{
    if (a->linked)
        snd_pcm_unlink(a->cap);
    if (a->play) { snd_pcm_drop(a->play); snd_pcm_close(a->play); }
    if (a->cap)  { snd_pcm_drop(a->cap);  snd_pcm_close(a->cap);  }
    fprintf(stderr, "ALSA closed.\n");
}

static void alsa_start(alsa_ctx_t *a)
{
    if (a->linked)
        snd_pcm_start(a->cap);
    else if (a->play) {
        snd_pcm_start(a->play);
        snd_pcm_start(a->cap);
    } else {
        snd_pcm_start(a->cap);
    }
}

/* ===== Logger ===== */
typedef struct {
    double err_sum_sq;
    double tone_sum_sq;
    double anti_sum_sq;
    double read_sum_sec;
    double compute_sum_sec;
    double write_sum_sec;
    double total_sum_sec;
    float  anti_peak;
    double read_max_sec;
    double compute_max_sec;
    double write_max_sec;
    double total_max_sec;
    int    frame_count;
    int    xrun_total;
    int    log_interval;
    int    clip_count;
    int    period_count;
    double baseline_err_rms;
    double baseline_tone_rms;
    double best_tone_db;
    double baseline_track_rms;
    double best_track_db;
    tone_meter_t tone_track;
    int    frozen;
    int    recover_sec;
} logger_t;

static void logger_init(logger_t *l, int sample_rate, float tone_hz)
{
    memset(l, 0, sizeof(*l));
    l->log_interval = sample_rate;
    l->best_tone_db = -1e9;
    l->best_track_db = -1e9;
    tone_meter_init(&l->tone_track, tone_hz, (float)sample_rate);
}

static void logger_update(logger_t *l, float e_sample, float anti_sample,
                          float e_tone_sample, int clipped)
{
    l->err_sum_sq  += (double)e_sample * e_sample;
    l->tone_sum_sq += (double)e_tone_sample * e_tone_sample;
    l->anti_sum_sq += (double)anti_sample * anti_sample;
    float abs_anti = fabsf(anti_sample);
    if (abs_anti > l->anti_peak) l->anti_peak = abs_anti;
    if (clipped) l->clip_count++;
    tone_meter_step(&l->tone_track, e_sample);
    l->frame_count++;
}

static void logger_xrun(logger_t *l)
{
    l->xrun_total++;
}

static void logger_period_times(logger_t *l, double read_sec, double compute_sec,
                                double write_sec, double total_sec)
{
    l->read_sum_sec += read_sec;
    l->compute_sum_sec += compute_sec;
    l->write_sum_sec += write_sec;
    l->total_sum_sec += total_sec;
    if (read_sec > l->read_max_sec) l->read_max_sec = read_sec;
    if (compute_sec > l->compute_max_sec) l->compute_max_sec = compute_sec;
    if (write_sec > l->write_max_sec) l->write_max_sec = write_sec;
    if (total_sec > l->total_max_sec) l->total_max_sec = total_sec;
    l->period_count++;
}

static int logger_should_print(logger_t *l)
{
    return l->frame_count >= l->log_interval;
}

/*
 * Print one-second stats.  Returns this interval's err_rms.
 * out_clip_frac receives clip fraction before counters reset.
 */
static double logger_print(logger_t *l, float w_norm, double elapsed_sec,
                           float mu, float mu_n, float fx_pow, float x_pow,
                           int n_bands, int w_len, int s_len,
                           double period_budget_ms,
                           const char *status,
                           double *out_clip_frac, double *out_tone_db,
                           double *out_track_db)
{
    if (l->frame_count == 0) {
        if (out_clip_frac) *out_clip_frac = 0;
        if (out_tone_db) *out_tone_db = 0;
        if (out_track_db) *out_track_db = 0;
        return 0;
    }

    double err_rms   = sqrt(l->err_sum_sq  / l->frame_count);
    double tone_rms  = sqrt(l->tone_sum_sq / l->frame_count);
    double anti_rms  = sqrt(l->anti_sum_sq / l->frame_count);
    double clip_frac = (double)l->clip_count / l->frame_count;
    double tone_db   = 0.0;
    double track_rms = tone_meter_rms(&l->tone_track);
    double track_db  = 0.0;

    fprintf(stderr, "[ANC %6.1fs] err=%.4f", elapsed_sec, err_rms);

    if (l->baseline_err_rms > 0) {
        double ratio = err_rms / l->baseline_err_rms;
        double improve_db = db_improvement(l->baseline_err_rms, err_rms);
        fprintf(stderr, "  base=%.4f  r=%.3f  %+.1fdB",
                l->baseline_err_rms, ratio, improve_db);
    }

    if (l->baseline_tone_rms > 0) {
        tone_db = db_improvement(l->baseline_tone_rms, tone_rms);
        if (tone_db > l->best_tone_db) l->best_tone_db = tone_db;
        fprintf(stderr, "  tone=%.4f  %+.1fdB(best=%+.1fdB)",
                tone_rms, tone_db, l->best_tone_db);
    } else {
        fprintf(stderr, "  tone=%.4f", tone_rms);
    }

    if (l->tone_track.freq_hz > 0.0f) {
        if (l->baseline_track_rms > 0) {
            track_db = db_improvement(l->baseline_track_rms, track_rms);
            if (track_db > l->best_track_db) l->best_track_db = track_db;
            fprintf(stderr, "  track@%.1f=%.4f  %+.1fdB(best=%+.1fdB)",
                    l->tone_track.freq_hz, track_rms,
                    track_db, l->best_track_db);
        } else {
            fprintf(stderr, "  track@%.1f=%.4f",
                    l->tone_track.freq_hz, track_rms);
        }
    }

    fprintf(stderr, "  anti=%.4f  pk=%.3f  clip=%d(%.2f%%)"
            "  w=%.3f  mu=%.5f  mu_n=%.5f  fx=%.3e  x=%.3e"
            "  bands=%d wlen=%d slen=%d  xr=%d  %s\n",
            anti_rms, l->anti_peak,
            l->clip_count, clip_frac * 100.0,
            w_norm,
            mu, mu_n, fx_pow, x_pow,
            n_bands, w_len, s_len,
            l->xrun_total, status);

    if (l->period_count > 0) {
        double read_avg_ms = l->read_sum_sec * 1000.0 / l->period_count;
        double compute_avg_ms = l->compute_sum_sec * 1000.0 / l->period_count;
        double write_avg_ms = l->write_sum_sec * 1000.0 / l->period_count;
        double total_avg_ms = l->total_sum_sec * 1000.0 / l->period_count;
        fprintf(stderr,
                "  perf_ms: read=%.3f/%.3f  compute=%.3f/%.3f"
                "  write=%.3f/%.3f  total=%.3f/%.3f  budget=%.3f\n",
                read_avg_ms, l->read_max_sec * 1000.0,
                compute_avg_ms, l->compute_max_sec * 1000.0,
                write_avg_ms, l->write_max_sec * 1000.0,
                total_avg_ms, l->total_max_sec * 1000.0,
                period_budget_ms);
    }

    if (clip_frac > CLIP_WARN_FRAC)
        fprintf(stderr, "  WARNING: limiter active on %.1f%% of samples\n",
                clip_frac * 100.0);

    if (out_clip_frac) *out_clip_frac = clip_frac;
    if (out_tone_db) *out_tone_db = tone_db;
    if (out_track_db) *out_track_db = track_db;
    double ret = err_rms;

    /* Reset per-interval (keep xrun_total, baseline_err_rms, frozen) */
    l->err_sum_sq  = 0;
    l->tone_sum_sq = 0;
    l->anti_sum_sq = 0;
    l->anti_peak   = 0.0f;
    l->clip_count  = 0;
    l->frame_count = 0;
    l->read_sum_sec = 0.0;
    l->compute_sum_sec = 0.0;
    l->write_sum_sec = 0.0;
    l->total_sum_sec = 0.0;
    l->read_max_sec = 0.0;
    l->compute_max_sec = 0.0;
    l->write_max_sec = 0.0;
    l->total_max_sec = 0.0;
    l->period_count = 0;
    tone_meter_clear_interval(&l->tone_track);

    return ret;
}

typedef struct {
    double ref_sum_sq;
    double err_sum_sq;
    double read_sum_sec;
    double total_sum_sec;
    double read_max_sec;
    double total_max_sec;
    double avail_sum;
    double delay_sum;
    snd_pcm_sframes_t avail_min;
    snd_pcm_sframes_t avail_max;
    snd_pcm_sframes_t delay_min;
    snd_pcm_sframes_t delay_max;
    long frame_count;
    int period_count;
    int xrun_total;
} capture_bench_stats_t;

static void capture_bench_stats_reset_interval(capture_bench_stats_t *s)
{
    s->ref_sum_sq = 0.0;
    s->err_sum_sq = 0.0;
    s->read_sum_sec = 0.0;
    s->total_sum_sec = 0.0;
    s->read_max_sec = 0.0;
    s->total_max_sec = 0.0;
    s->avail_sum = 0.0;
    s->delay_sum = 0.0;
    s->avail_min = 0;
    s->avail_max = 0;
    s->delay_min = 0;
    s->delay_max = 0;
    s->frame_count = 0;
    s->period_count = 0;
}

static void capture_bench_stats_init(capture_bench_stats_t *s)
{
    memset(s, 0, sizeof(*s));
    capture_bench_stats_reset_interval(s);
}

static void capture_bench_stats_update(capture_bench_stats_t *s,
                                       double read_sec, double total_sec,
                                       snd_pcm_sframes_t avail,
                                       snd_pcm_sframes_t delay,
                                       const int16_t *buf,
                                       snd_pcm_uframes_t frames)
{
    s->read_sum_sec += read_sec;
    s->total_sum_sec += total_sec;
    if (read_sec > s->read_max_sec) s->read_max_sec = read_sec;
    if (total_sec > s->total_max_sec) s->total_max_sec = total_sec;
    s->avail_sum += (double)avail;
    s->delay_sum += (double)delay;
    if (s->period_count == 0) {
        s->avail_min = s->avail_max = avail;
        s->delay_min = s->delay_max = delay;
    } else {
        if (avail < s->avail_min) s->avail_min = avail;
        if (avail > s->avail_max) s->avail_max = avail;
        if (delay < s->delay_min) s->delay_min = delay;
        if (delay > s->delay_max) s->delay_max = delay;
    }

    for (snd_pcm_uframes_t i = 0; i < frames; i++) {
        float ref = to_f(buf[i * 2 + REF_CH]);
        float err = to_f(buf[i * 2 + ERR_CH]);
        s->ref_sum_sq += (double)ref * ref;
        s->err_sum_sq += (double)err * err;
    }
    s->frame_count += (long)frames;
    s->period_count++;
}

static void capture_bench_print(capture_bench_stats_t *s, double elapsed_sec,
                                unsigned int rate, snd_pcm_uframes_t period)
{
    if (s->frame_count <= 0 || s->period_count <= 0)
        return;

    double ref_rms = sqrt(s->ref_sum_sq / s->frame_count);
    double err_rms = sqrt(s->err_sum_sq / s->frame_count);
    double read_avg_ms = s->read_sum_sec * 1000.0 / s->period_count;
    double total_avg_ms = s->total_sum_sec * 1000.0 / s->period_count;
    double avail_avg = s->avail_sum / s->period_count;
    double delay_avg = s->delay_sum / s->period_count;
    double budget_ms = 1000.0 * (double)period / (double)rate;

    fprintf(stderr,
            "[CAP %6.1fs] ref=%.4f  err=%.4f  xr=%d  periods=%d\n",
            elapsed_sec, ref_rms, err_rms, s->xrun_total, s->period_count);
    fprintf(stderr,
            "  perf_ms: read=%.3f/%.3f  total=%.3f/%.3f  budget=%.3f\n",
            read_avg_ms, s->read_max_sec * 1000.0,
            total_avg_ms, s->total_max_sec * 1000.0,
            budget_ms);
    fprintf(stderr,
            "  alsa_frames: avail=%.1f/%ld/%ld  delay=%.1f/%ld/%ld\n",
            avail_avg, (long)s->avail_min, (long)s->avail_max,
            delay_avg, (long)s->delay_min, (long)s->delay_max);
}

static void run_capture_bench(alsa_ctx_t *a, int duration_sec)
{
    snd_pcm_uframes_t period = a->period;
    int16_t *in_buf = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    capture_bench_stats_t stats;
    capture_bench_stats_init(&stats);

    if (!in_buf)
        return;

    if (alsa_prepare_streams(a) < 0) {
        free(in_buf);
        return;
    }
    alsa_start(a);

    double t_start = get_time();
    long total_frames = 0;
    long max_frames = (duration_sec > 0) ? (long)a->rate * duration_sec : LONG_MAX;
    fprintf(stderr, "Capture bench: capture-only read timing\n");

    while (g_running && total_frames < max_frames) {
        double loop_start = get_time();
        snd_pcm_sframes_t avail = 0, delay = 0;
        if (snd_pcm_avail_delay(a->cap, &avail, &delay) < 0) {
            avail = 0;
            delay = 0;
        }

        int err = pcm_read_full(a->cap, in_buf, period);
        double after_read = get_time();
        if (err < 0) {
            stats.xrun_total++;
            if (alsa_handle_io_error(a, a->cap, "capture", err, NULL, 0) < 0)
                break;
            continue;
        }

        capture_bench_stats_update(&stats,
                                   after_read - loop_start,
                                   after_read - loop_start,
                                   avail, delay, in_buf, period);
        total_frames += (long)period;

        if (stats.frame_count >= (long)a->rate) {
            int xruns = stats.xrun_total;
            capture_bench_print(&stats, get_time() - t_start, a->rate, a->period);
            capture_bench_stats_reset_interval(&stats);
            stats.xrun_total = xruns;
        }
    }

    if (stats.frame_count > 0)
        capture_bench_print(&stats, get_time() - t_start, a->rate, a->period);

    free(in_buf);
}

typedef struct {
    double prep_sum_sec;
    double prep_max_sec;
    double write_sum_sec;
    double write_max_sec;
    double read_sum_sec;
    double read_max_sec;
    double compute_sum_sec;
    double compute_max_sec;
    double total_sum_sec;
    double total_max_sec;
    int period_count;
} measure_perf_t;

static void measure_perf_reset(measure_perf_t *p)
{
    memset(p, 0, sizeof(*p));
}

static void measure_perf_update(measure_perf_t *p,
                                double prep_sec, double write_sec,
                                double read_sec, double compute_sec,
                                double total_sec)
{
    p->prep_sum_sec += prep_sec;
    p->write_sum_sec += write_sec;
    p->read_sum_sec += read_sec;
    p->compute_sum_sec += compute_sec;
    p->total_sum_sec += total_sec;

    if (prep_sec > p->prep_max_sec) p->prep_max_sec = prep_sec;
    if (write_sec > p->write_max_sec) p->write_max_sec = write_sec;
    if (read_sec > p->read_max_sec) p->read_max_sec = read_sec;
    if (compute_sec > p->compute_max_sec) p->compute_max_sec = compute_sec;
    if (total_sec > p->total_max_sec) p->total_max_sec = total_sec;
    p->period_count++;
}

static void measure_perf_print_interval(const char *tag,
                                        const char *prep_name,
                                        const char *compute_name,
                                        const measure_perf_t *p,
                                        unsigned int rate,
                                        snd_pcm_uframes_t period)
{
    if (p->period_count <= 0)
        return;

    double budget_ms = 1000.0 * (double)period / (double)rate;
    fprintf(stderr,
            "  %s perf_ms: %s=%.3f/%.3f  write=%.3f/%.3f"
            "  read=%.3f/%.3f  %s=%.3f/%.3f  total=%.3f/%.3f"
            "  budget=%.3f\n",
            tag,
            prep_name,
            p->prep_sum_sec * 1000.0 / p->period_count,
            p->prep_max_sec * 1000.0,
            p->write_sum_sec * 1000.0 / p->period_count,
            p->write_max_sec * 1000.0,
            p->read_sum_sec * 1000.0 / p->period_count,
            p->read_max_sec * 1000.0,
            compute_name,
            p->compute_sum_sec * 1000.0 / p->period_count,
            p->compute_max_sec * 1000.0,
            p->total_sum_sec * 1000.0 / p->period_count,
            p->total_max_sec * 1000.0,
            budget_ms);
}

static void measure_perf_print_summary(const char *label,
                                       const char *prep_name,
                                       const char *compute_name,
                                       const measure_perf_t *p,
                                       unsigned int rate,
                                       snd_pcm_uframes_t period,
                                       double resync_sec,
                                       const char *analysis_name,
                                       double analysis_sec,
                                       const char *extra_name,
                                       double extra_sec,
                                       double total_wall_sec)
{
    double prep_avg_ms = 0.0;
    double write_avg_ms = 0.0;
    double read_avg_ms = 0.0;
    double compute_avg_ms = 0.0;
    double total_avg_ms = 0.0;
    double budget_ms = 1000.0 * (double)period / (double)rate;
    double accounted_sec;

    if (p->period_count > 0) {
        prep_avg_ms = p->prep_sum_sec * 1000.0 / p->period_count;
        write_avg_ms = p->write_sum_sec * 1000.0 / p->period_count;
        read_avg_ms = p->read_sum_sec * 1000.0 / p->period_count;
        compute_avg_ms = p->compute_sum_sec * 1000.0 / p->period_count;
        total_avg_ms = p->total_sum_sec * 1000.0 / p->period_count;
    }

    accounted_sec = resync_sec + p->prep_sum_sec + p->write_sum_sec +
                    p->read_sum_sec + p->compute_sum_sec +
                    analysis_sec + extra_sec;

    fprintf(stderr, "%s timing:\n", label);
    fprintf(stderr,
            "  avg/max ms: %s=%.3f/%.3f  write=%.3f/%.3f"
            "  read=%.3f/%.3f  %s=%.3f/%.3f  total=%.3f/%.3f"
            "  budget=%.3f  periods=%d\n",
            prep_name, prep_avg_ms, p->prep_max_sec * 1000.0,
            write_avg_ms, p->write_max_sec * 1000.0,
            read_avg_ms, p->read_max_sec * 1000.0,
            compute_name, compute_avg_ms, p->compute_max_sec * 1000.0,
            total_avg_ms, p->total_max_sec * 1000.0,
            budget_ms, p->period_count);
    fprintf(stderr,
            "  time_s: resync=%.6f  %s=%.6f  write=%.6f  read=%.6f"
            "  %s=%.6f",
            resync_sec, prep_name, p->prep_sum_sec,
            p->write_sum_sec, p->read_sum_sec,
            compute_name, p->compute_sum_sec);
    if (analysis_name && analysis_name[0] != '\0')
        fprintf(stderr, "  %s=%.6f", analysis_name, analysis_sec);
    if (extra_name && extra_name[0] != '\0')
        fprintf(stderr, "  %s=%.6f", extra_name, extra_sec);
    fprintf(stderr, "  total_wall=%.6f", total_wall_sec);
    if (total_wall_sec > accounted_sec)
        fprintf(stderr, "  other=%.6f", total_wall_sec - accounted_sec);
    fprintf(stderr, "\n");

    if (total_wall_sec > 0.0) {
        fprintf(stderr,
                "  share%%: %s=%.1f  write=%.1f  read=%.1f  %s=%.1f",
                prep_name, 100.0 * p->prep_sum_sec / total_wall_sec,
                100.0 * p->write_sum_sec / total_wall_sec,
                100.0 * p->read_sum_sec / total_wall_sec,
                compute_name, 100.0 * p->compute_sum_sec / total_wall_sec);
        if (analysis_name && analysis_name[0] != '\0')
            fprintf(stderr, "  %s=%.1f",
                    analysis_name, 100.0 * analysis_sec / total_wall_sec);
        if (extra_name && extra_name[0] != '\0')
            fprintf(stderr, "  %s=%.1f",
                    extra_name, 100.0 * extra_sec / total_wall_sec);
        fprintf(stderr, "\n");
    }
}

/* ===== Measurement Mode ===== */
static void run_measurement(alsa_ctx_t *a, int s_len, float noise_amp,
                            int duration_sec, const char *out_path,
                            int start_fill_periods, int xrun_fill_periods)
{
    snd_pcm_uframes_t period = a->period;
    unsigned int rate = a->rate;

    int16_t *in_buf  = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    int16_t *out_buf = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    int xruns = 0;
    float *amb_ref = NULL;
    float *amb_err = NULL;
    float *sig_out = NULL;
    float *sig_err = NULL;
    float *s_hat = NULL;
    cbuf_t u_buf = {0};
    output_safety_t out_safety;
    measure_perf_t amb_perf_interval;
    measure_perf_t amb_perf_total;
    measure_perf_t meas_perf_interval;
    measure_perf_t meas_perf_total;
    double phase1_resync_sec = 0.0;
    double phase1_xcorr_sec = 0.0;
    double phase2_resync_sec = 0.0;
    double phase2_peak_scan_sec = 0.0;
    double phase2_xcorr_sec = 0.0;
    double phase2_save_sec = 0.0;
    double phase1_elapsed_sec = 0.0;

    measure_perf_reset(&amb_perf_interval);
    measure_perf_reset(&amb_perf_total);
    measure_perf_reset(&meas_perf_interval);
    measure_perf_reset(&meas_perf_total);
    output_safety_init(&out_safety, (float)rate);

    /* ========== Phase 1: Ambient capture — ref mic lead check ========== */
    long amb_total = (long)rate * AMBIENT_SECS;
    amb_ref = (float *)malloc(amb_total * sizeof(float));
    amb_err = (float *)malloc(amb_total * sizeof(float));
    long amb_count = 0;
    double amb_ref_sq = 0, amb_err_sq = 0;

    /* Pre-fill playback with silence, then start */
    double phase1_resync_start = get_time();
    if (alsa_resync(a, out_buf, start_fill_periods) < 0)
        goto cleanup;
    phase1_resync_sec = get_time() - phase1_resync_start;

    fprintf(stderr, "\n=== Phase 1: Ambient capture (%d sec) ===\n", AMBIENT_SECS);
    double phase1_start = get_time();
    double phase1_next_log = 1.0;

    while (g_running && amb_count < amb_total) {
        double loop_start = get_time();
        int err = pcm_write_full(a->play, out_buf, period);  /* silence */
        double after_write = get_time();
        if (err < 0) {
            xruns++;
            if (alsa_handle_io_error(a, a->play, "playback", err, out_buf,
                                     xrun_fill_periods) < 0)
                goto cleanup;
            continue;
        }

        err = pcm_read_full(a->cap, in_buf, period);
        double after_read = get_time();
        if (err < 0) {
            xruns++;
            if (alsa_handle_io_error(a, a->cap, "capture", err, out_buf,
                                     xrun_fill_periods) < 0)
                goto cleanup;
            continue;
        }

        for (int i = 0; i < (int)period && amb_count < amb_total; i++) {
            float r = to_f(in_buf[i * 2 + REF_CH]);
            float e = to_f(in_buf[i * 2 + ERR_CH]);
            amb_ref[amb_count] = r;
            amb_err[amb_count] = e;
            amb_ref_sq += (double)r * r;
            amb_err_sq += (double)e * e;
            amb_count++;
        }
        double after_copy = get_time();
        measure_perf_update(&amb_perf_interval,
                            0.0,
                            after_write - loop_start,
                            after_read - after_write,
                            after_copy - after_read,
                            after_copy - loop_start);
        measure_perf_update(&amb_perf_total,
                            0.0,
                            after_write - loop_start,
                            after_read - after_write,
                            after_copy - after_read,
                            after_copy - loop_start);

        double phase1_elapsed = after_copy - phase1_start;
        if (phase1_elapsed >= phase1_next_log) {
            measure_perf_print_interval("AMBIENT", "silence", "copy",
                                        &amb_perf_interval, rate, period);
            measure_perf_reset(&amb_perf_interval);
            phase1_next_log += 1.0;
        }
    }
    phase1_elapsed_sec = get_time() - phase1_start;

    /* Ref mic lead analysis */
    if (amb_count > 0) {
        double ref_rms = sqrt(amb_ref_sq / amb_count);
        double err_rms = sqrt(amb_err_sq / amb_count);
        fprintf(stderr, "Ambient RMS: ref=%.6f  err=%.6f  (%ld samples)\n",
                ref_rms, err_rms, amb_count);

        if (ref_rms < 0.001 && err_rms < 0.001) {
            fprintf(stderr, "WARNING: ambient too weak for reliable ref mic check\n");
        } else {
            float corr_val;
            double phase1_xcorr_start = get_time();
            int lag = xcorr_peak_lag(amb_ref, amb_err, (int)amb_count,
                                     XCORR_MAX_LAG, &corr_val);
            phase1_xcorr_sec = get_time() - phase1_xcorr_start;
            float lag_ms = (float)lag / (float)rate * 1000.0f;
            fprintf(stderr, "Ref vs Err xcorr: lag=%d samples (%.2f ms)  corr=%.6f\n",
                    lag, lag_ms, corr_val);
            if (lag > 0) {
                fprintf(stderr, "  -> error mic delayed %d samples vs ref (ref LEADS) — OK\n", lag);
            } else if (lag < 0) {
                fprintf(stderr, "  -> ref mic delayed %d samples vs error (ref LAGS)\n", -lag);
                fprintf(stderr, "  WARNING: feedforward ANC condition may not be met\n");
            } else {
                fprintf(stderr, "  -> no measurable delay between mics\n");
                fprintf(stderr, "  WARNING: cannot confirm feedforward condition\n");
            }
        }
    }
    measure_perf_print_summary("Phase 1",
                               "silence", "copy",
                               &amb_perf_total,
                               rate, period,
                               phase1_resync_sec,
                               "xcorr", phase1_xcorr_sec,
                               NULL, 0.0,
                               phase1_elapsed_sec + phase1_resync_sec + phase1_xcorr_sec);
    free(amb_ref);
    free(amb_err);
    amb_ref = NULL;
    amb_err = NULL;

    /* Re-sync ALSA between phases (playback may have underrun during setup) */
    double phase2_resync_start = get_time();
    if (alsa_resync(a, out_buf, start_fill_periods) < 0)
        goto cleanup;
    phase2_resync_sec = get_time() - phase2_resync_start;

    /* ========== Phase 2: White noise — secondary path estimation ========== */
    long total_samples = (long)rate * duration_sec;

    /* Ring buffer: keep only last 2 s for post-hoc xcorr (avoids large alloc) */
    long xc_buf_len = (long)rate * 2;
    sig_out = (float *)malloc(xc_buf_len * sizeof(float));
    sig_err = (float *)malloc(xc_buf_len * sizeof(float));
    long sig_count = 0;

    /* LMS state */
    s_hat = (float *)calloc(s_len, sizeof(float));
    cbuf_init(&u_buf, s_len);

    float mu_s = MU_MEASURE;
    float mu_s_max = MU_MEASURE_MAX;
    float u_pow_ema = 0.0f;
    double pred_err_sum_sq = 0;
    int pred_count = 0;
    uint32_t rng_seed = 42;
    double t_start = get_time();
    double next_log = 1.0;

    fprintf(stderr, "\n=== Phase 2: Secondary path measurement (%d sec) ===\n",
            duration_sec);
    fprintf(stderr, "s_len=%d  noise_amp=%.2f  mu_s=%.4f  mu_s_max=%.4f\n",
            s_len, noise_amp, mu_s, mu_s_max);

    while (g_running && sig_count < total_samples) {
        double loop_start = get_time();
        /* Generate white noise output */
        for (int i = 0; i < (int)period; i++) {
            float u = white_noise(&rng_seed) * noise_amp;
            u = output_safety_step(&out_safety, u);
            out_buf[i * 2 + 0] = clip16(u);
            out_buf[i * 2 + 1] = clip16(u);
        }
        double after_gen = get_time();

        int err = pcm_write_full(a->play, out_buf, period);
        double after_write = get_time();
        if (err < 0) {
            xruns++;
            if (alsa_handle_io_error(a, a->play, "playback", err, out_buf,
                                     xrun_fill_periods) < 0)
                break;
            continue;
        }

        err = pcm_read_full(a->cap, in_buf, period);
        double after_read = get_time();
        if (err < 0) {
            xruns++;
            if (alsa_handle_io_error(a, a->cap, "capture", err, out_buf,
                                     xrun_fill_periods) < 0)
                break;
            continue;
        }

        /* Per-sample LMS system identification */
        for (int i = 0; i < (int)period; i++) {
            float u_sample   = to_f(out_buf[i * 2 + 0]);
            float d_measured = to_f(in_buf[i * 2 + ERR_CH]);

            cbuf_push(&u_buf, u_sample);
            float *u = cbuf_ptr(&u_buf);

            float predicted = 0.0f;
            for (int k = 0; k < s_len; k++)
                predicted += s_hat[k] * u[k];

            float pe = d_measured - predicted;

            float u_pow = 0.0f;
            for (int k = 0; k < s_len; k++)
                u_pow += u[k] * u[k];
            if (u_pow_ema == 0.0f) u_pow_ema = u_pow;
            else u_pow_ema = MEASURE_EMA_ALPHA * u_pow_ema +
                             (1.0f - MEASURE_EMA_ALPHA) * u_pow;
            float den = u_pow_ema + MEASURE_EPS_ABS + MEASURE_EPS_REL * noise_amp * noise_amp;
            float mu_n = clampf(mu_s / den, 0.0f, mu_s_max);
            for (int k = 0; k < s_len; k++)
                s_hat[k] += mu_n * pe * u[k];

            pred_err_sum_sq += (double)pe * pe;
            pred_count++;

            sig_out[sig_count % xc_buf_len] = u_sample;
            sig_err[sig_count % xc_buf_len] = d_measured;
            sig_count++;
        }
        double after_compute = get_time();
        measure_perf_update(&meas_perf_interval,
                            after_gen - loop_start,
                            after_write - after_gen,
                            after_read - after_write,
                            after_compute - after_read,
                            after_compute - loop_start);
        measure_perf_update(&meas_perf_total,
                            after_gen - loop_start,
                            after_write - after_gen,
                            after_read - after_write,
                            after_compute - after_read,
                            after_compute - loop_start);

        /* 1-second log */
        double elapsed = after_compute - t_start;
        if (elapsed >= next_log) {
            double pred_err_rms = (pred_count > 0) ?
                sqrt(pred_err_sum_sq / pred_count) : 0;

            int peak_idx = 0;
            float peak_val = 0.0f;
            for (int k = 0; k < s_len; k++) {
                if (fabsf(s_hat[k]) > peak_val) {
                    peak_val = fabsf(s_hat[k]);
                    peak_idx = k;
                }
            }
            float lat_ms = (float)peak_idx / (float)rate * 1000.0f;

            fprintf(stderr, "[MEASURE %5.1fs] pred_err=%.4f  peak_idx=%d (%.2fms)\n",
                    elapsed, pred_err_rms, peak_idx, lat_ms);
            print_sec_path_stats(s_hat, s_len, rate, "S_hat(measure)");
            measure_perf_print_interval("MEASURE", "noise", "lms",
                                        &meas_perf_interval, rate, period);
            measure_perf_reset(&meas_perf_interval);

            pred_err_sum_sq = 0;
            pred_count = 0;
            next_log += 1.0;
        }
    }

    /* ========== Post-measurement analysis ========== */
    fprintf(stderr, "\n=== Latency Analysis ===\n");

    /* Method 1: LMS peak coefficient */
    double phase2_peak_scan_start = get_time();
    int lms_peak = 0;
    float lms_peak_val = 0.0f;
    for (int k = 0; k < s_len; k++) {
        if (fabsf(s_hat[k]) > lms_peak_val) {
            lms_peak_val = fabsf(s_hat[k]);
            lms_peak = k;
        }
    }
    phase2_peak_scan_sec = get_time() - phase2_peak_scan_start;
    float lms_ms = (float)lms_peak / (float)rate * 1000.0f;
    fprintf(stderr, "LMS peak_coeff:   %d samples (%.2f ms)  value=%.4f\n",
            lms_peak, lms_ms, lms_peak_val);

    /* Method 2: Cross-correlation on last 2 sec of data (ring buffer) */
    if (sig_count > 0) {
        int xc_n = (int)((sig_count < xc_buf_len) ? sig_count : xc_buf_len);
        float xc_val;
        double phase2_xcorr_start = get_time();
        int xc_lag;

        if (sig_count <= xc_buf_len) {
            /* Buffer not yet full — data is contiguous from index 0 */
            xc_lag = xcorr_peak_lag(sig_out, sig_err, xc_n, XCORR_MAX_LAG, &xc_val);
        } else {
            /* Ring buffer wrapped — unwrap chronologically into temp buffers */
            float *tmp_out = (float *)malloc(xc_n * sizeof(float));
            float *tmp_err = (float *)malloc(xc_n * sizeof(float));
            if (tmp_out && tmp_err) {
                long oldest = sig_count % xc_buf_len;
                for (int i = 0; i < xc_n; i++) {
                    long idx = (oldest + i) % xc_buf_len;
                    tmp_out[i] = sig_out[idx];
                    tmp_err[i] = sig_err[idx];
                }
                xc_lag = xcorr_peak_lag(tmp_out, tmp_err, xc_n, XCORR_MAX_LAG, &xc_val);
            } else {
                xc_lag = xcorr_peak_lag(sig_out, sig_err, xc_n, XCORR_MAX_LAG, &xc_val);
            }
            free(tmp_out);
            free(tmp_err);
        }
        phase2_xcorr_sec = get_time() - phase2_xcorr_start;
        float xc_ms = (float)xc_lag / (float)rate * 1000.0f;
        fprintf(stderr, "Xcorr latency:    %d samples (%.2f ms)  corr=%.6f\n",
                xc_lag, xc_ms, xc_val);

        int diff = abs(lms_peak - xc_lag);
        if (diff > 2)
            fprintf(stderr, "NOTE: LMS vs xcorr differ by %d samples (%.2f ms)\n",
                    diff, (float)diff / (float)rate * 1000.0f);
        else
            fprintf(stderr, "LMS and xcorr agree (diff=%d)\n", diff);
    }

    /* Summary */
    double total_elapsed = get_time() - t_start;
    fprintf(stderr, "\n=== Measurement Complete ===\n");
    fprintf(stderr, "Duration:     %.1f s\n", total_elapsed);
    fprintf(stderr, "Xruns:        %d\n", xruns);
    print_sec_path_stats(s_hat, s_len, rate, "S_hat(final)");

    double phase2_save_start = get_time();
    if (save_sec_path(s_hat, s_len, rate, out_path) == 0) {
        phase2_save_sec = get_time() - phase2_save_start;
        long fsize = 8 + s_len * (long)sizeof(float);
        fprintf(stderr, "Saved:        %s (%ld bytes)\n", out_path, fsize);
    } else {
        phase2_save_sec = get_time() - phase2_save_start;
    }
    measure_perf_print_summary("Phase 2",
                               "noise", "lms",
                               &meas_perf_total,
                               rate, period,
                               phase2_resync_sec,
                               "analysis", phase2_peak_scan_sec + phase2_xcorr_sec,
                               "save", phase2_save_sec,
                               total_elapsed + phase2_resync_sec + phase2_save_sec);

cleanup:
    free(amb_ref);
    free(amb_err);
    free(sig_out);
    free(sig_err);
    free(s_hat);
    cbuf_free(&u_buf);
    free(in_buf);
    free(out_buf);
}

/* ===== ANC Run Mode ===== */
static void run_anc(alsa_ctx_t *a, mbfxlms_t *m, logger_t *l, int no_adapt,
                    float band_lo_hz, float band_hi_hz,
                    int start_fill_periods, int xrun_fill_periods,
                    int adapt_delay_ms)
{
    snd_pcm_uframes_t period = a->period;
    double period_budget_ms = (double)period * 1000.0 / (double)a->rate;

    int16_t *in_buf  = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    int16_t *out_buf = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    band_ctl_t err_mon;
    output_safety_t out_safety;
    band_ctl_init(&err_mon, band_lo_hz, band_hi_hz, (float)a->rate);
    output_safety_init(&out_safety, (float)a->rate);

    /* Ensure clean start state */
    if (alsa_resync(a, out_buf, start_fill_periods) < 0) {
        free(in_buf);
        free(out_buf);
        return;
    }

    double t_start = get_time();

    /* --- Baseline phase: first BASELINE_SECS with adapt=0 --- */
    int    warmup_done = 0;
    int    warmup_sec  = 0;
    int    bl_done = 0;
    int    bl_sec  = 0;
    int    adapt_delay_active = 0;
    long   adapt_delay_frames = 0;
    double bl_err_sum = 0;
    double bl_tone_sum = 0;
    tone_meter_t baseline_track;
    long   bl_count  = 0;
    double prev_metric_db = 0.0;
    double best_metric_db = -1e9;
    int    best_snapshot_valid = 0;
    float *best_w[MBAND_BANDS_MAX] = {0};
    tone_meter_init(&baseline_track, l->tone_track.freq_hz, (float)a->rate);

    for (int i = 0; i < m->n_bands; i++) {
        best_w[i] = (float *)calloc((size_t)m->bands[i].w_len, sizeof(float));
        if (!best_w[i]) {
            fprintf(stderr, "ERROR: unable to allocate best-weight snapshot for band %d\n", i);
            for (int j = 0; j < i; j++) {
                free(best_w[j]);
                best_w[j] = NULL;
            }
            free(in_buf);
            free(out_buf);
            return;
        }
    }

    mbfxlms_set_adapt(m, 0);
    fprintf(stderr, "ANC: multi-band=%d  w_len=%d  s_len=%d  mu=%.6f  leak=%.4f  "
            "baseline=%ds  band=%.0f~%.0fHz  tone_track=%.1fHz  no_adapt=%d"
            "  period=%lu  buffer=%lu  start_fill=%d  xrun_fill=%d"
            "  adapt_delay_ms=%d\n",
            m->n_bands, m->bands[0].w_len, m->bands[0].s_len,
            mbfxlms_mu(m), m->bands[0].leakage, BASELINE_SECS,
            band_lo_hz, band_hi_hz, l->tone_track.freq_hz, no_adapt,
            (unsigned long)a->period, (unsigned long)a->buf_size,
            start_fill_periods, xrun_fill_periods, adapt_delay_ms);
    mbfxlms_print_bands(m);

    /* out_buf already zeroed (calloc) — first write outputs silence */
    while (g_running) {
        double loop_start = get_time();
        int err = pcm_write_full(a->play, out_buf, period);
        double after_write = get_time();
        if (err < 0) {
            logger_xrun(l);
            if (alsa_handle_io_error(a, a->play, "playback", err, out_buf,
                                     xrun_fill_periods) < 0)
                break;
            continue;
        }

        err = pcm_read_full(a->cap, in_buf, period);
        double after_read = get_time();
        if (err < 0) {
            logger_xrun(l);
            if (alsa_handle_io_error(a, a->cap, "capture", err, out_buf,
                                     xrun_fill_periods) < 0)
                break;
            continue;
        }

        for (int i = 0; i < (int)period; i++) {
            float ref  = to_f(in_buf[i * 2 + REF_CH]);
            float e    = to_f(in_buf[i * 2 + ERR_CH]);
            float e_tone = band_ctl_step(&err_mon, e);
            float anti = mbfxlms_step_sample(m, ref, e);
            anti = output_safety_step(&out_safety, anti);

            /* Soft limiter + clip count */
            int clipped = 0;
            if (anti >  OUTPUT_LIMIT) { anti =  OUTPUT_LIMIT; clipped = 1; }
            if (anti < -OUTPUT_LIMIT) { anti = -OUTPUT_LIMIT; clipped = 1; }

            out_buf[i * 2 + 0] = clip16(anti);
            out_buf[i * 2 + 1] = clip16(anti);

            logger_update(l, e, anti, e_tone, clipped);

            /* Accumulate baseline after band filters settle */
            if (warmup_done && !bl_done) {
                bl_err_sum += (double)e * e;
                bl_tone_sum += (double)e_tone * e_tone;
                tone_meter_step(&baseline_track, e);
                bl_count++;
            }
        }
        double after_compute = get_time();

        logger_period_times(l,
                            after_read - after_write,
                            after_compute - after_read,
                            after_write - loop_start,
                            after_compute - loop_start);

        if (adapt_delay_active && !l->frozen) {
            adapt_delay_frames -= period;
            if (adapt_delay_frames <= 0) {
                adapt_delay_active = 0;
                mbfxlms_set_adapt(m, 1);
                fprintf(stderr, "=== Adaptation ON (delay %d ms) ===\n",
                        adapt_delay_ms);
            }
        }

        /* --- 1-second reporting --- */
        if (logger_should_print(l)) {
            double elapsed = get_time() - t_start;
            float  wn = mbfxlms_w_norm(m);

            /* Build status tag BEFORE baseline transition */
            char status[32];
            if (l->frozen)
                snprintf(status, sizeof(status), "FROZEN");
            else if (!warmup_done)
                snprintf(status, sizeof(status), "WU%d/%d",
                         warmup_sec + 1, STARTUP_WARMUP_SECS);
            else if (!bl_done)
                snprintf(status, sizeof(status), "BL%d/%d",
                         bl_sec + 1, BASELINE_SECS);
            else if (adapt_delay_active)
                snprintf(status, sizeof(status), "ADLY");
            else if (m->bands[0].adapt)
                snprintf(status, sizeof(status), "adapt");
            else
                snprintf(status, sizeof(status), "fixed");

            /* Print interval (returns err_rms, writes clip_frac) */
            double clip_frac = 0;
            double tone_db = 0;
            double track_db = 0;
            double cur_err_rms = logger_print(l, wn, elapsed,
                                              mbfxlms_mu(m),
                                              mbfxlms_mu_n_avg(m),
                                              mbfxlms_fx_pow_sum(m),
                                              mbfxlms_x_pow_sum(m),
                                              m->n_bands,
                                              m->bands[0].w_len,
                                              m->bands[0].s_len,
                                              period_budget_ms,
                                              status,
                                              &clip_frac, &tone_db, &track_db);
            double metric_db = tone_db;
            if (l->baseline_track_rms > 0.0)
                metric_db = track_db;

            if (bl_done && !l->frozen && m->bands[0].adapt &&
                metric_db > 0.0 && metric_db > best_metric_db) {
                for (int i = 0; i < m->n_bands; i++) {
                    memcpy(best_w[i], m->bands[i].w,
                           (size_t)m->bands[i].w_len * sizeof(float));
                }
                best_metric_db = metric_db;
                best_snapshot_valid = 1;
            }

            /* Baseline phase tracking (AFTER print so tag is correct) */
            if (!warmup_done) {
                warmup_sec++;
                if (warmup_sec >= STARTUP_WARMUP_SECS) {
                    warmup_done = 1;
                    fprintf(stderr, "=== Warm-up done, baseline capture start ===\n");
                }
            } else if (!bl_done) {
                bl_sec++;
                if (bl_sec >= BASELINE_SECS) {
                    l->baseline_err_rms = sqrt(bl_err_sum / bl_count);
                    l->baseline_tone_rms = sqrt(bl_tone_sum / bl_count);
                    l->baseline_track_rms = tone_meter_rms(&baseline_track);
                    bl_done = 1;
                    fprintf(stderr, "=== Baseline: err_rms=%.6f "
                            " tone_rms=%.6f  track@%.1f=%.6f (%d sec, %ld samples) ===\n",
                            l->baseline_err_rms, l->baseline_tone_rms,
                            l->tone_track.freq_hz, l->baseline_track_rms,
                            BASELINE_SECS, bl_count);
                    if (!no_adapt) {
                        if (adapt_delay_ms > 0) {
                            adapt_delay_active = 1;
                            adapt_delay_frames =
                                ((long)a->rate * adapt_delay_ms + 999L) / 1000L;
                            fprintf(stderr,
                                    "=== Adaptation delay: %d ms (%ld frames) ===\n",
                                    adapt_delay_ms, adapt_delay_frames);
                        } else {
                            mbfxlms_set_adapt(m, 1);
                            fprintf(stderr, "=== Adaptation ON ===\n");
                        }
                    } else {
                        fprintf(stderr, "=== Adaptation OFF (--no-adapt) ===\n");
                    }
                }
            }

            /* Divergence detection (only after baseline, only if adapting) */
            if (bl_done && m->bands[0].adapt && !l->frozen) {
                int causes = 0;
                if (wn > W_NORM_MAX)
                    causes |= 1;
                if (l->baseline_err_rms > 0 &&
                    cur_err_rms > l->baseline_err_rms * ERR_DIVERGE)
                    causes |= 2;
                if (clip_frac > CLIP_DIVERGE_FRAC)
                    causes |= 4;

                if (causes) {
                    fprintf(stderr, "DIVERGENCE [");
                    if (causes & 1)
                        fprintf(stderr, " w_norm=%.1f>%.1f",
                                wn, W_NORM_MAX);
                    if (causes & 2)
                        fprintf(stderr, " err=%.4f>base*%.1f",
                                cur_err_rms, ERR_DIVERGE);
                    if (causes & 4)
                        fprintf(stderr, " clip=%.1f%%>%.1f%%",
                                clip_frac * 100.0, CLIP_DIVERGE_FRAC * 100.0);
                    if (best_snapshot_valid) {
                        fprintf(stderr, " ] -> restore best fixed weights (%+.1fdB)\n",
                                best_metric_db);
                        for (int i = 0; i < m->n_bands; i++) {
                            memcpy(m->bands[i].w, best_w[i],
                                   (size_t)m->bands[i].w_len * sizeof(float));
                        }
                        mbfxlms_set_adapt(m, 0);
                        mbfxlms_reset_state(m);
                        adapt_delay_active = 0;
                        adapt_delay_frames = 0;
                        l->frozen = 0;
                        l->recover_sec = 0;
                    } else {
                        fprintf(stderr, " ] -> reset + FROZEN\n");
                        mbfxlms_set_adapt(m, 0);
                        mbfxlms_scale_mu(m, TUNE_DOWN_RATE);
                        mbfxlms_scale_mu_n_max(m, 0.75f);
                        mbfxlms_reset_w(m);
                        adapt_delay_active = 0;
                        adapt_delay_frames = 0;
                        l->frozen = 1;
                        l->recover_sec = RECOVER_SECS;
                    }
                }
            }

            if (bl_done && !no_adapt) {
                if (l->frozen) {
                    if (l->recover_sec > 0) {
                        fprintf(stderr, "Recovery cooldown: %d sec (mu=%.5f)\n",
                                l->recover_sec, mbfxlms_mu(m));
                        l->recover_sec--;
                    }
                    if (l->recover_sec <= 0) {
                        l->frozen = 0;
                        if (adapt_delay_ms > 0) {
                            mbfxlms_scale_mu_n_max(m, 1.10f);
                            adapt_delay_active = 1;
                            adapt_delay_frames =
                                ((long)a->rate * adapt_delay_ms + 999L) / 1000L;
                            fprintf(stderr,
                                    "=== Recovery delay: %d ms (%ld frames), mu=%.5f ===\n",
                                    adapt_delay_ms, adapt_delay_frames,
                                    mbfxlms_mu(m));
                        } else {
                            mbfxlms_set_adapt(m, 1);
                            mbfxlms_scale_mu_n_max(m, 1.10f);
                            fprintf(stderr, "=== Recovery resume: mu=%.5f ===\n",
                                    mbfxlms_mu(m));
                        }
                    }
                } else if (m->bands[0].adapt) {
                    if (clip_frac > CLIP_WARN_FRAC || metric_db < prev_metric_db - 1.0) {
                        mbfxlms_scale_mu(m, TUNE_DOWN_RATE);
                        mbfxlms_scale_mu_n_max(m, 0.92f);
                    } else if (metric_db > TARGET_REDUCTION_DB + 3.0) {
                        mbfxlms_scale_mu(m, 0.98f);
                        mbfxlms_scale_mu_n_max(m, 0.90f);
                    } else if (metric_db > TARGET_REDUCTION_DB) {
                        mbfxlms_scale_mu_n_max(m, 0.96f);
                    } else if (metric_db > 2.0 &&
                               metric_db > prev_metric_db + 0.5 &&
                               clip_frac < CLIP_WARN_FRAC * 0.5) {
                        mbfxlms_scale_mu(m, 1.03f);
                        mbfxlms_scale_mu_n_max(m, 1.02f);
                    }
                }
            }

            prev_metric_db = metric_db;
        }
    }

    free(in_buf);
    free(out_buf);
    for (int i = 0; i < m->n_bands; i++)
        free(best_w[i]);
}

/* ===== Narrowband ANC Mode ===== */
static void run_nb_anc(alsa_ctx_t *a, const float *sec_path, int sec_len,
                       logger_t *l, int n_harm, float nb_mu, float nb_leak,
                       int start_fill_periods, int xrun_fill_periods)
{
    snd_pcm_uframes_t period = a->period;
    double period_budget_ms = (double)period * 1000.0 / (double)a->rate;

    int16_t *in_buf  = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    int16_t *out_buf = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    output_safety_t out_safety;
    output_safety_init(&out_safety, (float)a->rate);

    nb_anc_t nb;
    nb_init(&nb, n_harm, nb_mu, nb_leak, sec_path, sec_len, (float)a->rate);

    if (alsa_resync(a, out_buf, start_fill_periods) < 0) {
        free(in_buf); free(out_buf); return;
    }

    double t_start = get_time();

    /* Baseline phase */
    int    bl_done = 0, bl_sec = 0;
    double bl_err_sum = 0;
    long   bl_count = 0;
    double baseline_err_rms = 0;

    /* Best weights snapshot */
    float  best_wc[NB_MAX_HARM] = {0}, best_ws[NB_MAX_HARM] = {0};
    float  best_f0 = 0;
    double best_db = -1e9;
    int    best_valid = 0;

    fprintf(stderr, "NB-ANC: n_harm=%d  mu=%.6f  leak=%.6f  sec_len=%d  "
            "f0_range=%.0f~%.0fHz  period=%lu  budget=%.3fms\n",
            n_harm, nb_mu, nb_leak, sec_len,
            NB_F0_MIN_HZ, NB_F0_MAX_HZ,
            (unsigned long)period, period_budget_ms);

    /* out_buf already zeroed (calloc) — first write outputs silence */
    while (g_running) {
        double loop_start = get_time();
        /* Write-before-read: consistent with sec_path measurement timing.
         * Outputs the result computed in the PREVIOUS iteration (silence on
         * first iteration).  This also prevents playback underrun on linked
         * streams before the first capture period is ready. */
        int err = pcm_write_full(a->play, out_buf, period);
        double after_write = get_time();
        if (err < 0) {
            logger_xrun(l);
            if (alsa_handle_io_error(a, a->play, "playback", err, out_buf,
                                     xrun_fill_periods) < 0)
                break;
            continue;
        }

        err = pcm_read_full(a->cap, in_buf, period);
        double after_read = get_time();
        if (err < 0) {
            logger_xrun(l);
            if (alsa_handle_io_error(a, a->cap, "capture", err, out_buf,
                                     xrun_fill_periods) < 0)
                break;
            continue;
        }

        /* Buffer ref samples for f0 tracker (cheap) */
        for (int i = 0; i < (int)period; i++)
            nb_f0_buf_sample(&nb, to_f(in_buf[i * 2 + REF_CH]));

        /* f0 detection (expensive, runs ~every 200ms, between periods) */
        int f0_changed = nb_f0_update(&nb);
        if (f0_changed && nb.f0_hz > 0.0f)
            fprintf(stderr, "  f0=%.1fHz conf=%.2f\n", nb.f0_hz, nb.f0_conf);

        for (int i = 0; i < (int)period; i++) {
            float e   = to_f(in_buf[i * 2 + ERR_CH]);

            /* Narrowband anti-noise */
            float anti = nb_step(&nb, e);
            anti = output_safety_step(&out_safety, anti);

            /* Soft limiter */
            int clipped = 0;
            if (anti >  OUTPUT_LIMIT) { anti =  OUTPUT_LIMIT; clipped = 1; }
            if (anti < -OUTPUT_LIMIT) { anti = -OUTPUT_LIMIT; clipped = 1; }

            out_buf[i * 2 + 0] = clip16(anti);
            out_buf[i * 2 + 1] = clip16(anti);

            logger_update(l, e, anti, e, clipped);

            /* Baseline accumulation */
            if (!bl_done) {
                bl_err_sum += (double)e * e;
                bl_count++;
            }
        }
        double after_compute = get_time();

        logger_period_times(l,
                            after_read - after_write,   /* read = blocking wait */
                            after_compute - after_read, /* compute */
                            after_write - loop_start,   /* write = non-blocking */
                            after_compute - loop_start);

        /* 1-second reporting */
        if (logger_should_print(l)) {
            double elapsed = get_time() - t_start;
            float  wn = nb_w_norm(&nb);
            double clip_frac = 0;
            double tone_db = 0, track_db = 0;

            char status[32];
            if (l->frozen) snprintf(status, sizeof(status), "FROZEN");
            else if (!bl_done) snprintf(status, sizeof(status), "BL%d/%d", bl_sec + 1, BASELINE_SECS);
            else if (nb.adapt) snprintf(status, sizeof(status), "adapt");
            else snprintf(status, sizeof(status), "fixed");

            /* compute effective NLMS mu for logging */
            float nb_sx = (nb.f0_hz > 0 && nb.harm[0].sx_mag > 0)
                          ? nb.harm[0].sx_mag : 0.0f;
            float nb_mu_n = nb.mu / (nb_sx * nb_sx + NB_NLMS_EPS);
            double cur_err_rms = logger_print(l, wn, elapsed, nb.mu, (double)nb_mu_n,
                                              0.0, 0.0,
                                              n_harm, 2, sec_len,
                                              period_budget_ms, status,
                                              &clip_frac, &tone_db, &track_db);

            /* Extra NB info */
            fprintf(stderr, "  nb: f0=%.1fHz conf=%.2f harms=",
                    nb.f0_hz, nb.f0_conf);
            for (int h = 0; h < nb.n_harm; h++) {
                if (nb.harm[h].freq_hz > 0)
                    fprintf(stderr, "%.0f(%.4f) ", nb.harm[h].freq_hz,
                            sqrtf(nb.harm[h].w_c * nb.harm[h].w_c +
                                  nb.harm[h].w_s * nb.harm[h].w_s));
            }
            fprintf(stderr, "\n");

            if (!bl_done) {
                bl_sec++;
                if (bl_sec >= BASELINE_SECS) {
                    baseline_err_rms = sqrt(bl_err_sum / bl_count);
                    l->baseline_err_rms = baseline_err_rms;
                    l->baseline_tone_rms = baseline_err_rms;
                    bl_done = 1;
                    nb.adapt = 1;
                    fprintf(stderr, "=== Baseline: err_rms=%.6f (%d sec) -> Adaptation ON ===\n",
                            baseline_err_rms, BASELINE_SECS);
                }
            }

            /* Track best weights */
            if (bl_done && nb.adapt && tone_db > 0.0 && tone_db > best_db) {
                for (int h = 0; h < nb.n_harm; h++) {
                    best_wc[h] = nb.harm[h].w_c;
                    best_ws[h] = nb.harm[h].w_s;
                }
                best_f0 = nb.f0_hz;
                best_db = tone_db;
                best_valid = 1;
            }

            /* Divergence detection */
            if (bl_done && nb.adapt && !l->frozen) {
                int diverged = 0;
                if (baseline_err_rms > 0 &&
                    cur_err_rms > baseline_err_rms * ERR_DIVERGE) {
                    diverged = 1;
                    fprintf(stderr, "DIVERGENCE [ err=%.4f>base*%.1f ]",
                            cur_err_rms, ERR_DIVERGE);
                }
                if (wn > W_NORM_MAX) {
                    diverged = 1;
                    fprintf(stderr, "DIVERGENCE [ w_norm=%.1f>%.1f ]",
                            wn, W_NORM_MAX);
                }
                if (diverged) {
                    if (best_valid) {
                        fprintf(stderr, " -> restore best (%+.1fdB @ f0=%.0fHz)\n",
                                best_db, best_f0);
                        for (int h = 0; h < nb.n_harm; h++) {
                            nb.harm[h].w_c = best_wc[h];
                            nb.harm[h].w_s = best_ws[h];
                        }
                        nb.adapt = 0;
                        l->frozen = 0;
                        l->recover_sec = RECOVER_SECS;
                    } else {
                        fprintf(stderr, " -> reset + FROZEN\n");
                        nb_reset_weights(&nb);
                        nb.adapt = 0;
                        l->frozen = 1;
                        l->recover_sec = RECOVER_SECS;
                    }
                }
            }

            /* Recovery */
            if (bl_done && l->frozen) {
                if (l->recover_sec > 0) l->recover_sec--;
                if (l->recover_sec <= 0) {
                    l->frozen = 0;
                    nb.adapt = 1;
                    fprintf(stderr, "=== Recovery: adapt ON ===\n");
                }
            } else if (bl_done && !nb.adapt && !l->frozen) {
                if (l->recover_sec > 0) l->recover_sec--;
                if (l->recover_sec <= 0) {
                    nb.adapt = 1;
                    fprintf(stderr, "=== Re-adapt ON ===\n");
                }
            }
        }
    }

    free(in_buf);
    free(out_buf);
}

/* ===== Passthrough Mode ===== */
static void run_passthrough(alsa_ctx_t *a, logger_t *l,
                            int start_fill_periods, int xrun_fill_periods)
{
    snd_pcm_uframes_t period = a->period;

    int16_t *in_buf  = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    int16_t *out_buf = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));

    if (alsa_resync(a, out_buf, start_fill_periods) < 0) {
        free(in_buf);
        free(out_buf);
        return;
    }

    double t_start = get_time();
    fprintf(stderr, "Passthrough: capture -> playback echo-back\n");

    while (g_running) {
        double loop_start = get_time();
        int err = pcm_read_full(a->cap, in_buf, period);
        double after_read = get_time();
        if (err < 0) {
            logger_xrun(l);
            if (alsa_handle_io_error(a, a->cap, "capture", err, out_buf,
                                     xrun_fill_periods) < 0)
                break;
            continue;
        }

        for (int i = 0; i < (int)period; i++) {
            out_buf[i * 2 + 0] = in_buf[i * 2 + 0];
            out_buf[i * 2 + 1] = in_buf[i * 2 + 1];

            float e    = to_f(in_buf[i * 2 + 0]);
            float anti = to_f(out_buf[i * 2 + 0]);
            logger_update(l, e, anti, e, 0);
        }
        double after_compute = get_time();

        err = pcm_write_full(a->play, out_buf, period);
        double after_write = get_time();
        logger_period_times(l,
                            after_read - loop_start,
                            after_compute - after_read,
                            after_write - after_compute,
                            after_write - loop_start);
        if (err < 0) {
            logger_xrun(l);
            if (alsa_handle_io_error(a, a->play, "playback", err, out_buf,
                                     xrun_fill_periods) < 0)
                break;
        }

        if (logger_should_print(l)) {
            double elapsed = get_time() - t_start;
            logger_print(l, 0.0f, elapsed, 0.0f, 0.0f, 0.0f, 0.0f,
                         0, 0, 0,
                         (double)period * 1000.0 / (double)a->rate,
                         "pass", NULL, NULL, NULL);
        }
    }

    free(in_buf);
    free(out_buf);
}

/* ===== Simulation Mode ===== */
static void build_primary_path(float *p, int p_len, const float *s_hat, int s_len)
{
    memset(p, 0, p_len * sizeof(float));
    for (int k = 0; k < s_len; k++) {
        if (k + 120 < p_len)
            p[k + 120] += 0.95f * s_hat[k];
        if (k + 210 < p_len)
            p[k + 210] -= 0.18f * s_hat[k];
    }
}

static void build_secondary_true(const float *s_hat, float *s_true, int s_len)
{
    memset(s_true, 0, s_len * sizeof(float));
    for (int k = 0; k < s_len; k++) {
        int dst = k + 2;
        if (dst < s_len)
            s_true[dst] += 1.15f * s_hat[k];
        if (dst + 5 < s_len)
            s_true[dst + 5] += 0.08f * s_hat[k];
    }
}

static double simulate_one_freq(const float *s_hat, int s_len, float freq_hz,
                                float mu, float leakage, float mu_n_max,
                                float eps_abs, float eps_rel,
                                float band_lo_hz, float band_hi_hz,
                                int mb_bands)
{
    const int p_len = 1536;
    const int warmup_n = SAMPLE_RATE * SIM_WARMUP_SECS;
    const int total_n = SAMPLE_RATE * SIM_SECS;
    const int baseline_n = SAMPLE_RATE * SIM_BASELINE_SECS;
    const int tail_n = SAMPLE_RATE * 2;
    uint32_t seed = 7;

    float *p = (float *)calloc(p_len, sizeof(float));
    float *s_true = (float *)calloc(s_len, sizeof(float));
    build_primary_path(p, p_len, s_hat, s_len);
    build_secondary_true(s_hat, s_true, s_len);

    cbuf_t x_plant;
    cbuf_t y_plant;
    cbuf_init(&x_plant, p_len);
    cbuf_init(&y_plant, s_len);

    mbfxlms_t m;
    band_ctl_t err_mon;
    mbfxlms_init(&m, mb_bands, W_LEN_DEFAULT, s_len, mu, leakage,
                 band_lo_hz, band_hi_hz);
    mbfxlms_set_mu_n_max(&m, mu_n_max);
    mbfxlms_set_eps(&m, eps_abs, eps_rel);
    mbfxlms_set_adapt(&m, 0);
    for (int i = 0; i < m.n_bands; i++)
        memcpy(m.bands[i].s, s_hat, s_len * sizeof(float));
    band_ctl_init(&err_mon, band_lo_hz, band_hi_hz, SAMPLE_RATE);

    double base_sq = 0.0;
    double tail_sq = 0.0;
    int tail_count = 0;

    for (int n = 0; n < total_n; n++) {
        float t = (float)n / (float)SAMPLE_RATE;
        float x = 0.45f * sinf(2.0f * (float)M_PI * freq_hz * t);
        x += 0.03f * sinf(2.0f * (float)M_PI * (freq_hz * 0.5f) * t);
        x += 0.01f * white_noise(&seed);

        cbuf_push(&x_plant, x);
        float *xp = cbuf_ptr(&x_plant);
        float d = 0.0f;
        for (int k = 0; k < p_len; k++)
            d += p[k] * xp[k];

        float *yp = cbuf_ptr(&y_plant);
        float anti_path = 0.0f;
        for (int k = 0; k < s_len; k++)
            anti_path += s_true[k] * yp[k];

        float e = d + anti_path;
        float e_tone = band_ctl_step(&err_mon, e);
        float anti = mbfxlms_step_sample(&m, x, e);
        anti = clampf(anti, -OUTPUT_LIMIT, OUTPUT_LIMIT);
        cbuf_push(&y_plant, anti);

        if (n >= warmup_n && n < warmup_n + baseline_n) {
            base_sq += (double)e_tone * e_tone;
            if (n == warmup_n + baseline_n - 1)
                mbfxlms_set_adapt(&m, 1);
        } else if (n >= total_n - tail_n) {
            tail_sq += (double)e_tone * e_tone;
            tail_count++;
        }
    }

    double baseline_rms = sqrt(base_sq / baseline_n);
    double tail_rms = sqrt(tail_sq / (tail_count > 0 ? tail_count : 1));
    double improve_db = db_improvement(baseline_rms, tail_rms);

    mbfxlms_free(&m);
    cbuf_free(&x_plant);
    cbuf_free(&y_plant);
    free(p);
    free(s_true);
    return improve_db;
}

static int run_sim_sweep(float tone_hz,
                          float band_lo_hz, float band_hi_hz, int mb_bands)
{
    fxlms_t tmp;
    fxlms_init(&tmp, W_LEN_DEFAULT, S_LEN_DEFAULT, MU_DEFAULT, LEAKAGE_DEFAULT);
    if (fxlms_load_sec_default(&tmp) < 0) {
        fprintf(stderr, "ERROR: missing %s. run './anc measure' first.\n", SEC_PATH_FILE);
        fxlms_free(&tmp);
        return -1;
    }

    const float default_freqs[] = { 80.0f, 100.0f, 120.0f, 150.0f };
    float single_freq[1];
    const float *freqs = default_freqs;
    int n_freq = (int)(sizeof(default_freqs) / sizeof(default_freqs[0]));
    if (tone_hz > 0.0f) {
        single_freq[0] = tone_hz;
        freqs = single_freq;
        n_freq = 1;
    }

    {
        const float mu_vals[] = { 0.004f, 0.006f, 0.008f, 0.010f, 0.012f };
        const float leak_vals[] = { 0.99999f, 0.999995f, 0.999997f };
        const float mu_n_vals[] = { 0.02f, 0.03f, 0.04f };
        const float eps_rel_vals[] = { 0.01f, 0.02f, 0.03f };

        double best_worst = -1e9;
        float best_mu = MU_DEFAULT;
        float best_leak = LEAKAGE_DEFAULT;
        float best_mu_n = MU_N_MAX;
        float best_eps_rel = EPSILON_REL;

        for (int mi = 0; mi < (int)(sizeof(mu_vals) / sizeof(mu_vals[0])); mi++) {
            for (int li = 0; li < (int)(sizeof(leak_vals) / sizeof(leak_vals[0])); li++) {
                for (int ni = 0; ni < (int)(sizeof(mu_n_vals) / sizeof(mu_n_vals[0])); ni++) {
                    for (int ei = 0; ei < (int)(sizeof(eps_rel_vals) / sizeof(eps_rel_vals[0])); ei++) {
                        double worst_db = 1e9;
                        for (int fi = 0; fi < n_freq; fi++) {
                            double db = simulate_one_freq(tmp.s, tmp.s_len, freqs[fi],
                                                          mu_vals[mi], leak_vals[li],
                                                          mu_n_vals[ni], EPSILON_ABS,
                                                          eps_rel_vals[ei],
                                                          band_lo_hz, band_hi_hz,
                                                          mb_bands);
                            if (db < worst_db) worst_db = db;
                        }
                        if (worst_db > best_worst) {
                            best_worst = worst_db;
                            best_mu = mu_vals[mi];
                            best_leak = leak_vals[li];
                            best_mu_n = mu_n_vals[ni];
                            best_eps_rel = eps_rel_vals[ei];
                        }
                    }
                }
            }
        }

        fprintf(stderr,
                "[SIM sweep] best worst-case=%+.2fdB  mu=%.5f  leak=%.6f  mu_n_max=%.3f  eps_rel=%.3f\n",
                best_worst, best_mu, best_leak, best_mu_n, best_eps_rel);
    }

    fxlms_free(&tmp);
    return 0;
}

typedef struct {
    float mu;
    float leak;
    float mu_n_max;
    float eps_abs;
    float eps_rel;
    float tone_hz;
    float band_lo_hz;
    float band_hi_hz;
    int   mb_bands;
    int   req_period;
    int   buffer_mult;
    int   start_fill_periods;
    int   xrun_fill_periods;
    int   adapt_delay_ms;
    int   measure_secs;
    int   capture_secs;
    float measure_noise_amp;
    int   nb_harms;
    float nb_mu;
    float nb_leak;
} tune_cfg_t;

static void tune_cfg_init(tune_cfg_t *cfg)
{
    cfg->mu = MU_DEFAULT;
    cfg->leak = LEAKAGE_DEFAULT;
    cfg->mu_n_max = MU_N_MAX;
    cfg->eps_abs = EPSILON_ABS;
    cfg->eps_rel = EPSILON_REL;
    cfg->tone_hz = TONE_TRACK_HZ_DEFAULT;
    cfg->band_lo_hz = TARGET_BAND_LO_HZ;
    cfg->band_hi_hz = TARGET_BAND_HI_HZ;
    cfg->mb_bands = MBAND_BANDS_DEFAULT;
    cfg->req_period = REQ_PERIOD;
    cfg->buffer_mult = REQ_BUFFER / REQ_PERIOD;
    cfg->start_fill_periods = START_FILL_PERIODS;
    cfg->xrun_fill_periods = XRUN_FILL_PERIODS;
    cfg->adapt_delay_ms = ADAPT_DELAY_MS_DEFAULT;
    cfg->measure_secs = MEASURE_SECS;
    cfg->capture_secs = CAPTURE_BENCH_SECS_DEFAULT;
    cfg->measure_noise_amp = NOISE_AMP;
    cfg->nb_harms = 4;
    cfg->nb_mu    = NB_MU_DEFAULT;
    cfg->nb_leak  = NB_LEAK_DEFAULT;
}

/* ===== main ===== */
static void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s <mode> [options]\n"
            "  passthrough           capture -> playback echo-back\n"
            "  capture-bench         capture-only I/O timing benchmark\n"
            "  measure               estimate secondary path -> %s\n"
            "  run [--no-adapt]      FxLMS active noise cancellation\n"
            "  nb                    narrowband ANC (pitch-tracking)\n"
            "  sim [--sweep]         offline tuning check\n"
            "Options: --mu=F --leak=F --mu-n-max=F --eps-abs=F --eps-rel=F\n"
            "         --tone-hz=F --band-lo=F --band-hi=F --mb-bands=N\n"
            "         --period=N --buffer-mult=N --start-fill=N --xrun-fill=N\n"
            "         --adapt-delay-ms=N --n-harm=N --nb-mu=F --nb-leak=F\n"
            "         --measure-secs=N --capture-secs=N --measure-noise=F\n",
            prog, SEC_PATH_FILE);
}

int main(int argc, char **argv)
{
    if (argc < 2) { usage(argv[0]); return 1; }

    const char *mode = argv[1];
    int no_adapt = 0;
    int sim_sweep = 0;
    tune_cfg_t cfg;
    tune_cfg_init(&cfg);

    for (int i = 2; i < argc; i++) {
        const char *arg = argv[i];
        if (strcmp(arg, "--no-adapt") == 0) {
            no_adapt = 1;
        } else if (strcmp(arg, "--sweep") == 0) {
            sim_sweep = 1;
        } else if (strncmp(arg, "--mu=", 5) == 0) {
            cfg.mu = strtof(arg + 5, NULL);
        } else if (strncmp(arg, "--leak=", 7) == 0) {
            cfg.leak = strtof(arg + 7, NULL);
        } else if (strncmp(arg, "--mu-n-max=", 11) == 0) {
            cfg.mu_n_max = strtof(arg + 11, NULL);
        } else if (strncmp(arg, "--eps-abs=", 10) == 0) {
            cfg.eps_abs = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--eps-rel=", 10) == 0) {
            cfg.eps_rel = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--tone-hz=", 10) == 0) {
            cfg.tone_hz = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--band-lo=", 10) == 0) {
            cfg.band_lo_hz = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--band-hi=", 10) == 0) {
            cfg.band_hi_hz = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--mb-bands=", 11) == 0) {
            cfg.mb_bands = atoi(arg + 11);
        } else if (strncmp(arg, "--period=", 9) == 0) {
            cfg.req_period = atoi(arg + 9);
        } else if (strncmp(arg, "--buffer-mult=", 14) == 0) {
            cfg.buffer_mult = atoi(arg + 14);
        } else if (strncmp(arg, "--start-fill=", 13) == 0) {
            cfg.start_fill_periods = atoi(arg + 13);
        } else if (strncmp(arg, "--xrun-fill=", 12) == 0) {
            cfg.xrun_fill_periods = atoi(arg + 12);
        } else if (strncmp(arg, "--adapt-delay-ms=", 17) == 0) {
            cfg.adapt_delay_ms = atoi(arg + 17);
        } else if (strncmp(arg, "--measure-secs=", 15) == 0) {
            cfg.measure_secs = atoi(arg + 15);
        } else if (strncmp(arg, "--capture-secs=", 15) == 0) {
            cfg.capture_secs = atoi(arg + 15);
        } else if (strncmp(arg, "--measure-noise=", 16) == 0) {
            cfg.measure_noise_amp = strtof(arg + 16, NULL);
        } else if (strncmp(arg, "--n-harm=", 9) == 0) {
            cfg.nb_harms = atoi(arg + 9);
        } else if (strncmp(arg, "--nb-mu=", 8) == 0) {
            cfg.nb_mu = strtof(arg + 8, NULL);
        } else if (strncmp(arg, "--nb-leak=", 10) == 0) {
            cfg.nb_leak = strtof(arg + 10, NULL);
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (cfg.band_lo_hz <= 0.0f || cfg.band_hi_hz <= cfg.band_lo_hz) {
        fprintf(stderr, "ERROR: invalid band range %.1f~%.1f Hz\n",
                cfg.band_lo_hz, cfg.band_hi_hz);
        return 1;
    }
    if (cfg.mb_bands < 1 || cfg.mb_bands > MBAND_BANDS_MAX) {
        fprintf(stderr, "ERROR: invalid mb-bands %d (valid 1..%d)\n",
                cfg.mb_bands, MBAND_BANDS_MAX);
        return 1;
    }
    if (cfg.req_period < 16) {
        fprintf(stderr, "ERROR: invalid period %d\n", cfg.req_period);
        return 1;
    }
    if (cfg.buffer_mult < 2) {
        fprintf(stderr, "ERROR: invalid buffer-mult %d\n", cfg.buffer_mult);
        return 1;
    }
    if (cfg.start_fill_periods < 1 || cfg.start_fill_periods > cfg.buffer_mult) {
        fprintf(stderr, "ERROR: invalid start-fill %d (buffer-mult=%d)\n",
                cfg.start_fill_periods, cfg.buffer_mult);
        return 1;
    }
    if (cfg.xrun_fill_periods < 1 || cfg.xrun_fill_periods > cfg.buffer_mult) {
        fprintf(stderr, "ERROR: invalid xrun-fill %d (buffer-mult=%d)\n",
                cfg.xrun_fill_periods, cfg.buffer_mult);
        return 1;
    }
    if (cfg.adapt_delay_ms < 0 || cfg.adapt_delay_ms > 10000) {
        fprintf(stderr, "ERROR: invalid adapt-delay-ms %d\n",
                cfg.adapt_delay_ms);
        return 1;
    }
    if (cfg.measure_secs < 1) {
        fprintf(stderr, "ERROR: invalid measure-secs %d\n", cfg.measure_secs);
        return 1;
    }
    if (cfg.capture_secs < 0) {
        fprintf(stderr, "ERROR: invalid capture-secs %d\n", cfg.capture_secs);
        return 1;
    }
    if (cfg.measure_noise_amp <= 0.0f || cfg.measure_noise_amp > 1.0f) {
        fprintf(stderr, "ERROR: invalid measure-noise %.3f\n",
                cfg.measure_noise_amp);
        return 1;
    }

    if (strcmp(mode, "sim") == 0) {
        if (!sim_sweep) {
            fxlms_t tmp;
            fxlms_init(&tmp, W_LEN_DEFAULT, S_LEN_DEFAULT, cfg.mu, cfg.leak);
            fxlms_set_band(&tmp, cfg.band_lo_hz, cfg.band_hi_hz);
            if (fxlms_load_sec_default(&tmp) < 0) {
                fprintf(stderr, "ERROR: missing %s. run './anc measure' first.\n", SEC_PATH_FILE);
                fxlms_free(&tmp);
                return 1;
            }

            const float default_freqs[] = { 80.0f, 100.0f, 120.0f, 150.0f };
            float single_freq[1];
            const float *freqs = default_freqs;
            int n_freq = 4;
            if (cfg.tone_hz > 0.0f) {
                single_freq[0] = cfg.tone_hz;
                freqs = single_freq;
                n_freq = 1;
            }
            double worst_db = 1e9;
            for (int i = 0; i < n_freq; i++) {
                double db = simulate_one_freq(tmp.s, tmp.s_len, freqs[i],
                                              cfg.mu, cfg.leak, cfg.mu_n_max,
                                              cfg.eps_abs, cfg.eps_rel,
                                              cfg.band_lo_hz, cfg.band_hi_hz,
                                              cfg.mb_bands);
                if (db < worst_db) worst_db = db;
                fprintf(stderr, "[SIM %.0fHz] reduction=%+.2fdB\n", freqs[i], db);
            }
            fprintf(stderr,
                    "[SIM config] mu=%.5f leak=%.6f mu_n_max=%.3f eps_abs=%.4g eps_rel=%.4g tone_hz=%.1f band=%.0f~%.0fHz mb_bands=%d worst=%+.2fdB\n",
                    cfg.mu, cfg.leak, cfg.mu_n_max, cfg.eps_abs, cfg.eps_rel,
                    cfg.tone_hz, cfg.band_lo_hz, cfg.band_hi_hz,
                    cfg.mb_bands, worst_db);
            fxlms_free(&tmp);
            return 0;
        }
        return run_sim_sweep(cfg.tone_hz, cfg.band_lo_hz, cfg.band_hi_hz,
                              cfg.mb_bands) == 0 ? 0 : 1;
    }

    /* Signal handlers */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    /* measure/sim modes allocate large signal buffers — skip MCL_FUTURE to
     * avoid RLIMIT_MEMLOCK failures.  RT run modes lock future pages so that
     * the real-time loop buffers are never swapped out. */
    int need_lock_future = !(strcmp(mode, "measure") == 0 ||
                              strcmp(mode, "sim")     == 0 ||
                              strcmp(mode, "passthrough") == 0 ||
                              strcmp(mode, "capture-bench") == 0);
    rt_prepare_memory(need_lock_future);

    /* ALSA init (strict validation) */
    alsa_ctx_t alsa;
    if (strcmp(mode, "capture-bench") == 0) {
        if (alsa_init_capture_only(&alsa, DEVICE, SAMPLE_RATE,
                                   (snd_pcm_uframes_t)cfg.req_period,
                                   (snd_pcm_uframes_t)(cfg.req_period * cfg.buffer_mult)) < 0)
            return 1;
    } else {
        if (alsa_init(&alsa, DEVICE, SAMPLE_RATE,
                      (snd_pcm_uframes_t)cfg.req_period,
                      (snd_pcm_uframes_t)(cfg.req_period * cfg.buffer_mult)) < 0)
            return 1;
    }
    alsa_print(&alsa);

    /* Logger */
    logger_t log;
    logger_init(&log, alsa.rate, cfg.tone_hz);

    if (strcmp(mode, "passthrough") == 0) {
        run_passthrough(&alsa, &log,
                        cfg.start_fill_periods, cfg.xrun_fill_periods);
    }
    else if (strcmp(mode, "capture-bench") == 0) {
        run_capture_bench(&alsa, cfg.capture_secs);
    }
    else if (strcmp(mode, "measure") == 0) {
        run_measurement(&alsa, S_LEN_DEFAULT, cfg.measure_noise_amp,
                        cfg.measure_secs, SEC_PATH_FILE,
                        cfg.start_fill_periods, cfg.xrun_fill_periods);
    }
    else if (strcmp(mode, "run") == 0) {
        mbfxlms_t mb;
        mbfxlms_init(&mb, cfg.mb_bands, W_LEN_DEFAULT, S_LEN_DEFAULT,
                     cfg.mu, cfg.leak, cfg.band_lo_hz, cfg.band_hi_hz);
        mbfxlms_set_mu_n_max(&mb, cfg.mu_n_max);
        mbfxlms_set_eps(&mb, cfg.eps_abs, cfg.eps_rel);

        if (mbfxlms_load_sec_default(&mb) < 0) {
            fprintf(stderr, "ERROR: run './anc measure' first.\n");
            mbfxlms_free(&mb);
            alsa_close(&alsa);
            return 1;
        }

        run_anc(&alsa, &mb, &log, no_adapt,
                cfg.band_lo_hz, cfg.band_hi_hz,
                cfg.start_fill_periods, cfg.xrun_fill_periods,
                cfg.adapt_delay_ms);
        mbfxlms_free(&mb);
    }
    else if (strcmp(mode, "nb") == 0) {
        /* Load secondary path for narrowband eval */
        fxlms_t tmp;
        fxlms_init(&tmp, 2, S_LEN_DEFAULT, 0.01f, 1.0f);
        if (fxlms_load_sec_default(&tmp) < 0) {
            fprintf(stderr, "ERROR: run './anc measure' first.\n");
            fxlms_free(&tmp);
            alsa_close(&alsa);
            return 1;
        }
        run_nb_anc(&alsa, tmp.s, tmp.s_len, &log,
                    cfg.nb_harms, cfg.nb_mu, cfg.nb_leak,
                    cfg.start_fill_periods, cfg.xrun_fill_periods);
        fxlms_free(&tmp);
    }
    else {
        usage(argv[0]);
        alsa_close(&alsa);
        return 1;
    }

    fprintf(stderr, "Shutting down...\n");
    alsa_close(&alsa);
    return 0;
}
