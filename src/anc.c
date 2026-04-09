/*
 * anc.c — FxLMS Active Noise Cancellation (single-file prototype)
 *
 * Build:  gcc -O2 -o anc anc.c -lasound -lm
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
#include <sched.h>
#include <sys/mman.h>
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
#ifndef MU_FLOOR
#define MU_FLOOR        0.001f
#endif
#ifndef MU_CEIL
#define MU_CEIL         0.030f
#endif
#ifndef MU_N_MIN
#define MU_N_MIN        0.0f
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
#ifndef ERR_UPDATE_MAX
#define ERR_UPDATE_MAX  0.02f
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
#ifndef TUNE_UP_RATE
#define TUNE_UP_RATE    1.08f
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

static void rt_prepare_memory(void)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
        fprintf(stderr, "WARNING: mlockall failed: %s\n", strerror(errno));

    struct sched_param sp;
    sp.sched_priority = 49;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0)
        fprintf(stderr, "WARNING: SCHED_FIFO failed: %s (run with sudo or set rtprio)\n",
                strerror(errno));
    else
        fprintf(stderr, "RT: SCHED_FIFO priority %d\n", sp.sched_priority);

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
    b->pos = (b->pos - 1 + b->cap) % b->cap;
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
    float   mu_n_min;
    float   mu_n_floor;
    float   mu_n_max;
    float   mu_n_ceil;
    float   leakage;
    float   epsilon_abs;
    float   epsilon_rel;
    float   ema_alpha;
    float   update_pow_floor;
    float   err_clip;
    float   err_update_max;
    float   grad_clip;
    float   fx_pow_ema;
    float   x_pow_ema;
    float   last_mu_n;
    float   last_fx_pow;
    float   last_x_pow;
    float   band_lo_hz;
    float   band_hi_hz;
    band_ctl_t ref_band;
    band_ctl_t err_band;
    int     adapt;
} fxlms_t;

static void fxlms_set_band(fxlms_t *f, float lo_hz, float hi_hz)
{
    f->band_lo_hz = lo_hz;
    f->band_hi_hz = hi_hz;
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
    f->mu_n_min = MU_N_MIN;
    f->mu_n_floor = MU_N_FLOOR;
    f->mu_n_max = MU_N_MAX;
    f->mu_n_ceil = MU_N_MAX;
    f->leakage = leak;
    f->epsilon_abs = EPSILON_ABS;
    f->epsilon_rel = EPSILON_REL;
    f->ema_alpha = POWER_EMA_ALPHA;
    f->update_pow_floor = UPDATE_POW_FLOOR;
    f->err_clip = ERR_CLIP;
    f->err_update_max = ERR_UPDATE_MAX;
    f->grad_clip = GRAD_CLIP;
    f->fx_pow_ema = 0.0f;
    f->x_pow_ema = 0.0f;
    f->last_mu_n = 0.0f;
    f->last_fx_pow = 0.0f;
    f->last_x_pow = 0.0f;
    f->band_lo_hz = TARGET_BAND_LO_HZ;
    f->band_hi_hz = TARGET_BAND_HI_HZ;
    f->adapt   = 1;
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
    float x_filt = 0.0f;
    for (int k = 0; k < f->s_len; k++)
        x_filt += f->s[k] * x[k];

    cbuf_push(&f->fx_buf, x_filt);
    float *fx = cbuf_ptr(&f->fx_buf);

    /* Anti-noise output: y = W^T . X */
    float y = 0.0f;
    for (int k = 0; k < f->w_len; k++)
        y += f->w[k] * x[k];

    float fx_pow = 0.0f;
    float x_pow  = 0.0f;
    for (int k = 0; k < f->w_len; k++) {
        fx_pow += fx[k] * fx[k];
        x_pow  += x[k]  * x[k];
    }

    f->last_fx_pow = fx_pow;
    f->last_x_pow  = x_pow;
    if (f->fx_pow_ema == 0.0f) f->fx_pow_ema = fx_pow;
    else f->fx_pow_ema = f->ema_alpha * f->fx_pow_ema + (1.0f - f->ema_alpha) * fx_pow;
    if (f->x_pow_ema == 0.0f) f->x_pow_ema = x_pow;
    else f->x_pow_ema = f->ema_alpha * f->x_pow_ema + (1.0f - f->ema_alpha) * x_pow;

    /* Leaky FxNLMS weight update with guarded denominator. */
    if (f->adapt) {
        float den = f->fx_pow_ema + f->epsilon_abs + f->epsilon_rel * f->x_pow_ema;
        float mu_n = f->mu / den;
        mu_n = clampf(mu_n, f->mu_n_min, f->mu_n_max);
        f->last_mu_n = mu_n;

        if (f->fx_pow_ema > f->update_pow_floor &&
            fabsf(e) <= f->err_update_max) {
            float e_use = clampf(e, -f->err_clip, f->err_clip);
            for (int k = 0; k < f->w_len; k++) {
                float grad = clampf(e_use * fx[k], -f->grad_clip, f->grad_clip);
                f->w[k] = f->leakage * f->w[k] - mu_n * grad;
            }
        }
    }

    return y;
}

static float fxlms_step_sample(fxlms_t *f, float ref, float e, float *out_e_band)
{
    float ref_band = band_ctl_step(&f->ref_band, ref);
    float e_band = band_ctl_step(&f->err_band, e);
    if (out_e_band)
        *out_e_band = e_band;
    return fxlms_step(f, ref_band, e_band);
}

static float fxlms_w_norm(fxlms_t *f)
{
    float sum = 0.0f;
    for (int k = 0; k < f->w_len; k++)
        sum += f->w[k] * f->w[k];
    return sqrtf(sum);
}

static int fxlms_load_sec_default(fxlms_t *f);

static void fxlms_reset_w(fxlms_t *f)
{
    memset(f->w, 0, f->w_len * sizeof(float));
    cbuf_reset(&f->x_buf);
    cbuf_reset(&f->fx_buf);
    band_ctl_reset(&f->ref_band);
    band_ctl_reset(&f->err_band);
    f->fx_pow_ema = 0.0f;
    f->x_pow_ema = 0.0f;
    f->last_mu_n = 0.0f;
    f->last_fx_pow = 0.0f;
    f->last_x_pow = 0.0f;
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

static void mbfxlms_set_mu(mbfxlms_t *m, float mu)
{
    for (int i = 0; i < m->n_bands; i++)
        m->bands[i].mu = mu;
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
    for (int i = 0; i < m->n_bands; i++) {
        fxlms_t *f = &m->bands[i];
        f->mu_n_max = mu_n_max;
        f->mu_n_ceil = mu_n_max;
    }
}

static void mbfxlms_scale_mu_n_max(mbfxlms_t *m, float scale)
{
    for (int i = 0; i < m->n_bands; i++) {
        fxlms_t *f = &m->bands[i];
        f->mu_n_max = clampf(f->mu_n_max * scale, f->mu_n_floor, f->mu_n_ceil);
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

static float mbfxlms_step_sample(mbfxlms_t *m, float ref, float e)
{
    float anti = 0.0f;
    for (int i = 0; i < m->n_bands; i++) {
        fxlms_t *f = &m->bands[i];
        float ref_band = band_ctl_step(&f->ref_band, ref);
        anti += fxlms_step(f, ref_band, e);
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

    if (a->linked)
        return snd_pcm_prepare(a->cap);

    if ((err = snd_pcm_prepare(a->play)) < 0)
        return err;
    return snd_pcm_prepare(a->cap);
}

static int alsa_prefill_playback(alsa_ctx_t *a, const int16_t *buf, int periods)
{
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

    memset(silence_buf, 0, a->period * CHANNELS * sizeof(int16_t));
    snd_pcm_drop(a->play);
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
    else {
        snd_pcm_start(a->play);
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

    /* ========== Phase 1: Ambient capture — ref mic lead check ========== */
    long amb_total = (long)rate * AMBIENT_SECS;
    amb_ref = (float *)malloc(amb_total * sizeof(float));
    amb_err = (float *)malloc(amb_total * sizeof(float));
    long amb_count = 0;
    double amb_ref_sq = 0, amb_err_sq = 0;

    /* Pre-fill playback with silence, then start */
    if (alsa_resync(a, out_buf, start_fill_periods) < 0)
        goto cleanup;

    fprintf(stderr, "\n=== Phase 1: Ambient capture (%d sec) ===\n", AMBIENT_SECS);

    while (g_running && amb_count < amb_total) {
        int err = pcm_write_full(a->play, out_buf, period);  /* silence */
        if (err < 0) {
            xruns++;
            if (alsa_handle_io_error(a, a->play, "playback", err, out_buf,
                                     xrun_fill_periods) < 0)
                goto cleanup;
            continue;
        }

        err = pcm_read_full(a->cap, in_buf, period);
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
    }

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
            int lag = xcorr_peak_lag(amb_ref, amb_err, (int)amb_count,
                                     XCORR_MAX_LAG, &corr_val);
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
    free(amb_ref);
    free(amb_err);
    amb_ref = NULL;
    amb_err = NULL;

    /* Re-sync ALSA between phases (playback may have underrun during setup) */
    if (alsa_resync(a, out_buf, start_fill_periods) < 0)
        goto cleanup;

    /* ========== Phase 2: White noise — secondary path estimation ========== */
    long total_samples = (long)rate * duration_sec;

    /* Signal buffers for post-hoc cross-correlation */
    sig_out = (float *)malloc(total_samples * sizeof(float));
    sig_err = (float *)malloc(total_samples * sizeof(float));
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
        /* Generate white noise output */
        for (int i = 0; i < (int)period; i++) {
            float u = white_noise(&rng_seed) * noise_amp;
            out_buf[i * 2 + 0] = clip16(u);
            out_buf[i * 2 + 1] = clip16(u);
        }

        int err = pcm_write_full(a->play, out_buf, period);
        if (err < 0) {
            xruns++;
            if (alsa_handle_io_error(a, a->play, "playback", err, out_buf,
                                     xrun_fill_periods) < 0)
                break;
            continue;
        }

        err = pcm_read_full(a->cap, in_buf, period);
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

            if (sig_count < total_samples) {
                sig_out[sig_count] = u_sample;
                sig_err[sig_count] = d_measured;
                sig_count++;
            }
        }

        /* 1-second log */
        double elapsed = get_time() - t_start;
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

            pred_err_sum_sq = 0;
            pred_count = 0;
            next_log += 1.0;
        }
    }

    /* ========== Post-measurement analysis ========== */
    fprintf(stderr, "\n=== Latency Analysis ===\n");

    /* Method 1: LMS peak coefficient */
    int lms_peak = 0;
    float lms_peak_val = 0.0f;
    for (int k = 0; k < s_len; k++) {
        if (fabsf(s_hat[k]) > lms_peak_val) {
            lms_peak_val = fabsf(s_hat[k]);
            lms_peak = k;
        }
    }
    float lms_ms = (float)lms_peak / (float)rate * 1000.0f;
    fprintf(stderr, "LMS peak_coeff:   %d samples (%.2f ms)  value=%.4f\n",
            lms_peak, lms_ms, lms_peak_val);

    /* Method 2: Cross-correlation on last 2 sec of data */
    if (sig_count > 0) {
        int xc_n = (int)sig_count;
        int xc_off = 0;
        if (xc_n > (int)rate * 2) {
            xc_off = xc_n - (int)rate * 2;
            xc_n   = (int)rate * 2;
        }

        float xc_val;
        int xc_lag = xcorr_peak_lag(sig_out + xc_off, sig_err + xc_off,
                                    xc_n, XCORR_MAX_LAG, &xc_val);
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

    if (save_sec_path(s_hat, s_len, rate, out_path) == 0) {
        long fsize = 8 + s_len * (long)sizeof(float);
        fprintf(stderr, "Saved:        %s (%ld bytes)\n", out_path, fsize);
    }

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
    band_ctl_init(&err_mon, band_lo_hz, band_hi_hz, (float)a->rate);

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
    tone_meter_init(&baseline_track, l->tone_track.freq_hz, (float)a->rate);

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
            float ref  = to_f(in_buf[i * 2 + REF_CH]);
            float e    = to_f(in_buf[i * 2 + ERR_CH]);
            float e_tone = band_ctl_step(&err_mon, e);
            float anti = mbfxlms_step_sample(m, ref, e);

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

static int run_simulation(const char *sec_path, int sweep, float tone_hz,
                          float band_lo_hz, float band_hi_hz, int mb_bands)
{
    fxlms_t tmp;
    fxlms_init(&tmp, W_LEN_DEFAULT, S_LEN_DEFAULT, MU_DEFAULT, LEAKAGE_DEFAULT);
    (void)sec_path;
    if (fxlms_load_sec_default(&tmp) < 0) {
        fprintf(stderr, "ERROR: missing %s. run './anc measure' first.\n", sec_path);
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

    if (!sweep) {
        double worst_db = 1e9;
        for (int i = 0; i < n_freq; i++) {
            double db = simulate_one_freq(tmp.s, tmp.s_len, freqs[i],
                                          MU_DEFAULT, LEAKAGE_DEFAULT,
                                          MU_N_MAX, EPSILON_ABS, EPSILON_REL,
                                          band_lo_hz, band_hi_hz, mb_bands);
            if (db < worst_db) worst_db = db;
            fprintf(stderr, "[SIM %.0fHz] reduction=%+.2fdB\n", freqs[i], db);
        }
        fprintf(stderr, "[SIM default] worst-case=%+.2fdB  target=%+.1fdB\n",
                worst_db, TARGET_REDUCTION_DB);
    } else {
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
    float measure_noise_amp;
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
    cfg->measure_noise_amp = NOISE_AMP;
}

/* ===== main ===== */
static void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s <mode> [options]\n"
            "  passthrough           capture -> playback echo-back\n"
            "  measure               estimate secondary path -> %s\n"
            "  run [--no-adapt]      FxLMS active noise cancellation\n"
            "  sim [--sweep]         offline tuning check\n"
            "Options: --mu=F --leak=F --mu-n-max=F --eps-abs=F --eps-rel=F\n"
            "         --tone-hz=F --band-lo=F --band-hi=F --mb-bands=N\n"
            "         --period=N --buffer-mult=N --start-fill=N --xrun-fill=N\n"
            "         --adapt-delay-ms=N\n"
            "         --measure-secs=N --measure-noise=F\n",
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
        } else if (strncmp(arg, "--measure-noise=", 16) == 0) {
            cfg.measure_noise_amp = strtof(arg + 16, NULL);
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
        return run_simulation(SEC_PATH_FILE, sim_sweep, cfg.tone_hz,
                              cfg.band_lo_hz, cfg.band_hi_hz,
                              cfg.mb_bands) == 0 ? 0 : 1;
    }

    /* Signal handlers */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    rt_prepare_memory();

    /* ALSA init (strict validation) */
    alsa_ctx_t alsa;
    if (alsa_init(&alsa, DEVICE, SAMPLE_RATE,
                  (snd_pcm_uframes_t)cfg.req_period,
                  (snd_pcm_uframes_t)(cfg.req_period * cfg.buffer_mult)) < 0)
        return 1;
    alsa_print(&alsa);

    /* Logger */
    logger_t log;
    logger_init(&log, alsa.rate, cfg.tone_hz);

    if (strcmp(mode, "passthrough") == 0) {
        run_passthrough(&alsa, &log,
                        cfg.start_fill_periods, cfg.xrun_fill_periods);
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
    else {
        usage(argv[0]);
        alsa_close(&alsa);
        return 1;
    }

    fprintf(stderr, "Shutting down...\n");
    alsa_close(&alsa);
    return 0;
}
