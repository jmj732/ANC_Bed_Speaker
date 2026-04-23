/*
 * anc_dsp.h — 저수준 DSP 원소.
 *
 * 포함 내용:
 *   - 신호 변환 helpers (to_f, clip16, clampf)
 *   - NEON 가속 커널 (dot_f32, sumsq2_f32, update_weights_f32)
 *   - 유틸리티 (db_improvement, white_noise, get_time)
 *   - 순환 버퍼 (cbuf_t)
 *   - 대역 필터 (band_ctl_t, output_safety_t)
 *   - 단음 미터 (tone_meter_t)
 *   - 교차 상관 (xcorr_peak_lag)
 *   - 전역 정지 플래그 (g_running, sig_handler)
 *
 * 의존성: anc_defs.h
 */
#ifndef ANC_DSP_H
#define ANC_DSP_H

/* ===== 전역 정지 플래그 (모든 런타임 루프가 참조) ===== */
static volatile sig_atomic_t g_running = 1;

static void sig_handler(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ===== 샘플 변환 helpers ===== */
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

/* ===== NEON 벡터 합산 ===== */
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

/* ===== 내적 (NEON 가속) ===== */
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

/* ===== 두 벡터의 제곱합 동시 계산 (NEON 가속) ===== */
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

/* ===== Leaky FxNLMS 가중치 갱신 (NEON 가속) ===== */
static void update_weights_f32(float *w, const float *fx, int n,
                               float leakage, float mu_n,
                               float e_use, float grad_clip)
{
#if ANC_HAVE_NEON
    float32x4_t v_leak   = vdupq_n_f32(leakage);
    float32x4_t v_mu     = vdupq_n_f32(mu_n);
    float32x4_t v_e      = vdupq_n_f32(e_use);
    float32x4_t v_clip_hi = vdupq_n_f32(grad_clip);
    float32x4_t v_clip_lo = vdupq_n_f32(-grad_clip);
    int k = 0;
    for (; k + 4 <= n; k += 4) {
        float32x4_t v_w   = vld1q_f32(w + k);
        float32x4_t v_fx  = vld1q_f32(fx + k);
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

/* ===== 유틸리티 ===== */
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

/* ===== 순환 버퍼 (double-buffer, 연속 읽기 보장) ===== */
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

/* ===== 1차 대역 필터 (HPF + LPF 직렬) ===== */
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

/* 출력 보호용 2단 LPF (고주파 스파이크 차단) */
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

/* ===== 단음 미터 (고정 주파수 RMS 측정) ===== */
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

/* ===== 교차 상관 (ref mic 리드 확인용) ===== */
/* 양수 반환: y가 x보다 lag 샘플 늦음 (x = ref이면 ref LEADS) */
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

#endif /* ANC_DSP_H */
