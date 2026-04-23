/*
 * anc_algo.h — 적응 필터 알고리즘.
 *
 * 포함 내용:
 *   - FxLMS 단일 밴드 필터 (fxlms_t)
 *   - 이차 경로 파일 I/O (fxlms_load_sec, save_sec_path)
 *   - 멀티밴드 FxLMS (mbfxlms_t)
 *   - 협대역 피치 추적 ANC (nb_anc_t, nb_harm_t)
 *
 * 정렬 규칙: fxlms_load_sec_default가 mbfxlms_load_sec_default보다 먼저 정의되므로
 * forward 선언 없이 컴파일 가능.
 *
 * 의존성: anc_defs.h, anc_dsp.h
 */
#ifndef ANC_ALGO_H
#define ANC_ALGO_H

#include "anc_dsp.h"

/* ===== FxLMS 단일 밴드 적응 필터 ===== */
typedef struct {
    int     w_len;
    int     s_len;
    float  *w;           /* 제어 필터 계수 */
    float  *s;           /* 이차 경로 추정 계수 */
    cbuf_t  x_buf;
    cbuf_t  fx_buf;
    float   mu;          /* VSS raw step size */
    float   mu_floor;
    float   mu_ceil;     /* VSS raw mu 상한 */
    float   mu_n_floor;
    float   mu_n_max;    /* FxNLMS 정규화 step 상한 */
    float   leakage;
    float   epsilon_abs; /* NLMS 분모 절대 하한 */
    float   epsilon_rel; /* NLMS 분모 상대 정규화 계수 */
    float   ema_alpha;
    float   update_pow_floor;
    float   err_clip;
    float   grad_clip;
    float   fx_pow_ema;
    float   x_pow_ema;
    float   last_mu_n;   /* 로깅용: 직전 샘플의 정규화 step */
    band_ctl_t ref_band;
    band_ctl_t err_band;
    int     adapt;
    /* Akhtar VSS: mu(n+1) = vss_alpha*mu(n) + vss_gamma*e² */
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
    f->w_len           = w_len;
    f->s_len           = s_len;
    f->mu              = mu;
    f->mu_floor        = MU_FLOOR;
    f->mu_ceil         = MU_CEIL;
    f->mu_n_floor      = MU_N_FLOOR;
    f->mu_n_max        = MU_N_MAX;
    f->leakage         = leak;
    f->epsilon_abs     = EPSILON_ABS;
    f->epsilon_rel     = EPSILON_REL;
    f->ema_alpha       = POWER_EMA_ALPHA;
    f->update_pow_floor = UPDATE_POW_FLOOR;
    f->err_clip        = ERR_CLIP;
    f->grad_clip       = GRAD_CLIP;
    f->fx_pow_ema      = 0.0f;
    f->x_pow_ema       = 0.0f;
    f->last_mu_n       = 0.0f;
    f->adapt           = 1;
    f->vss_alpha       = VSS_ALPHA;
    f->vss_gamma       = VSS_GAMMA;
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

    /* Filtered-x: x'(n) = S_hat^T · X */
    float x_filt = dot_f32(f->s, x, f->s_len);
    cbuf_push(&f->fx_buf, x_filt);
    float *fx = cbuf_ptr(&f->fx_buf);

    /* Anti-noise output: y = W^T · X */
    float y = dot_f32(f->w, x, f->w_len);

    float fx_pow = 0.0f, x_pow = 0.0f;
    sumsq2_f32(fx, x, f->w_len, &fx_pow, &x_pow);

    if (f->fx_pow_ema == 0.0f) f->fx_pow_ema = fx_pow;
    else f->fx_pow_ema = f->ema_alpha * f->fx_pow_ema + (1.0f - f->ema_alpha) * fx_pow;
    if (f->x_pow_ema == 0.0f)  f->x_pow_ema  = x_pow;
    else f->x_pow_ema  = f->ema_alpha * f->x_pow_ema  + (1.0f - f->ema_alpha) * x_pow;

    /* Akhtar VSS: mu(n+1) = alpha*mu(n) + gamma*e², clamped to [floor, ceil] */
    if (f->vss_gamma > 0.0f)
        f->mu = clampf(f->vss_alpha * f->mu + f->vss_gamma * e * e,
                       f->mu_floor, f->mu_ceil);

    /* Leaky FxNLMS weight update.
     * 분모: ||fx||²*(1+ε_rel) + ε_abs — 표준 NLMS 정규화.
     * 결과를 [mu_n_floor, mu_n_max]로 clamp해 안정성 보장. */
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

static void fxlms_reset_state(fxlms_t *f)
{
    cbuf_reset(&f->x_buf);
    cbuf_reset(&f->fx_buf);
    band_ctl_reset(&f->ref_band);
    band_ctl_reset(&f->err_band);
    f->fx_pow_ema = 0.0f;
    f->x_pow_ema  = 0.0f;
    f->last_mu_n  = 0.0f;
}

static void fxlms_reset_w(fxlms_t *f)
{
    memset(f->w, 0, f->w_len * sizeof(float));
    fxlms_reset_state(f);
}

/* ===== 이차 경로 파일 I/O ===== */
/* mbfxlms_load_sec_default가 이 함수를 호출하므로 mbfxlms 구조체보다 먼저 정의 */

static void print_sec_path_stats(const float *s, int s_len, unsigned int rate,
                                 const char *tag)
{
    int peak_idx = 0;
    float peak_val = 0.0f;
    double energy = 0.0;
    for (int k = 0; k < s_len; k++) {
        float abs_v = fabsf(s[k]);
        if (abs_v > peak_val) { peak_val = abs_v; peak_idx = k; }
        energy += (double)s[k] * s[k];
    }
    fprintf(stderr, "%s: peak=%.4f @ %d (%.2fms)  rms=%.6f  energy=%.6f\n",
            tag, peak_val, peak_idx,
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
        fclose(fp); return -1;
    }
    if (hdr[0] != f->s_len) {
        fprintf(stderr, "%s: s_len mismatch (file=%d, expected=%d)\n",
                path, hdr[0], f->s_len);
        fclose(fp); return -1;
    }
    if (fread(f->s, sizeof(float), f->s_len, fp) != (size_t)f->s_len) {
        fprintf(stderr, "%s: data read failed\n", path);
        fclose(fp); return -1;
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

/* ===== 멀티밴드 FxLMS ===== */
typedef struct {
    int     n_bands;
    float   band_edges[MBAND_BANDS_MAX + 1];
    fxlms_t bands[MBAND_BANDS_MAX];
} mbfxlms_t;

static void mbfxlms_build_edges(float *edges, int n_bands, float lo_hz, float hi_hz)
{
    if (n_bands <= 1) { edges[0] = lo_hz; edges[1] = hi_hz; return; }
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
    if (n_bands < 1) n_bands = 1;
    if (n_bands > MBAND_BANDS_MAX) n_bands = MBAND_BANDS_MAX;
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

/* fxlms_load_sec_default가 위에서 정의됐으므로 forward 선언 불필요 */
static int mbfxlms_load_sec_default(mbfxlms_t *m)
{
    if (m->n_bands <= 0) return -1;
    if (fxlms_load_sec_default(&m->bands[0]) < 0) return -1;
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
    /* FxNLMS 정규화 step 상한 mu_n_max 설정. 범위: [mu_n_floor, 2.0] */
    for (int i = 0; i < m->n_bands; i++)
        m->bands[i].mu_n_max = clampf(mu_n_max, m->bands[i].mu_n_floor, 2.0f);
}

static void mbfxlms_scale_mu_n_max(mbfxlms_t *m, float scale)
{
    /* 발산 시 낮추고(scale<1) 회복 시 높인다(scale>1) */
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
    if (m->n_bands <= 0) return 0.0f;
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
    for (int i = 0; i < m->n_bands; i++)
        fprintf(stderr, "  band%d: %.0f~%.0fHz\n",
                i, m->band_edges[i], m->band_edges[i + 1]);
}

/* ===== 협대역 ANC — 피치 추적 고조파 제거기 ===== */
typedef struct {
    float w_c, w_s;   /* cos / sin 가중치 */
    float phase;      /* 발진기 위상 */
    float phase_step; /* 2π·f / fs */
    float sx_mag;     /* |S(f)| */
    float sx_phase;   /* ∠S(f) */
    float cos_sx;     /* cosf(sx_phase) — 덧셈 정리용 사전 계산 */
    float sin_sx;     /* sinf(sx_phase) — 덧셈 정리용 사전 계산 */
    float freq_hz;
} nb_harm_t;

typedef struct {
    /* f0 추적기 */
    float  f0_buf[NB_F0_BUF_LEN];
    int    f0_pos;
    int    f0_count;        /* 누적 샘플 수 */
    int    f0_next_update;
    float  f0_hz;
    float  f0_conf;
    int    lag_min;         /* fs / f_max */
    int    lag_max;         /* fs / f_min */
    /* f0 홀드: 2회 연속 일치 검출 후 적용 */
    float  f0_candidate;
    int    f0_cand_count;

    /* 고조파 */
    int       n_harm;
    nb_harm_t harm[NB_MAX_HARM];

    /* 이차 경로 (소유권 없는 참조 포인터) */
    const float *sec;
    int    sec_len;
    float  fs;

    /* 제어 파라미터 */
    float  mu;
    float  leak;
    int    adapt;

    /* 페이드아웃 상태: f0 소실 시 하드 뮤트 대신 부드럽게 감쇠 */
    float  last_y;   /* 직전 출력 (페이드아웃 씨드) */
    float  out_env;  /* 페이드아웃 엔벨로프 [1→0] */
} nb_anc_t;

/* 이차 경로 주파수 응답 단일 주파수 평가 */
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
        nb->harm[h].freq_hz    = fh;
        nb->harm[h].phase_step = 2.0f * (float)M_PI * fh / nb->fs;
        nb_eval_sec(nb->sec, nb->sec_len, fh, nb->fs,
                    &nb->harm[h].sx_mag, &nb->harm[h].sx_phase);
        nb->harm[h].cos_sx = cosf(nb->harm[h].sx_phase);
        nb->harm[h].sin_sx = sinf(nb->harm[h].sx_phase);
        if (old_freq <= 0.0f || fabsf(fh - old_freq) > old_freq * 0.15f) {
            nb->harm[h].w_c = 0;
            nb->harm[h].w_s = 0;
        }
    }
}

static float nb_detect_f0(const float *buf, int len, int lag_min, int lag_max,
                          float fs, float *out_conf)
{
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
        if (r > best_r) { best_r = r; best_lag = lag; }
    }

    *out_conf = best_r;
    if (best_r < NB_F0_CONF_THR) return 0.0f;

    /* 포물선 보간 */
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

/* 샘플당 f0 버퍼 적재 (저비용 — 매 샘플 호출) */
static void nb_f0_buf_sample(nb_anc_t *nb, float sample)
{
    nb->f0_buf[nb->f0_pos] = sample;
    nb->f0_pos = (nb->f0_pos + 1) % NB_F0_BUF_LEN;
    nb->f0_count++;
}

/* f0 검출 실행 (고비용 — period 사이에 호출). f0 변경 시 1 반환 */
static int nb_f0_update(nb_anc_t *nb)
{
    if (nb->f0_count < nb->f0_next_update) return 0;
    nb->f0_next_update = nb->f0_count + NB_F0_UPDATE_SAMPLES;

    int filled = (nb->f0_count < NB_F0_BUF_LEN) ? nb->f0_count : NB_F0_BUF_LEN;
    int start  = (nb->f0_pos - filled + NB_F0_BUF_LEN) % NB_F0_BUF_LEN;

    /* 3탭 평균 LPF 후 4× 데시메이션 (48kHz→12kHz, f0 검출에 충분) */
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
        /* 홀드: 15% 이내 2회 연속 검출 후 적용 */
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
        nb->f0_candidate  = 0.0f;
        nb->f0_cand_count = 0;
        nb->f0_hz = 0;
        return 1;
    }
    return 0;
}

/* 샘플당 협대역 FxLMS: 반잡음 출력 반환 */
static float nb_step(nb_anc_t *nb, float err)
{
    /* f0 소실 시 하드 뮤트 대신 ~200ms 지수 페이드아웃 */
    if (nb->f0_hz <= 0.0f) {
        if (nb->out_env <= 0.0f) return 0.0f;
        nb->out_env *= 0.999896f;  /* tau ≈ 9600 샘플 @ 48kHz */
        if (nb->out_env < 1e-4f) nb->out_env = 0.0f;
        return nb->last_y * nb->out_env;
    }

    nb->out_env = 1.0f;
    float y = 0.0f;
    for (int h = 0; h < nb->n_harm; h++) {
        nb_harm_t *p = &nb->harm[h];
        if (p->freq_hz <= 0.0f) continue;

        /* sincosf 1회 호출로 cos_p, sin_p 동시 계산 */
        float cos_p, sin_p;
        sincosf(p->phase, &sin_p, &cos_p);

        y += p->w_c * cos_p + p->w_s * sin_p;

        /* Filtered-x 가중치 갱신 (NLMS):
         * 덧셈 정리로 trig 추가 호출 없이 fx_cos, fx_sin 계산
         *   cos(p+sx) = cos_p*cos_sx - sin_p*sin_sx
         *   sin(p+sx) = sin_p*cos_sx + cos_p*sin_sx */
        if (nb->adapt && p->sx_mag > 1e-6f) {
            float fx_cos = p->sx_mag * (cos_p * p->cos_sx - sin_p * p->sin_sx);
            float fx_sin = p->sx_mag * (sin_p * p->cos_sx + cos_p * p->sin_sx);
            float power  = p->sx_mag * p->sx_mag + NB_NLMS_EPS;
            float mu_n   = nb->mu / power;
            p->w_c -= mu_n * err * fx_cos;
            p->w_s -= mu_n * err * fx_sin;
            p->w_c *= nb->leak;
            p->w_s *= nb->leak;
            float wmag = sqrtf(p->w_c * p->w_c + p->w_s * p->w_s);
            if (wmag > 0.3f) {
                float inv = 0.3f / wmag;
                p->w_c *= inv;
                p->w_s *= inv;
            }
        }

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

#endif /* ANC_ALGO_H */
