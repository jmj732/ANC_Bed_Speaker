/* sim_compare.c — FxLMS 적응 전략 비교 시뮬레이터
 *
 * 비교 대상:
 *   A) FxNLMS          : 현재 방식 (고정 base mu, 전력 정규화)
 *   B) Akhtar VSS      : 순수 VSS, NLMS 없음  mu = α·mu + γ·e²
 *   C) NLMS + Akhtar   : anc.c에 구현된 혼합 방식
 *
 * Build: gcc -O2 -lm src/sim_compare.c -o build/sim_compare
 * Run:   cd anc-rt && ./build/sim_compare runtime/sec_path.bin > compare.csv
 *        python3 scripts/plot_compare.py compare.csv
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* ── 시스템 상수 ────────────────────────────────────────────── */
#define FS          48000
#define W_LEN       1024
#define P_LEN       1536

/* ── 시뮬레이션 길이 ────────────────────────────────────────── */
#define WARMUP_SEC  2       /* 적응 시작 전 워밍업 (바이어스 제거) */
#define RUN_SEC     12      /* 적응 실행 시간                     */
#define WIN_SEC     0.10f   /* CSV 출력 윈도우 (초)               */

/* ── NLMS 파라미터 (현재 anc.c 값과 동일) ───────────────────── */
#define MU_0         0.010f
#define MU_FLOOR     0.001f
#define MU_CEIL      0.030f
#define MU_N_MAX     0.030f
#define EPSILON_ABS  2e-4f
#define EPSILON_REL  0.010f
#define EMA_ALPHA    0.95f
#define LEAKAGE      0.999999f
#define ERR_CLIP     1.0f
#define GRAD_CLIP    0.2f
#define OUT_LIMIT    0.9f

/* ── Akhtar VSS 파라미터 ────────────────────────────────────── */
#define VSS_ALPHA         0.9999f   /* 망각 인수 (τ ≈ 0.21s @ 48kHz)   */
#define VSS_GAMMA_PURE    1e-5f     /* B용: NLMS 없음 → mu 직접 제어   */
#define VSS_GAMMA_MIX     1e-5f     /* C용: anc.c 구현 값과 동일        */
/* 순수 VSS의 mu 상한: 안정 조건 mu < 2/fx_pow
 * 테스트 주파수(80~150Hz) 중 worst-case(150Hz, fx_pow≈138):
 *   2/138 ≈ 0.0145 → 여유 50% 반영 → 0.008                          */
#define VSS_MU_MAX_PURE   0.008f

/* ── 유틸리티 ───────────────────────────────────────────────── */
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float white_noise(uint32_t *s)
{
    *s = *s * 1664525u + 1013904223u;
    return (int32_t)(*s) * (1.0f / 2147483648.0f);
}

/* 이중 링버퍼 push / pointer-to-oldest */
static inline void rbuf_push(float *buf, int *pos, int cap, float v)
{
    buf[*pos]       = v;
    buf[*pos + cap] = v;
    *pos = (*pos + 1) % cap;
}
static inline float *rbuf_ptr(const float *buf, int pos)
{
    return (float *)(buf + pos);   /* oldest sample */
}

static float dotf(const float *a, const float *b, int n)
{
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += a[i] * b[i];
    return s;
}

static float sumsqf(const float *a, int n)
{
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += a[i] * a[i];
    return s;
}

/* ── FxLMS 상태 ─────────────────────────────────────────────── */
typedef struct {
    float w[W_LEN];           /* 적응 필터 계수                  */
    float s[W_LEN];           /* 추정 이차경로 (s_hat)            */
    float x_buf[2 * W_LEN];   /* 참조신호 링버퍼                  */
    float fx_buf[2 * W_LEN];  /* filtered-x 링버퍼               */
    int   xp, fxp;
    float fx_pow_ema;
    float x_pow_ema;
    float mu;                 /* base mu (NLMS) 또는 VSS 현재값  */
    float last_mu_n;          /* 로그용                          */
} sim_t;

static void sim_init(sim_t *f, const float *s_hat, float mu0)
{
    memset(f, 0, sizeof(*f));
    memcpy(f->s, s_hat, W_LEN * sizeof(float));
    f->mu = mu0;
}

/* ── 3가지 step 함수 ────────────────────────────────────────── */

/* A: FxNLMS — mu 고정, 전력 정규화 */
static float step_nlms(sim_t *f, float xr, float e, int adapt)
{
    rbuf_push(f->x_buf,  &f->xp,  W_LEN, xr);
    float *x  = rbuf_ptr(f->x_buf,  f->xp);

    float xf  = dotf(f->s, x, W_LEN);
    rbuf_push(f->fx_buf, &f->fxp, W_LEN, xf);
    float *fx = rbuf_ptr(f->fx_buf, f->fxp);

    float y   = dotf(f->w, x, W_LEN);

    float fp  = sumsqf(fx, W_LEN);
    float xp  = sumsqf(x,  W_LEN);
    f->fx_pow_ema = (f->fx_pow_ema == 0.f) ? fp :
                    EMA_ALPHA * f->fx_pow_ema + (1.f - EMA_ALPHA) * fp;
    f->x_pow_ema  = (f->x_pow_ema  == 0.f) ? xp :
                    EMA_ALPHA * f->x_pow_ema  + (1.f - EMA_ALPHA) * xp;

    if (adapt) {
        float den  = f->fx_pow_ema + EPSILON_ABS + EPSILON_REL * f->x_pow_ema;
        float mu_n = clampf(f->mu / den, 0.f, MU_N_MAX);
        f->last_mu_n = mu_n;
        float eu   = clampf(e, -ERR_CLIP, ERR_CLIP);
        for (int k = 0; k < W_LEN; k++) {
            float g = clampf(eu * fx[k], -GRAD_CLIP, GRAD_CLIP);
            f->w[k] = LEAKAGE * f->w[k] - mu_n * g;
        }
    }
    return y;
}

/* B: 순수 Akhtar VSS — NLMS 없음, mu = α·mu + γ·e² */
static float step_vss(sim_t *f, float xr, float e, int adapt)
{
    rbuf_push(f->x_buf,  &f->xp,  W_LEN, xr);
    float *x  = rbuf_ptr(f->x_buf,  f->xp);

    float xf  = dotf(f->s, x, W_LEN);
    rbuf_push(f->fx_buf, &f->fxp, W_LEN, xf);
    float *fx = rbuf_ptr(f->fx_buf, f->fxp);

    float y   = dotf(f->w, x, W_LEN);

    if (adapt) {
        f->mu = clampf(VSS_ALPHA * f->mu + VSS_GAMMA_PURE * e * e,
                       MU_FLOOR, VSS_MU_MAX_PURE);
        f->last_mu_n = f->mu;
        float eu = clampf(e, -ERR_CLIP, ERR_CLIP);
        for (int k = 0; k < W_LEN; k++) {
            float g = clampf(eu * fx[k], -GRAD_CLIP, GRAD_CLIP);
            f->w[k] = LEAKAGE * f->w[k] - f->mu * g;
        }
    }
    return y;
}

/* C: NLMS + Akhtar VSS — anc.c 구현 그대로 */
static float step_vss_nlms(sim_t *f, float xr, float e, int adapt)
{
    rbuf_push(f->x_buf,  &f->xp,  W_LEN, xr);
    float *x  = rbuf_ptr(f->x_buf,  f->xp);

    float xf  = dotf(f->s, x, W_LEN);
    rbuf_push(f->fx_buf, &f->fxp, W_LEN, xf);
    float *fx = rbuf_ptr(f->fx_buf, f->fxp);

    float y   = dotf(f->w, x, W_LEN);

    float fp  = sumsqf(fx, W_LEN);
    float xp  = sumsqf(x,  W_LEN);
    f->fx_pow_ema = (f->fx_pow_ema == 0.f) ? fp :
                    EMA_ALPHA * f->fx_pow_ema + (1.f - EMA_ALPHA) * fp;
    f->x_pow_ema  = (f->x_pow_ema  == 0.f) ? xp :
                    EMA_ALPHA * f->x_pow_ema  + (1.f - EMA_ALPHA) * xp;

    /* VSS: adapt 블록 바깥 — 현재 구현의 버그 포함 상태 */
    f->mu = clampf(VSS_ALPHA * f->mu + VSS_GAMMA_MIX * e * e,
                   MU_FLOOR, MU_CEIL);

    if (adapt) {
        float den  = f->fx_pow_ema + EPSILON_ABS + EPSILON_REL * f->x_pow_ema;
        float mu_n = clampf(f->mu / den, 0.f, MU_N_MAX);
        f->last_mu_n = mu_n;
        float eu   = clampf(e, -ERR_CLIP, ERR_CLIP);
        for (int k = 0; k < W_LEN; k++) {
            float g = clampf(eu * fx[k], -GRAD_CLIP, GRAD_CLIP);
            f->w[k] = LEAKAGE * f->w[k] - mu_n * g;
        }
    }
    return y;
}

/* ── 경로 빌더 ──────────────────────────────────────────────── */
/* anc.c와 동일한 계수 구조로 빌드한 뒤 뒤집음.
 *
 * step 함수들은 ring buffer를 oldest-first로 읽으므로:
 *   dotf(h, oldest_ptr, N) = sum_k h[k]*x(n-N+1+k)  ← 뒤집힌 합성곱
 *
 * 플랜트 시뮬레이션은 인과(causal) 합성곱이어야 함:
 *   d(n) = sum_k p[k]*x(n-k)
 *
 * 따라서 플랜트 계수를 뒤집으면:
 *   dotf(p_rev, oldest_ptr, N) = sum_k p[N-1-k]*x(n-N+1+k)
 *                               = sum_j p[j]*x(n-j)  ← 인과 합성곱 ✓
 */
static void reverse_f32(float *a, int n)
{
    for (int i = 0; i < n / 2; i++) {
        float t = a[i]; a[i] = a[n-1-i]; a[n-1-i] = t;
    }
}

static void build_primary(float *p, const float *s)
{
    memset(p, 0, P_LEN * sizeof(float));
    for (int k = 0; k < W_LEN; k++) {
        if (k + 120 < P_LEN) p[k + 120] += 0.95f * s[k];
        if (k + 210 < P_LEN) p[k + 210] -= 0.18f * s[k];
    }
    reverse_f32(p, P_LEN);   /* oldest-first ring buffer용 인과 보정 */
}

static void build_sec_true(float *st, const float *s)
{
    memset(st, 0, W_LEN * sizeof(float));
    for (int k = 0; k < W_LEN; k++) {
        if (k + 2     < W_LEN) st[k + 2] += 1.15f * s[k];
        if (k + 2 + 5 < W_LEN) st[k + 7] += 0.08f * s[k];
    }
    reverse_f32(st, W_LEN);  /* oldest-first ring buffer용 인과 보정 */
}

/* ── 단일 주파수 시뮬레이션 ─────────────────────────────────── */
static void run_sim(const float *s_hat, float freq_hz)
{
    const int warmup_n = FS * WARMUP_SEC;
    const int total_n  = FS * (WARMUP_SEC + RUN_SEC);
    const int win_n    = (int)(FS * WIN_SEC);

    float p[P_LEN], st[W_LEN];
    build_primary(p, s_hat);
    build_sec_true(st, s_hat);

    /* 식물(플랜트) 링버퍼: 참조 + 각 방식의 anti-noise */
    float x_pbuf[2 * P_LEN];
    float ya_buf[2 * W_LEN], yb_buf[2 * W_LEN], yc_buf[2 * W_LEN];
    memset(x_pbuf, 0, sizeof(x_pbuf));
    memset(ya_buf, 0, sizeof(ya_buf));
    memset(yb_buf, 0, sizeof(yb_buf));
    memset(yc_buf, 0, sizeof(yc_buf));
    int xpp = 0, yap = 0, ybp = 0, ycp = 0;

    sim_t fa, fb, fc;
    sim_init(&fa, s_hat, MU_0);          /* A: NLMS          */
    sim_init(&fb, s_hat, MU_FLOOR);      /* B: VSS (mu 작게 시작) */
    sim_init(&fc, s_hat, MU_0);          /* C: NLMS+VSS      */

    double ref_sq = 0.0;
    int    ref_cnt = 0;

    double wa = 0, wb = 0, wc = 0, wd = 0;
    int    wcnt = 0;

    uint32_t seed = 7;

    for (int n = 0; n < total_n; n++) {
        float t   = (float)n / FS;
        float x   = 0.45f * sinf(2.f * (float)M_PI * freq_hz * t)
                  + 0.01f * white_noise(&seed);
        int adapt = (n >= warmup_n);

        /* d(n): 일차경로 출력 */
        rbuf_push(x_pbuf, &xpp, P_LEN, x);
        float d = dotf(p, rbuf_ptr(x_pbuf, xpp), P_LEN);

        /* 각 방식의 이차경로 기여: S_true * y_prev */
        float ap_a = dotf(st, rbuf_ptr(ya_buf, yap), W_LEN);
        float ap_b = dotf(st, rbuf_ptr(yb_buf, ybp), W_LEN);
        float ap_c = dotf(st, rbuf_ptr(yc_buf, ycp), W_LEN);

        float ea = d + ap_a;
        float eb = d + ap_b;
        float ec = d + ap_c;

        float ya = clampf(step_nlms    (&fa, x, ea, adapt), -OUT_LIMIT, OUT_LIMIT);
        float yb = clampf(step_vss     (&fb, x, eb, adapt), -OUT_LIMIT, OUT_LIMIT);
        float yc = clampf(step_vss_nlms(&fc, x, ec, adapt), -OUT_LIMIT, OUT_LIMIT);

        rbuf_push(ya_buf, &yap, W_LEN, ya);
        rbuf_push(yb_buf, &ybp, W_LEN, yb);
        rbuf_push(yc_buf, &ycp, W_LEN, yc);

        /* 기준 RMS: 워밍업 전체 */
        if (!adapt) {
            ref_sq += d * d;
            ref_cnt++;
        }

        /* 윈도우 누적 */
        wa   += ea * ea;
        wb   += eb * eb;
        wc   += ec * ec;
        wd   += d  * d;
        wcnt++;

        if (adapt && wcnt >= win_n) {
            double ref_rms = sqrt(ref_sq / (ref_cnt > 0 ? ref_cnt : 1));
            double rms_a = sqrt(wa / wcnt);
            double rms_b = sqrt(wb / wcnt);
            double rms_c = sqrt(wc / wcnt);
            double rms_d = sqrt(wd / wcnt);

#define TO_DB(r) ((ref_rms > 1e-12 && (r) > 1e-12) ? \
                  20.0 * log10((r) / ref_rms) : 0.0)

            printf("%.0f,%.2f,%.3f,%.3f,%.3f,%.3f,"
                   "%.5f,%.5f,%.5f\n",
                   freq_hz,
                   t - WARMUP_SEC,
                   TO_DB(rms_a),   /* A: NLMS          */
                   TO_DB(rms_b),   /* B: VSS           */
                   TO_DB(rms_c),   /* C: NLMS+VSS      */
                   TO_DB(rms_d),   /* D: no-adapt 기준 */
                   fa.last_mu_n,
                   fb.last_mu_n,
                   fc.last_mu_n);
#undef TO_DB
            wa = wb = wc = wd = 0.0;
            wcnt = 0;
        }
    }
}

/* ── main ───────────────────────────────────────────────────── */
int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1] : "runtime/sec_path.bin";

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        fprintf(stderr, "ERROR: cannot open %s\n", path);
        return 1;
    }
    int hdr[2];
    if (fread(hdr, sizeof(int), 2, fp) != 2) {
        fprintf(stderr, "ERROR: bad header\n"); fclose(fp); return 1;
    }
    int s_len = hdr[0];
    float *s_hat = (float *)calloc(W_LEN, sizeof(float));
    int load = s_len < W_LEN ? s_len : W_LEN;
    if ((int)fread(s_hat, sizeof(float), load, fp) != load) {
        fprintf(stderr, "ERROR: short read\n"); fclose(fp); free(s_hat); return 1;
    }
    fclose(fp);

    fprintf(stderr,
            "sec_path: s_len=%d  rate=%d\n"
            "params: mu0=%.4f  nlms_mu_n_max=%.4f  "
            "vss_alpha=%.6f  vss_gamma_pure=%.2e  vss_mu_max=%.4f\n"
            "warmup=%ds  run=%ds  window=%.2fs\n\n",
            s_len, hdr[1],
            MU_0, MU_N_MAX,
            VSS_ALPHA, (double)VSS_GAMMA_PURE, VSS_MU_MAX_PURE,
            WARMUP_SEC, RUN_SEC, (double)WIN_SEC);

    printf("freq_hz,time_s,"
           "nlms_db,vss_db,vss_nlms_db,no_adapt_db,"
           "nlms_mu_n,vss_mu,vss_nlms_mu_n\n");

    float freqs[] = {80.f, 100.f, 120.f, 150.f};
    int   nf      = (int)(sizeof(freqs) / sizeof(freqs[0]));
    for (int i = 0; i < nf; i++) {
        fprintf(stderr, "simulating %.0f Hz ...\n", freqs[i]);
        run_sim(s_hat, freqs[i]);
    }

    free(s_hat);
    fprintf(stderr, "done.\n");
    return 0;
}
