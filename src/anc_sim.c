#include "anc_sim.h"
#include "anc_dsp.h"
#include "anc_algo.h"

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

double simulate_one_freq(const float *s_hat, int s_len, float freq_hz,
                          float mu, float leakage, float mu_n_max,
                          float eps_abs, float eps_rel,
                          float band_lo_hz, float band_hi_hz,
                          int mb_bands, float output_limit)
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
        anti = clampf(anti, -output_limit, output_limit);
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

int run_sim_sweep(float tone_hz,
                   float band_lo_hz, float band_hi_hz, int mb_bands,
                   float output_limit)
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
                                                          mb_bands, output_limit);
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
