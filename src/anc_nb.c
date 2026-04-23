#include "anc_nb.h"
#include "anc_dsp.h"
#include "anc_algo.h"

void run_nb_anc(alsa_ctx_t *a, const float *sec_path, int sec_len,
                logger_t *l, int n_harm, float nb_mu, float nb_leak,
                int start_fill_periods, int xrun_fill_periods,
                const anc_runtime_cfg_t *cfg)
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

    int    bl_done = 0, bl_sec = 0;
    double bl_err_sum = 0;
    long   bl_count = 0;
    double baseline_err_rms = 0;

    float  best_wc[NB_MAX_HARM] = {0}, best_ws[NB_MAX_HARM] = {0};
    float  best_f0 = 0;
    double best_db = -1e9;
    int    best_valid = 0;

    fprintf(stderr, "NB-ANC: n_harm=%d  mu=%.6f  leak=%.6f  sec_len=%d  "
            "f0_range=%.0f~%.0fHz  period=%lu  budget=%.3fms\n",
            n_harm, nb_mu, nb_leak, sec_len,
            NB_F0_MIN_HZ, NB_F0_MAX_HZ,
            (unsigned long)period, period_budget_ms);

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

        for (int i = 0; i < (int)period; i++)
            nb_f0_buf_sample(&nb, to_f(in_buf[i * 2 + REF_CH]));

        int f0_changed = nb_f0_update(&nb);
        if (f0_changed && nb.f0_hz > 0.0f)
            fprintf(stderr, "  f0=%.1fHz conf=%.2f\n", nb.f0_hz, nb.f0_conf);

        for (int i = 0; i < (int)period; i++) {
            float e   = to_f(in_buf[i * 2 + ERR_CH]);

            float anti = nb_step(&nb, e);
            anti = output_safety_step(&out_safety, anti);

            int clipped = 0;
            if (anti >  cfg->output_limit) { anti =  cfg->output_limit; clipped = 1; }
            if (anti < -cfg->output_limit) { anti = -cfg->output_limit; clipped = 1; }

            out_buf[i * 2 + 0] = clip16(anti);
            out_buf[i * 2 + 1] = clip16(anti);

            logger_update(l, e, anti, e, clipped);

            if (!bl_done) {
                bl_err_sum += (double)e * e;
                bl_count++;
            }
        }
        double after_compute = get_time();

        logger_period_times(l,
                            after_read - after_write,
                            after_compute - after_read,
                            after_write - loop_start,
                            after_compute - loop_start);

        if (logger_should_print(l)) {
            double elapsed = get_time() - t_start;
            float  wn = nb_w_norm(&nb);
            double clip_frac = 0;
            double tone_db = 0, track_db = 0;

            char status[32];
            if (l->frozen) snprintf(status, sizeof(status), "FROZEN");
            else if (!bl_done) snprintf(status, sizeof(status), "BL%d/%d", bl_sec + 1, cfg->baseline_secs);
            else if (nb.adapt) snprintf(status, sizeof(status), "adapt");
            else snprintf(status, sizeof(status), "fixed");

            float nb_sx = (nb.f0_hz > 0 && nb.harm[0].sx_mag > 0)
                          ? nb.harm[0].sx_mag : 0.0f;
            float nb_mu_n = nb.mu / (nb_sx * nb_sx + NB_NLMS_EPS);
            double cur_err_rms = logger_print(l, wn, elapsed, nb.mu, (double)nb_mu_n,
                                              0.0, 0.0,
                                              n_harm, 2, sec_len,
                                              period_budget_ms, status,
                                              &clip_frac, &tone_db, &track_db);

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
                if (bl_sec >= cfg->baseline_secs) {
                    baseline_err_rms = sqrt(bl_err_sum / bl_count);
                    l->baseline_err_rms = baseline_err_rms;
                    l->baseline_tone_rms = baseline_err_rms;
                    bl_done = 1;
                    nb.adapt = 1;
                    fprintf(stderr, "=== Baseline: err_rms=%.6f (%d sec) -> Adaptation ON ===\n",
                            baseline_err_rms, cfg->baseline_secs);
                }
            }

            if (bl_done && nb.adapt && tone_db > 0.0 && tone_db > best_db) {
                for (int h = 0; h < nb.n_harm; h++) {
                    best_wc[h] = nb.harm[h].w_c;
                    best_ws[h] = nb.harm[h].w_s;
                }
                best_f0 = nb.f0_hz;
                best_db = tone_db;
                best_valid = 1;
            }

            if (bl_done && nb.adapt && !l->frozen) {
                int diverged = 0;
                if (baseline_err_rms > 0 &&
                    cur_err_rms > baseline_err_rms * cfg->err_diverge) {
                    diverged = 1;
                    fprintf(stderr, "DIVERGENCE [ err=%.4f>base*%.1f ]",
                            cur_err_rms, cfg->err_diverge);
                }
                if (wn > cfg->w_norm_max) {
                    diverged = 1;
                    fprintf(stderr, "DIVERGENCE [ w_norm=%.1f>%.1f ]",
                            wn, cfg->w_norm_max);
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
                        l->recover_sec = cfg->recover_secs;
                    } else {
                        fprintf(stderr, " -> reset + FROZEN\n");
                        nb_reset_weights(&nb);
                        nb.adapt = 0;
                        l->frozen = 1;
                        l->recover_sec = cfg->recover_secs;
                    }
                }
            }

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
