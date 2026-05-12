#include "anc_nb.h"
#include "anc_dsp.h"
#include "anc_algo.h"

void run_nb_anc(alsa_ctx_t *a, const float *sec_path, int sec_len,
                logger_t *l, int n_harm, float nb_mu, float nb_leak,
                int start_fill_periods, int xrun_fill_periods,
                const anc_runtime_cfg_t *cfg, int record_secs,
                float snore_f0, int snore_n_harm, const char *snore_file)
{
    snd_pcm_uframes_t period = a->period;
    double period_budget_ms = (double)period * 1000.0 / (double)a->rate;

    /* Recording setup */
    long rec_total = (record_secs > 0) ? (long)record_secs * (long)a->rate : 0;
    long rec_count = 0;
    FILE *f_ref = NULL, *f_err = NULL;
    if (rec_total > 0) {
        f_ref = fopen("/tmp/rec_ref.raw", "wb");
        f_err = fopen("/tmp/rec_err.raw", "wb");
        if (!f_ref || !f_err)
            fprintf(stderr, "WARNING: cannot open recording files\n");
        else
            fprintf(stderr, "REC: recording %d sec -> /tmp/rec_ref.raw, /tmp/rec_err.raw\n",
                    record_secs);
    }

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

    /* Snore file playback (raw int16 mono, same rate as ALSA) */
    FILE *f_snore = NULL;
    if (snore_file) {
        f_snore = fopen(snore_file, "rb");
        if (!f_snore)
            fprintf(stderr, "WARNING: cannot open snore file: %s\n", snore_file);
        else
            fprintf(stderr, "SNORE-FILE: %s\n", snore_file);
    }

    /* Noise source on L channel — snoring sim or fallback 150Hz tone */
#define SNORE_MAX_HARM 8
    float tone_amp = 0.3f;
    int   snore_active = (snore_f0 > 0.0f);
    int   snh = snore_active
                ? (snore_n_harm > 0 ? (snore_n_harm < SNORE_MAX_HARM ? snore_n_harm : SNORE_MAX_HARM) : 1)
                : 0;
    float snore_phase[SNORE_MAX_HARM] = {0};
    float snore_phase_inc[SNORE_MAX_HARM] = {0};
    if (snore_active) {
        for (int h = 0; h < snh; h++)
            snore_phase_inc[h] = 2.0f * (float)M_PI * snore_f0 * (h + 1) / (float)a->rate;
        fprintf(stderr, "SNORE-SIM: f0=%.1fHz  n_harm=%d  amp=%.2f\n",
                snore_f0, snh, tone_amp);
    }
    /* fallback single tone (used when snore_f0==0) */
    float tone_hz    = 150.0f;
    float tone_phase = 0.0f;
    float tone_phase_inc = 2.0f * (float)M_PI * tone_hz / (float)a->rate;

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

        /* Record ref and error mic samples */
        if (f_ref && f_err && rec_count < rec_total) {
            for (int i = 0; i < (int)period && rec_count < rec_total; i++, rec_count++) {
                int16_t s_ref = in_buf[i * 2 + REF_CH];
                int16_t s_err = in_buf[i * 2 + ERR_CH];
                fwrite(&s_ref, sizeof(int16_t), 1, f_ref);
                fwrite(&s_err, sizeof(int16_t), 1, f_err);
            }
            if (rec_count >= rec_total) {
                fclose(f_ref); f_ref = NULL;
                fclose(f_err); f_err = NULL;
                fprintf(stderr, "REC: done. saved %ld samples each\n", rec_count);
            }
        }

        for (int i = 0; i < (int)period; i++)
            nb_f0_buf_sample(&nb, to_f(in_buf[i * 2 + REF_CH]));

        int f0_changed = nb_f0_update(&nb);
        if (f0_changed && nb.f0_hz > 0.0f)
            fprintf(stderr, "  f0=%.1fHz conf=%.2f\n", nb.f0_hz, nb.f0_conf);

        /* 코골이 활성 여부: f0 검출 + conf 충분 */
        int f0_active = (nb.f0_hz > 0.0f && nb.f0_conf >= NB_F0_CONF_THR);

        for (int i = 0; i < (int)period; i++) {
            float e   = to_f(in_buf[i * 2 + ERR_CH]);

            float anti = nb_step(&nb, e);
            anti = output_safety_step(&out_safety, anti);

            int clipped = 0;
            if (anti >  cfg->output_limit) { anti =  cfg->output_limit; clipped = 1; }
            if (anti < -cfg->output_limit) { anti = -cfg->output_limit; clipped = 1; }

            float noise_out;
            if (f_snore) {
                int16_t s16 = 0;
                if (fread(&s16, sizeof(int16_t), 1, f_snore) < 1) {
                    rewind(f_snore);  /* loop */
                    fread(&s16, sizeof(int16_t), 1, f_snore);
                }
                noise_out = (float)s16 / 32768.0f;
            } else if (snore_active) {
                float s = 0.0f;
                float amp_per = tone_amp / (float)snh;
                for (int h = 0; h < snh; h++) {
                    s += amp_per * sinf(snore_phase[h]);
                    snore_phase[h] += snore_phase_inc[h];
                    if (snore_phase[h] > (float)M_PI) snore_phase[h] -= 2.0f * (float)M_PI;
                }
                noise_out = s;
            } else {
                noise_out = tone_amp * sinf(tone_phase);
                tone_phase += tone_phase_inc;
                if (tone_phase > (float)M_PI) tone_phase -= 2.0f * (float)M_PI;
            }

            out_buf[i * 2 + 0] = clip16(noise_out); /* L: noise source (snore sim or 150Hz) */
            out_buf[i * 2 + 1] = clip16(anti);      /* R: ANC anti-noise  */

            logger_update(l, e, anti, e, clipped);

            if (!bl_done && f0_active) {
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
            else if (!bl_done && !f0_active) snprintf(status, sizeof(status), "SILENT");
            else if (!bl_done) snprintf(status, sizeof(status), "BL%d/%d", bl_sec + 1, cfg->baseline_secs);
            else if (!f0_active) snprintf(status, sizeof(status), "SILENT");
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
                if (f0_active) bl_sec++;
                if (bl_sec >= cfg->baseline_secs && bl_count > 0) {
                    baseline_err_rms = sqrt(bl_err_sum / bl_count);
                    l->baseline_err_rms = baseline_err_rms;
                    l->baseline_tone_rms = baseline_err_rms;
                    bl_done = 1;
                    nb.adapt = 1;
                    fprintf(stderr, "=== Baseline: err_rms=%.6f (%d sec) -> Adaptation ON ===\n",
                            baseline_err_rms, cfg->baseline_secs);
                }
            }

            if (bl_done && nb.adapt && f0_active && tone_db > 0.0 && tone_db > best_db) {
                for (int h = 0; h < nb.n_harm; h++) {
                    best_wc[h] = nb.harm[h].w_c;
                    best_ws[h] = nb.harm[h].w_s;
                }
                best_f0 = nb.f0_hz;
                best_db = tone_db;
                best_valid = 1;
            }

            if (bl_done && nb.adapt && !l->frozen && f0_active) {
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
    if (f_ref) fclose(f_ref);
    if (f_err) fclose(f_err);
    if (f_snore) fclose(f_snore);
}
