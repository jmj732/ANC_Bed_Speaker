#include "anc_run.h"
#include "anc_dsp.h"

void run_anc(alsa_ctx_t *a, mbfxlms_t *m, logger_t *l, int no_adapt,
             float band_lo_hz, float band_hi_hz,
             int start_fill_periods, int xrun_fill_periods,
             int adapt_delay_ms,
             const anc_runtime_cfg_t *cfg)
{
    snd_pcm_uframes_t period = a->period;
    double period_budget_ms = (double)period * 1000.0 / (double)a->rate;

    int16_t *in_buf  = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    int16_t *out_buf = (int16_t *)calloc(period * CHANNELS, sizeof(int16_t));
    band_ctl_t err_mon;
    output_safety_t out_safety;
    band_ctl_init(&err_mon, band_lo_hz, band_hi_hz, (float)a->rate);
    output_safety_init(&out_safety, (float)a->rate);

    if (alsa_resync(a, out_buf, start_fill_periods) < 0) {
        free(in_buf);
        free(out_buf);
        return;
    }

    double t_start = get_time();

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
            mbfxlms_mu(m), m->bands[0].leakage, cfg->baseline_secs,
            band_lo_hz, band_hi_hz, l->tone_track.freq_hz, no_adapt,
            (unsigned long)a->period, (unsigned long)a->buf_size,
            start_fill_periods, xrun_fill_periods, adapt_delay_ms);
    mbfxlms_print_bands(m);

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

            int clipped = 0;
            if (anti >  cfg->output_limit) { anti =  cfg->output_limit; clipped = 1; }
            if (anti < -cfg->output_limit) { anti = -cfg->output_limit; clipped = 1; }

            out_buf[i * 2 + 0] = clip16(anti);
            out_buf[i * 2 + 1] = clip16(anti);

            logger_update(l, e, anti, e_tone, clipped);

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

        if (logger_should_print(l)) {
            double elapsed = get_time() - t_start;
            float  wn = mbfxlms_w_norm(m);

            char status[32];
            if (l->frozen)
                snprintf(status, sizeof(status), "FROZEN");
            else if (!warmup_done)
                snprintf(status, sizeof(status), "WU%d/%d",
                         warmup_sec + 1, cfg->warmup_secs);
            else if (!bl_done)
                snprintf(status, sizeof(status), "BL%d/%d",
                         bl_sec + 1, cfg->baseline_secs);
            else if (adapt_delay_active)
                snprintf(status, sizeof(status), "ADLY");
            else if (m->bands[0].adapt)
                snprintf(status, sizeof(status), "adapt");
            else
                snprintf(status, sizeof(status), "fixed");

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

            if (!warmup_done) {
                warmup_sec++;
                if (warmup_sec >= cfg->warmup_secs) {
                    warmup_done = 1;
                    fprintf(stderr, "=== Warm-up done, baseline capture start ===\n");
                }
            } else if (!bl_done) {
                bl_sec++;
                if (bl_sec >= cfg->baseline_secs) {
                    l->baseline_err_rms = sqrt(bl_err_sum / bl_count);
                    l->baseline_tone_rms = sqrt(bl_tone_sum / bl_count);
                    l->baseline_track_rms = tone_meter_rms(&baseline_track);
                    bl_done = 1;
                    fprintf(stderr, "=== Baseline: err_rms=%.6f "
                            " tone_rms=%.6f  track@%.1f=%.6f (%d sec, %ld samples) ===\n",
                            l->baseline_err_rms, l->baseline_tone_rms,
                            l->tone_track.freq_hz, l->baseline_track_rms,
                            cfg->baseline_secs, bl_count);
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

            if (bl_done && m->bands[0].adapt && !l->frozen) {
                int causes = 0;
                if (wn > cfg->w_norm_max)
                    causes |= 1;
                if (l->baseline_err_rms > 0 &&
                    cur_err_rms > l->baseline_err_rms * cfg->err_diverge)
                    causes |= 2;
                if (clip_frac > cfg->clip_diverge_frac)
                    causes |= 4;

                if (causes) {
                    fprintf(stderr, "DIVERGENCE [");
                    if (causes & 1)
                        fprintf(stderr, " w_norm=%.1f>%.1f",
                                wn, cfg->w_norm_max);
                    if (causes & 2)
                        fprintf(stderr, " err=%.4f>base*%.1f",
                                cur_err_rms, cfg->err_diverge);
                    if (causes & 4)
                        fprintf(stderr, " clip=%.1f%%>%.1f%%",
                                clip_frac * 100.0, cfg->clip_diverge_frac * 100.0);
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
                        mbfxlms_scale_mu(m, cfg->tune_down_rate);
                        mbfxlms_scale_mu_n_max(m, 0.75f);
                        mbfxlms_reset_w(m);
                        adapt_delay_active = 0;
                        adapt_delay_frames = 0;
                        l->frozen = 1;
                        l->recover_sec = cfg->recover_secs;
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
                    if (clip_frac > cfg->clip_warn_frac || metric_db < prev_metric_db - 1.0) {
                        mbfxlms_scale_mu(m, cfg->tune_down_rate);
                        mbfxlms_scale_mu_n_max(m, 0.92f);
                    } else if (metric_db > cfg->target_reduction_db + 3.0) {
                        mbfxlms_scale_mu(m, 0.98f);
                        mbfxlms_scale_mu_n_max(m, 0.90f);
                    } else if (metric_db > cfg->target_reduction_db) {
                        mbfxlms_scale_mu_n_max(m, 0.96f);
                    } else if (metric_db > 2.0 &&
                               metric_db > prev_metric_db + 0.5 &&
                               clip_frac < cfg->clip_warn_frac * 0.5) {
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
