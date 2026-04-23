/*
 * anc_measure.h — 이차 경로 측정 모드.
 *
 * 포함 내용:
 *   - run_measurement: 화이트 노이즈 → LMS 시스템 식별 → sec_path.bin 저장
 *     (Phase 1: ambient xcorr, Phase 2: LMS 경로 추정)
 *
 * 의존성: anc_defs.h, anc_dsp.h, anc_algo.h, anc_io.h, anc_log.h
 */
#ifndef ANC_MEASURE_H
#define ANC_MEASURE_H

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

#endif /* ANC_MEASURE_H */
