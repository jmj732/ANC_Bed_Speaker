/*
 * anc_log.h — 런타임 로깅 및 성능 통계.
 *
 * 포함 내용:
 *   - logger_t: 1초 간격 err/tone/anti RMS + 타이밍 통계 + 기준선 추적
 *   - capture_bench_stats_t: 캡처 전용 벤치마크 통계
 *   - measure_perf_t: 측정 모드 구간별/전체 타이밍
 *   - run_capture_bench: capture-bench 모드 메인 루프
 *
 * 의존성: anc_defs.h, anc_dsp.h, anc_io.h
 */
#ifndef ANC_LOG_H
#define ANC_LOG_H

#include "anc_dsp.h"

/* ===== 런타임 로거 ===== */
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
 * 1초 통계 출력.  이번 구간의 err_rms 반환.
 * out_clip_frac: 카운터 리셋 전 클립 비율.
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

/* ===== 캡처 벤치마크 통계 ===== */
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

/* ===== 측정 모드 타이밍 통계 ===== */
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

#endif /* ANC_LOG_H */
