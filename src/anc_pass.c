#include "anc_pass.h"
#include "anc_dsp.h"

void run_passthrough(alsa_ctx_t *a, logger_t *l,
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
