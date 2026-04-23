#include "anc_app.h"
#include "anc_dsp.h"
#include "anc_algo.h"
#include "anc_io.h"
#include "anc_log.h"
#include "anc_measure.h"
#include "anc_run.h"
#include "anc_nb.h"
#include "anc_pass.h"
#include "anc_sim.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

typedef struct {
    float mu;
    float leak;
    float mu_n_max;
    float eps_abs;
    float eps_rel;
    float tone_hz;
    float band_lo_hz;
    float band_hi_hz;
    int   mb_bands;
    int   req_period;
    int   buffer_mult;
    int   start_fill_periods;
    int   xrun_fill_periods;
    int   adapt_delay_ms;
    int   measure_secs;
    int   capture_secs;
    float measure_noise_amp;
    int   nb_harms;
    float nb_mu;
    float nb_leak;
} tune_cfg_t;

static void tune_cfg_init(tune_cfg_t *cfg)
{
    cfg->mu = MU_DEFAULT;
    cfg->leak = LEAKAGE_DEFAULT;
    cfg->mu_n_max = MU_N_MAX;
    cfg->eps_abs = EPSILON_ABS;
    cfg->eps_rel = EPSILON_REL;
    cfg->tone_hz = TONE_TRACK_HZ_DEFAULT;
    cfg->band_lo_hz = TARGET_BAND_LO_HZ;
    cfg->band_hi_hz = TARGET_BAND_HI_HZ;
    cfg->mb_bands = MBAND_BANDS_DEFAULT;
    cfg->req_period = REQ_PERIOD;
    cfg->buffer_mult = REQ_BUFFER / REQ_PERIOD;
    cfg->start_fill_periods = START_FILL_PERIODS;
    cfg->xrun_fill_periods = XRUN_FILL_PERIODS;
    cfg->adapt_delay_ms = ADAPT_DELAY_MS_DEFAULT;
    cfg->measure_secs = MEASURE_SECS;
    cfg->capture_secs = CAPTURE_BENCH_SECS_DEFAULT;
    cfg->measure_noise_amp = NOISE_AMP;
    cfg->nb_harms = 4;
    cfg->nb_mu    = NB_MU_DEFAULT;
    cfg->nb_leak  = NB_LEAK_DEFAULT;
}

static void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s <mode> [options]\n"
            "  passthrough           capture -> playback echo-back\n"
            "  capture-bench         capture-only I/O timing benchmark\n"
            "  measure               estimate secondary path -> %s\n"
            "  run [--no-adapt]      FxLMS active noise cancellation\n"
            "  nb                    narrowband ANC (pitch-tracking)\n"
            "  sim [--sweep]         offline tuning check\n"
            "Options: --mu=F --leak=F --mu-n-max=F --eps-abs=F --eps-rel=F\n"
            "         --tone-hz=F --band-lo=F --band-hi=F --mb-bands=N\n"
            "         --period=N --buffer-mult=N --start-fill=N --xrun-fill=N\n"
            "         --adapt-delay-ms=N --n-harm=N --nb-mu=F --nb-leak=F\n"
            "         --measure-secs=N --capture-secs=N --measure-noise=F\n",
            prog, SEC_PATH_FILE);
}

int anc_app_main(int argc, char **argv, const anc_runtime_cfg_t *rt_cfg)
{
    if (argc < 2) { usage(argv[0]); return 1; }

    const char *mode = argv[1];
    int no_adapt = 0;
    int sim_sweep = 0;
    tune_cfg_t cfg;
    tune_cfg_init(&cfg);

    for (int i = 2; i < argc; i++) {
        const char *arg = argv[i];
        if (strcmp(arg, "--no-adapt") == 0) {
            no_adapt = 1;
        } else if (strcmp(arg, "--sweep") == 0) {
            sim_sweep = 1;
        } else if (strncmp(arg, "--mu=", 5) == 0) {
            cfg.mu = strtof(arg + 5, NULL);
        } else if (strncmp(arg, "--leak=", 7) == 0) {
            cfg.leak = strtof(arg + 7, NULL);
        } else if (strncmp(arg, "--mu-n-max=", 11) == 0) {
            cfg.mu_n_max = strtof(arg + 11, NULL);
        } else if (strncmp(arg, "--eps-abs=", 10) == 0) {
            cfg.eps_abs = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--eps-rel=", 10) == 0) {
            cfg.eps_rel = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--tone-hz=", 10) == 0) {
            cfg.tone_hz = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--band-lo=", 10) == 0) {
            cfg.band_lo_hz = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--band-hi=", 10) == 0) {
            cfg.band_hi_hz = strtof(arg + 10, NULL);
        } else if (strncmp(arg, "--mb-bands=", 11) == 0) {
            cfg.mb_bands = atoi(arg + 11);
        } else if (strncmp(arg, "--period=", 9) == 0) {
            cfg.req_period = atoi(arg + 9);
        } else if (strncmp(arg, "--buffer-mult=", 14) == 0) {
            cfg.buffer_mult = atoi(arg + 14);
        } else if (strncmp(arg, "--start-fill=", 13) == 0) {
            cfg.start_fill_periods = atoi(arg + 13);
        } else if (strncmp(arg, "--xrun-fill=", 12) == 0) {
            cfg.xrun_fill_periods = atoi(arg + 12);
        } else if (strncmp(arg, "--adapt-delay-ms=", 17) == 0) {
            cfg.adapt_delay_ms = atoi(arg + 17);
        } else if (strncmp(arg, "--measure-secs=", 15) == 0) {
            cfg.measure_secs = atoi(arg + 15);
        } else if (strncmp(arg, "--capture-secs=", 15) == 0) {
            cfg.capture_secs = atoi(arg + 15);
        } else if (strncmp(arg, "--measure-noise=", 16) == 0) {
            cfg.measure_noise_amp = strtof(arg + 16, NULL);
        } else if (strncmp(arg, "--n-harm=", 9) == 0) {
            cfg.nb_harms = atoi(arg + 9);
        } else if (strncmp(arg, "--nb-mu=", 8) == 0) {
            cfg.nb_mu = strtof(arg + 8, NULL);
        } else if (strncmp(arg, "--nb-leak=", 10) == 0) {
            cfg.nb_leak = strtof(arg + 10, NULL);
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (cfg.band_lo_hz <= 0.0f || cfg.band_hi_hz <= cfg.band_lo_hz) {
        fprintf(stderr, "ERROR: invalid band range %.1f~%.1f Hz\n",
                cfg.band_lo_hz, cfg.band_hi_hz);
        return 1;
    }
    if (cfg.mb_bands < 1 || cfg.mb_bands > MBAND_BANDS_MAX) {
        fprintf(stderr, "ERROR: invalid mb-bands %d (valid 1..%d)\n",
                cfg.mb_bands, MBAND_BANDS_MAX);
        return 1;
    }
    if (cfg.req_period < 16) {
        fprintf(stderr, "ERROR: invalid period %d\n", cfg.req_period);
        return 1;
    }
    if (cfg.buffer_mult < 2) {
        fprintf(stderr, "ERROR: invalid buffer-mult %d\n", cfg.buffer_mult);
        return 1;
    }
    if (cfg.start_fill_periods < 1 || cfg.start_fill_periods > cfg.buffer_mult) {
        fprintf(stderr, "ERROR: invalid start-fill %d (buffer-mult=%d)\n",
                cfg.start_fill_periods, cfg.buffer_mult);
        return 1;
    }
    if (cfg.xrun_fill_periods < 1 || cfg.xrun_fill_periods > cfg.buffer_mult) {
        fprintf(stderr, "ERROR: invalid xrun-fill %d (buffer-mult=%d)\n",
                cfg.xrun_fill_periods, cfg.buffer_mult);
        return 1;
    }
    if (cfg.adapt_delay_ms < 0 || cfg.adapt_delay_ms > 10000) {
        fprintf(stderr, "ERROR: invalid adapt-delay-ms %d\n",
                cfg.adapt_delay_ms);
        return 1;
    }
    if (cfg.measure_secs < 1) {
        fprintf(stderr, "ERROR: invalid measure-secs %d\n", cfg.measure_secs);
        return 1;
    }
    if (cfg.capture_secs < 0) {
        fprintf(stderr, "ERROR: invalid capture-secs %d\n", cfg.capture_secs);
        return 1;
    }
    if (cfg.measure_noise_amp <= 0.0f || cfg.measure_noise_amp > 1.0f) {
        fprintf(stderr, "ERROR: invalid measure-noise %.3f\n",
                cfg.measure_noise_amp);
        return 1;
    }

    if (strcmp(mode, "sim") == 0) {
        if (!sim_sweep) {
            fxlms_t tmp;
            fxlms_init(&tmp, W_LEN_DEFAULT, S_LEN_DEFAULT, cfg.mu, cfg.leak);
            fxlms_set_band(&tmp, cfg.band_lo_hz, cfg.band_hi_hz);
            if (fxlms_load_sec_default(&tmp) < 0) {
                fprintf(stderr, "ERROR: missing %s. run './anc measure' first.\n", SEC_PATH_FILE);
                fxlms_free(&tmp);
                return 1;
            }

            const float default_freqs[] = { 80.0f, 100.0f, 120.0f, 150.0f };
            float single_freq[1];
            const float *freqs = default_freqs;
            int n_freq = 4;
            if (cfg.tone_hz > 0.0f) {
                single_freq[0] = cfg.tone_hz;
                freqs = single_freq;
                n_freq = 1;
            }
            double worst_db = 1e9;
            for (int i = 0; i < n_freq; i++) {
                double db = simulate_one_freq(tmp.s, tmp.s_len, freqs[i],
                                              cfg.mu, cfg.leak, cfg.mu_n_max,
                                              cfg.eps_abs, cfg.eps_rel,
                                              cfg.band_lo_hz, cfg.band_hi_hz,
                                              cfg.mb_bands, rt_cfg->output_limit);
                if (db < worst_db) worst_db = db;
                fprintf(stderr, "[SIM %.0fHz] reduction=%+.2fdB\n", freqs[i], db);
            }
            fprintf(stderr,
                    "[SIM config] mu=%.5f leak=%.6f mu_n_max=%.3f eps_abs=%.4g eps_rel=%.4g tone_hz=%.1f band=%.0f~%.0fHz mb_bands=%d worst=%+.2fdB\n",
                    cfg.mu, cfg.leak, cfg.mu_n_max, cfg.eps_abs, cfg.eps_rel,
                    cfg.tone_hz, cfg.band_lo_hz, cfg.band_hi_hz,
                    cfg.mb_bands, worst_db);
            fxlms_free(&tmp);
            return 0;
        }
        return run_sim_sweep(cfg.tone_hz, cfg.band_lo_hz, cfg.band_hi_hz,
                              cfg.mb_bands, rt_cfg->output_limit) == 0 ? 0 : 1;
    }

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sig_handler;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    int need_lock_future = !(strcmp(mode, "measure") == 0 ||
                              strcmp(mode, "sim")     == 0 ||
                              strcmp(mode, "passthrough") == 0 ||
                              strcmp(mode, "capture-bench") == 0);
    rt_prepare_memory(need_lock_future);

    alsa_ctx_t alsa;
    if (strcmp(mode, "capture-bench") == 0) {
        if (alsa_init_capture_only(&alsa, DEVICE, SAMPLE_RATE,
                                   (snd_pcm_uframes_t)cfg.req_period,
                                   (snd_pcm_uframes_t)(cfg.req_period * cfg.buffer_mult)) < 0)
            return 1;
    } else {
        if (alsa_init(&alsa, DEVICE, SAMPLE_RATE,
                      (snd_pcm_uframes_t)cfg.req_period,
                      (snd_pcm_uframes_t)(cfg.req_period * cfg.buffer_mult)) < 0)
            return 1;
    }
    alsa_print(&alsa);

    logger_t log;
    logger_init(&log, alsa.rate, cfg.tone_hz);

    if (strcmp(mode, "passthrough") == 0) {
        run_passthrough(&alsa, &log,
                        cfg.start_fill_periods, cfg.xrun_fill_periods);
    }
    else if (strcmp(mode, "capture-bench") == 0) {
        run_capture_bench(&alsa, cfg.capture_secs);
    }
    else if (strcmp(mode, "measure") == 0) {
        run_measurement(&alsa, S_LEN_DEFAULT, cfg.measure_noise_amp,
                        cfg.measure_secs, SEC_PATH_FILE,
                        cfg.start_fill_periods, cfg.xrun_fill_periods);
    }
    else if (strcmp(mode, "run") == 0) {
        mbfxlms_t mb;
        mbfxlms_init(&mb, cfg.mb_bands, W_LEN_DEFAULT, S_LEN_DEFAULT,
                     cfg.mu, cfg.leak, cfg.band_lo_hz, cfg.band_hi_hz);
        mbfxlms_set_mu_n_max(&mb, cfg.mu_n_max);
        mbfxlms_set_eps(&mb, cfg.eps_abs, cfg.eps_rel);

        if (mbfxlms_load_sec_default(&mb) < 0) {
            fprintf(stderr, "ERROR: run './anc measure' first.\n");
            mbfxlms_free(&mb);
            alsa_close(&alsa);
            return 1;
        }

        run_anc(&alsa, &mb, &log, no_adapt,
                cfg.band_lo_hz, cfg.band_hi_hz,
                cfg.start_fill_periods, cfg.xrun_fill_periods,
                cfg.adapt_delay_ms, rt_cfg);
        mbfxlms_free(&mb);
    }
    else if (strcmp(mode, "nb") == 0) {
        fxlms_t tmp;
        fxlms_init(&tmp, 2, S_LEN_DEFAULT, 0.01f, 1.0f);
        if (fxlms_load_sec_default(&tmp) < 0) {
            fprintf(stderr, "ERROR: run './anc measure' first.\n");
            fxlms_free(&tmp);
            alsa_close(&alsa);
            return 1;
        }
        run_nb_anc(&alsa, tmp.s, tmp.s_len, &log,
                    cfg.nb_harms, cfg.nb_mu, cfg.nb_leak,
                    cfg.start_fill_periods, cfg.xrun_fill_periods,
                    rt_cfg);
        fxlms_free(&tmp);
    }
    else {
        usage(argv[0]);
        alsa_close(&alsa);
        return 1;
    }

    fprintf(stderr, "Shutting down...\n");
    alsa_close(&alsa);
    return 0;
}
