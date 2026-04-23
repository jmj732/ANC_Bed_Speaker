/*
 * anc_io.h — 실시간 메모리 설정 및 ALSA 입출력.
 *
 * 포함 내용:
 *   - rt_prepare_memory (RT 스케줄링, mlockall, cpu_dma_latency)
 *   - alsa_ctx_t 구조체
 *   - set_hw_params, set_sw_params
 *   - pcm_write_full, pcm_read_full
 *   - alsa_prepare_streams, alsa_prefill_playback
 *   - alsa_start (alsa_resync보다 먼저 정의 — forward 선언 불필요)
 *   - alsa_resync, alsa_handle_io_error
 *   - alsa_init, alsa_init_capture_only
 *   - alsa_print, alsa_close
 *
 * 의존성: anc_defs.h
 */
#ifndef ANC_IO_H
#define ANC_IO_H

/* ===== 실시간 메모리 준비 ===== */
static void rt_prepare_memory(int lock_future)
{
    int mlock_flags = MCL_CURRENT | (lock_future ? MCL_FUTURE : 0);
    if (mlockall(mlock_flags) < 0)
        fprintf(stderr, "WARNING: mlockall failed: %s\n", strerror(errno));

    struct sched_param sp;
    sp.sched_priority = 49;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) < 0)
        fprintf(stderr, "WARNING: SCHED_FIFO failed: %s (run with sudo or set rtprio)\n",
                strerror(errno));
    else
        fprintf(stderr, "RT: SCHED_FIFO priority %d\n", sp.sched_priority);

    /* Prevent CPU from entering deep C-states between periods.
     * /dev/cpu_dma_latency=0 tells the PM layer to stay in C0.
     * The fd must remain open for the lifetime of the process. */
    static int dma_lat_fd = -1;
    dma_lat_fd = open("/dev/cpu_dma_latency", O_WRONLY);
    if (dma_lat_fd < 0) {
        fprintf(stderr, "WARNING: cannot open /dev/cpu_dma_latency: %s\n",
                strerror(errno));
    } else {
        const int32_t lat = 0;
        if (write(dma_lat_fd, &lat, sizeof(lat)) < 0)
            fprintf(stderr, "WARNING: cpu_dma_latency write failed: %s\n",
                    strerror(errno));
        else
            fprintf(stderr, "RT: cpu_dma_latency set to 0 (C-states disabled)\n");
        /* intentionally not closed — closing reverts the setting */
    }

    volatile unsigned char stack_prefault[STACK_PREFAULT_BYTES];
    for (size_t i = 0; i < sizeof(stack_prefault); i += 4096)
        stack_prefault[i] = 0;
    stack_prefault[sizeof(stack_prefault) - 1] = 0;
}

/* ===== ALSA 컨텍스트 ===== */
typedef struct {
    snd_pcm_t          *cap;
    snd_pcm_t          *play;
    snd_pcm_uframes_t   period;
    snd_pcm_uframes_t   buf_size;
    unsigned int         rate;
    int                  linked;
} alsa_ctx_t;

static int set_hw_params(snd_pcm_t *pcm, unsigned int *rate,
                         snd_pcm_uframes_t *period, snd_pcm_uframes_t *buf)
{
    snd_pcm_hw_params_t *p;
    int err;

    snd_pcm_hw_params_alloca(&p);
    snd_pcm_hw_params_any(pcm, p);
    snd_pcm_hw_params_set_rate_resample(pcm, p, 0);
    snd_pcm_hw_params_set_access(pcm, p, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm, p, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcm, p, CHANNELS);
    snd_pcm_hw_params_set_periods_integer(pcm, p);
    snd_pcm_hw_params_set_rate_near(pcm, p, rate, 0);
    snd_pcm_hw_params_set_period_size_near(pcm, p, period, 0);
    snd_pcm_hw_params_set_buffer_size_near(pcm, p, buf);

    if ((err = snd_pcm_hw_params(pcm, p)) < 0) {
        fprintf(stderr, "hw_params: %s\n", snd_strerror(err));
        return err;
    }

    snd_pcm_hw_params_get_rate(p, rate, 0);
    snd_pcm_hw_params_get_period_size(p, period, 0);
    snd_pcm_hw_params_get_buffer_size(p, buf);

    return 0;
}

static int set_sw_params(snd_pcm_t *pcm, snd_pcm_uframes_t period,
                         snd_pcm_uframes_t buf,
                         snd_pcm_uframes_t start_thr)
{
    snd_pcm_sw_params_t *p;
    int err;

    snd_pcm_sw_params_alloca(&p);
    if ((err = snd_pcm_sw_params_current(pcm, p)) < 0) {
        fprintf(stderr, "sw_params_current: %s\n", snd_strerror(err));
        return err;
    }

    if ((err = snd_pcm_sw_params_set_avail_min(pcm, p, period)) < 0) {
        fprintf(stderr, "sw_params avail_min: %s\n", snd_strerror(err));
        return err;
    }
    if ((err = snd_pcm_sw_params_set_start_threshold(pcm, p, start_thr)) < 0) {
        fprintf(stderr, "sw_params start_threshold: %s\n", snd_strerror(err));
        return err;
    }
    if ((err = snd_pcm_sw_params_set_stop_threshold(pcm, p, buf)) < 0) {
        fprintf(stderr, "sw_params stop_threshold: %s\n", snd_strerror(err));
        return err;
    }

    if ((err = snd_pcm_sw_params(pcm, p)) < 0) {
        fprintf(stderr, "sw_params: %s\n", snd_strerror(err));
        return err;
    }
    return 0;
}

static int pcm_write_full(snd_pcm_t *pcm, const int16_t *buf,
                          snd_pcm_uframes_t frames)
{
    const int16_t *ptr = buf;
    snd_pcm_uframes_t left = frames;

    while (left > 0) {
        snd_pcm_sframes_t n = snd_pcm_writei(pcm, ptr, left);
        if (n < 0)
            return (int)n;
        if (n == 0)
            return -EIO;
        ptr  += n * CHANNELS;
        left -= (snd_pcm_uframes_t)n;
    }
    return 0;
}

static int pcm_read_full(snd_pcm_t *pcm, int16_t *buf, snd_pcm_uframes_t frames)
{
    int16_t *ptr = buf;
    snd_pcm_uframes_t left = frames;

    while (left > 0) {
        snd_pcm_sframes_t n = snd_pcm_readi(pcm, ptr, left);
        if (n < 0)
            return (int)n;
        if (n == 0)
            return -EIO;
        ptr  += n * CHANNELS;
        left -= (snd_pcm_uframes_t)n;
    }
    return 0;
}

static int alsa_prepare_streams(alsa_ctx_t *a)
{
    int err;

    if (a->linked || !a->play)
        return snd_pcm_prepare(a->cap);

    if ((err = snd_pcm_prepare(a->play)) < 0)
        return err;
    return snd_pcm_prepare(a->cap);
}

static int alsa_prefill_playback(alsa_ctx_t *a, const int16_t *buf, int periods)
{
    if (!a->play || !buf || periods <= 0)
        return 0;

    for (int i = 0; i < periods; i++) {
        int err = pcm_write_full(a->play, buf, a->period);
        if (err < 0)
            return err;
    }
    return 0;
}

/* alsa_start 정의 — alsa_resync보다 먼저 위치하므로 forward 선언 불필요 */
static void alsa_start(alsa_ctx_t *a)
{
    if (a->linked)
        snd_pcm_start(a->cap);
    else if (a->play) {
        snd_pcm_start(a->play);
        snd_pcm_start(a->cap);
    } else {
        snd_pcm_start(a->cap);
    }
}

static int alsa_resync(alsa_ctx_t *a, int16_t *silence_buf, int fill_periods)
{
    int err;

    if (a->play && silence_buf)
        memset(silence_buf, 0, a->period * CHANNELS * sizeof(int16_t));
    if (a->play)
        snd_pcm_drop(a->play);
    if (a->cap)
        snd_pcm_drop(a->cap);

    if ((err = alsa_prepare_streams(a)) < 0) {
        fprintf(stderr, "prepare after xrun failed: %s\n", snd_strerror(err));
        return err;
    }
    if ((err = alsa_prefill_playback(a, silence_buf, fill_periods)) < 0) {
        fprintf(stderr, "prefill after xrun failed: %s\n", snd_strerror(err));
        return err;
    }
    alsa_start(a);
    return 0;
}

static int alsa_handle_io_error(alsa_ctx_t *a, snd_pcm_t *pcm, const char *stream,
                                int err, int16_t *silence_buf,
                                int xrun_fill_periods)
{
    fprintf(stderr, "xrun (%s): %s\n", stream, snd_strerror(err));
    err = snd_pcm_recover(pcm, err, 0);
    if (err < 0) {
        fprintf(stderr, "recover (%s) failed: %s\n", stream, snd_strerror(err));
        return err;
    }
    return alsa_resync(a, silence_buf, xrun_fill_periods);
}

static int alsa_init(alsa_ctx_t *a, const char *dev, unsigned int req_rate,
                     snd_pcm_uframes_t req_period, snd_pcm_uframes_t req_buf)
{
    int err;
    memset(a, 0, sizeof(*a));

    if ((err = snd_pcm_open(&a->cap, dev, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf(stderr, "capture open: %s\n", snd_strerror(err));
        return err;
    }
    if ((err = snd_pcm_open(&a->play, dev, SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
        fprintf(stderr, "playback open: %s\n", snd_strerror(err));
        snd_pcm_close(a->cap);
        return err;
    }

    unsigned int cap_rate = req_rate, play_rate = req_rate;
    snd_pcm_uframes_t cap_period = req_period, play_period = req_period;
    snd_pcm_uframes_t cap_buf = req_buf, play_buf = req_buf;

    if (set_hw_params(a->cap,  &cap_rate,  &cap_period,  &cap_buf)  < 0) goto fail;
    if (set_hw_params(a->play, &play_rate, &play_period, &play_buf) < 0) goto fail;
    if (set_sw_params(a->cap, cap_period, cap_buf, cap_period) < 0) goto fail;
    if (set_sw_params(a->play, play_period, play_buf, play_buf) < 0) goto fail;

    /* --- Strict parameter validation --- */
    if (cap_rate != play_rate) {
        fprintf(stderr, "FATAL: rate mismatch  capture=%u  playback=%u\n",
                cap_rate, play_rate);
        goto fail;
    }
    if (cap_period != play_period) {
        fprintf(stderr, "FATAL: period mismatch  capture=%lu  playback=%lu\n",
                (unsigned long)cap_period, (unsigned long)play_period);
        goto fail;
    }
    if (cap_buf != play_buf) {
        fprintf(stderr, "FATAL: buffer mismatch  capture=%lu  playback=%lu\n",
                (unsigned long)cap_buf, (unsigned long)play_buf);
        goto fail;
    }

    a->rate     = cap_rate;
    a->period   = cap_period;
    a->buf_size = cap_buf;

    /* Link capture and playback for synchronous start */
    if ((err = snd_pcm_link(a->cap, a->play)) < 0) {
        fprintf(stderr, "WARNING: snd_pcm_link failed: %s (continuing unlinked)\n",
                snd_strerror(err));
        a->linked = 0;
    } else {
        a->linked = 1;
    }

    return 0;

fail:
    snd_pcm_close(a->play);
    snd_pcm_close(a->cap);
    return -1;
}

static int alsa_init_capture_only(alsa_ctx_t *a, const char *dev,
                                  unsigned int req_rate,
                                  snd_pcm_uframes_t req_period,
                                  snd_pcm_uframes_t req_buf)
{
    int err;
    memset(a, 0, sizeof(*a));

    if ((err = snd_pcm_open(&a->cap, dev, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        fprintf(stderr, "capture open: %s\n", snd_strerror(err));
        return err;
    }

    unsigned int cap_rate = req_rate;
    snd_pcm_uframes_t cap_period = req_period;
    snd_pcm_uframes_t cap_buf = req_buf;

    if (set_hw_params(a->cap, &cap_rate, &cap_period, &cap_buf) < 0)
        goto fail;
    if (set_sw_params(a->cap, cap_period, cap_buf, cap_period) < 0)
        goto fail;

    a->rate = cap_rate;
    a->period = cap_period;
    a->buf_size = cap_buf;
    a->linked = 0;
    return 0;

fail:
    snd_pcm_close(a->cap);
    return -1;
}

static void alsa_print(alsa_ctx_t *a)
{
    fprintf(stderr, "ALSA: rate=%u  period=%lu  buffer=%lu  linked=%d\n",
            a->rate, (unsigned long)a->period, (unsigned long)a->buf_size,
            a->linked);
}

static void alsa_close(alsa_ctx_t *a)
{
    if (a->linked)
        snd_pcm_unlink(a->cap);
    if (a->play) { snd_pcm_drop(a->play); snd_pcm_close(a->play); }
    if (a->cap)  { snd_pcm_drop(a->cap);  snd_pcm_close(a->cap);  }
    fprintf(stderr, "ALSA closed.\n");
}

#endif /* ANC_IO_H */
