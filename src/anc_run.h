#ifndef ANC_RUN_H
#define ANC_RUN_H

#include "anc_defs.h"
#include "anc_algo.h"
#include "anc_io.h"
#include "anc_log.h"

void run_anc(alsa_ctx_t *a, mbfxlms_t *m, logger_t *l, int no_adapt,
             float band_lo_hz, float band_hi_hz,
             int start_fill_periods, int xrun_fill_periods,
             int adapt_delay_ms,
             const anc_runtime_cfg_t *cfg);

#endif
