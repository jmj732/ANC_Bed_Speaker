#ifndef ANC_PASS_H
#define ANC_PASS_H

#include "anc_defs.h"
#include "anc_io.h"
#include "anc_log.h"

void run_passthrough(alsa_ctx_t *a, logger_t *l,
                     int start_fill_periods, int xrun_fill_periods);

#endif
