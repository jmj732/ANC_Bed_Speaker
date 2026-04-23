#ifndef ANC_NB_H
#define ANC_NB_H

#include "anc_defs.h"
#include "anc_io.h"
#include "anc_log.h"

void run_nb_anc(alsa_ctx_t *a, const float *sec_path, int sec_len,
                logger_t *l, int n_harm, float nb_mu, float nb_leak,
                int start_fill_periods, int xrun_fill_periods,
                const anc_runtime_cfg_t *cfg);

#endif
