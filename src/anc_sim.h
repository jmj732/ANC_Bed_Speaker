#ifndef ANC_SIM_H
#define ANC_SIM_H

#include "anc_defs.h"

double simulate_one_freq(const float *s_hat, int s_len, float freq_hz,
                          float mu, float leakage, float mu_n_max,
                          float eps_abs, float eps_rel,
                          float band_lo_hz, float band_hi_hz,
                          int mb_bands, float output_limit);
int run_sim_sweep(float tone_hz,
                   float band_lo_hz, float band_hi_hz, int mb_bands,
                   float output_limit);

#endif
