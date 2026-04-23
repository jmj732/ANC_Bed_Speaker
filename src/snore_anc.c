/*
 * snore_anc.c
 *
 * Snore-oriented ANC application:
 * - wider low-frequency control band (60~250Hz)
 * - longer adaptive filter
 * - more conservative update gate
 * - dedicated secondary-path filename with fallback
 */

/* ===== 1. Override defaults BEFORE including headers ===== */
#define W_LEN_DEFAULT       1536
#define S_LEN_DEFAULT       1024

#define MU_DEFAULT          0.008f
#define MU_FLOOR            0.001f
#define MU_CEIL             0.008f
#define EPSILON_REL         0.015f
#define LEAKAGE_DEFAULT     0.999999f
#define ERR_UPDATE_MAX      0.015f
#define GRAD_CLIP           0.15f

#define MEASURE_SECS        45
#define SEC_PATH_FILE       "snore_sec_path.bin"
#define SEC_PATH_FALLBACK   "sec_path.bin"

#define BASELINE_SECS       4
#define STARTUP_WARMUP_SECS 2
#define TARGET_BAND_LO_HZ   60.0f
#define TARGET_BAND_HI_HZ   250.0f
#define TARGET_REDUCTION_DB 8.0
#define TUNE_DOWN_RATE      0.75f
#define RECOVER_SECS        3

/* ===== 2. Now include app header ===== */
#include "anc_app.h"

int main(int argc, char **argv)
{
    anc_runtime_cfg_t rt_cfg;
    anc_runtime_cfg_init_default(&rt_cfg);
    return anc_app_main(argc, argv, &rt_cfg);
}
