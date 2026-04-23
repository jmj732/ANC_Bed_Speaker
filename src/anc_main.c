#include "anc_app.h"

int main(int argc, char **argv)
{
    anc_runtime_cfg_t rt_cfg;
    anc_runtime_cfg_init_default(&rt_cfg);
    return anc_app_main(argc, argv, &rt_cfg);
}
