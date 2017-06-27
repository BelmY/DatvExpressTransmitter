#include "express.h"

#ifdef ENABLE_LIMESDR

int limesdr_write_16_bit_samples(scmplx *s, int len);

int limesdr_init();

void limesdr_set_freq(double freq);

void limesdr_deinit(void);

void limesdr_set_level(int level);

int limesdr_set_sr(double sr);

void limesdr_run(void);

void limesdr_pause(void);


#endif