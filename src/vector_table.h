#ifndef VECTOR_TABLE_H
#define VECTOR_TABLE_H

#include "types.h"

extern u32 _estack;

extern void reset_handler(void);

// optional handlers
extern void tim1_cc_handler(void) __attribute__((weak));
extern void dma1_channel3_handler(void) __attribute__((weak));
extern void tim2_handler(void) __attribute__((weak));

#endif
