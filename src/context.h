#ifndef CONTEXT_H
#define CONTEXT_H

#include "stdint.h"

#include "radio/crsf.h"

struct picoFC_ctx_s {
    uint16_t channels[CRSF_CHAN_COUNT];
};

typedef struct picoFC_ctx_s picoFC_ctx_t;

extern picoFC_ctx_t picoFC_ctx;

#endif