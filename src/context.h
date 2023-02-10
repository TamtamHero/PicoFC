#ifndef CONTEXT_H
#define CONTEXT_H

#include "stdint.h"

#include "radio/crsf.h"
#include "control/control.h"


struct picoFC_ctx_s {
    uint16_t channels[CRSF_CHAN_COUNT];
    command_t command;
    orientation_t orientation;
};

typedef struct picoFC_ctx_s picoFC_ctx_t;

extern picoFC_ctx_t picoFC_ctx;

#endif