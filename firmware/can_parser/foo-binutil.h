#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "dbccodeconf.h"

#include "foo.h"

// This version definition comes from main driver version and
// can be compared in user code space for strict compatibility test
#define VER_FOO_MAJ (0U)
#define VER_FOO_MIN (0U)

typedef struct
{
  INIT_FB_t INIT_FB;
  MOTOR_OUT_ELEC_t MOTOR_OUT_ELEC;
  MOTOR_OUT_MEC_t MOTOR_OUT_MEC;
  MOTOR_ANGLE_SP_t MOTOR_ANGLE_SP;
} foo_rx_t;

// There is no any TX mapped massage.

uint32_t foo_Receive(foo_rx_t* m, const uint8_t* d, uint32_t msgid, uint8_t dlc);

#ifdef __DEF_FOO__

extern foo_rx_t foo_rx;

#endif // __DEF_FOO__

#ifdef __cplusplus
}
#endif
