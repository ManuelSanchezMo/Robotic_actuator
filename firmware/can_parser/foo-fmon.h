#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// DBC file version
#define VER_FOO_MAJ_FMON (0U)
#define VER_FOO_MIN_FMON (0U)

#include "foo-config.h"

#ifdef FOO_USE_DIAG_MONITORS

#include "canmonitorutil.h"
/*
This file contains the prototypes of all the functions that will be called
from each Unpack_*name* function to detect DBC related errors
It is the user responsibility to defined these functions in the
separated .c file. If it won't be done the linkage error will happen
*/

void FMon_INIT_FB_foo(FrameMonitor_t* _mon, uint32_t msgid);
void FMon_MOTOR_OUT_ELEC_foo(FrameMonitor_t* _mon, uint32_t msgid);
void FMon_MOTOR_OUT_MEC_foo(FrameMonitor_t* _mon, uint32_t msgid);
void FMon_MOTOR_ANGLE_SP_foo(FrameMonitor_t* _mon, uint32_t msgid);

#endif // FOO_USE_DIAG_MONITORS

#ifdef __cplusplus
}
#endif
