#include "foo-fmon.h"

#ifdef FOO_USE_DIAG_MONITORS

/*
Put the monitor function content here, keep in mind -
next generation will completely clear all manually added code (!)
*/

void FMon_INIT_FB_foo(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void FMon_MOTOR_OUT_ELEC_foo(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void FMon_MOTOR_OUT_MEC_foo(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

void FMon_MOTOR_ANGLE_SP_foo(FrameMonitor_t* _mon, uint32_t msgid)
{
  (void)_mon;
  (void)msgid;
}

#endif // FOO_USE_DIAG_MONITORS
