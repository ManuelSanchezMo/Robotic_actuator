#include "foo.h"


#ifdef FOO_USE_DIAG_MONITORS
// Function prototypes to be called each time CAN frame is unpacked
// FMon function may detect RC, CRC or DLC violation
#include "foo-fmon.h"

#endif // FOO_USE_DIAG_MONITORS


uint32_t Unpack_INIT_FB_foo(INIT_FB_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->P_control_ro = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->P_control_phys = (sigfloat_t)(FOO_P_control_ro_fromS(_m->P_control_ro));
#endif // FOO_USE_SIGFLOAT

  _m->I_control_ro = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->I_control_phys = (sigfloat_t)(FOO_I_control_ro_fromS(_m->I_control_ro));
#endif // FOO_USE_SIGFLOAT

  _m->D_control_ro = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->D_control_phys = (sigfloat_t)(FOO_D_control_ro_fromS(_m->D_control_ro));
#endif // FOO_USE_SIGFLOAT

  _m->Volt_limit_ro = ((_d[7] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->Volt_limit_phys = (sigfloat_t)(FOO_Volt_limit_ro_fromS(_m->Volt_limit_ro));
#endif // FOO_USE_SIGFLOAT

#ifdef FOO_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < INIT_FB_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_INIT_FB_foo(&_m->mon1, INIT_FB_CANID);
#endif // FOO_USE_DIAG_MONITORS

  return INIT_FB_CANID;
}

#ifdef FOO_USE_CANSTRUCT

uint32_t Pack_INIT_FB_foo(INIT_FB_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < INIT_FB_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef FOO_USE_SIGFLOAT
  _m->P_control_ro = FOO_P_control_ro_toS(_m->P_control_phys);
  _m->I_control_ro = FOO_I_control_ro_toS(_m->I_control_phys);
  _m->D_control_ro = FOO_D_control_ro_toS(_m->D_control_phys);
  _m->Volt_limit_ro = FOO_Volt_limit_ro_toS(_m->Volt_limit_phys);
#endif // FOO_USE_SIGFLOAT

  cframe->Data[0] |= (_m->P_control_ro & (0xFFU));
  cframe->Data[1] |= ((_m->P_control_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->I_control_ro & (0xFFU));
  cframe->Data[3] |= ((_m->I_control_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->D_control_ro & (0xFFU));
  cframe->Data[5] |= ((_m->D_control_ro >> 8) & (0xFFU));
  cframe->Data[6] |= (_m->Volt_limit_ro & (0xFFU));
  cframe->Data[7] |= ((_m->Volt_limit_ro >> 8) & (0xFFU));

  cframe->MsgId = INIT_FB_CANID;
  cframe->DLC = INIT_FB_DLC;
  cframe->IDE = INIT_FB_IDE;
  return INIT_FB_CANID;
}

#else

uint32_t Pack_INIT_FB_foo(INIT_FB_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < INIT_FB_DLC) && (i < 8); _d[i++] = 0);

#ifdef FOO_USE_SIGFLOAT
  _m->P_control_ro = FOO_P_control_ro_toS(_m->P_control_phys);
  _m->I_control_ro = FOO_I_control_ro_toS(_m->I_control_phys);
  _m->D_control_ro = FOO_D_control_ro_toS(_m->D_control_phys);
  _m->Volt_limit_ro = FOO_Volt_limit_ro_toS(_m->Volt_limit_phys);
#endif // FOO_USE_SIGFLOAT

  _d[0] |= (_m->P_control_ro & (0xFFU));
  _d[1] |= ((_m->P_control_ro >> 8) & (0xFFU));
  _d[2] |= (_m->I_control_ro & (0xFFU));
  _d[3] |= ((_m->I_control_ro >> 8) & (0xFFU));
  _d[4] |= (_m->D_control_ro & (0xFFU));
  _d[5] |= ((_m->D_control_ro >> 8) & (0xFFU));
  _d[6] |= (_m->Volt_limit_ro & (0xFFU));
  _d[7] |= ((_m->Volt_limit_ro >> 8) & (0xFFU));

  *_len = INIT_FB_DLC;
  *_ide = INIT_FB_IDE;
  return INIT_FB_CANID;
}

#endif // FOO_USE_CANSTRUCT

uint32_t Unpack_MOTOR_OUT_ELEC_foo(MOTOR_OUT_ELEC_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->Ua_ro = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->Ua_phys = (sigfloat_t)(FOO_Ua_ro_fromS(_m->Ua_ro));
#endif // FOO_USE_SIGFLOAT

  _m->Ub_ro = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->Ub_phys = (sigfloat_t)(FOO_Ub_ro_fromS(_m->Ub_ro));
#endif // FOO_USE_SIGFLOAT

  _m->current_ro = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->current_phys = (sigfloat_t)(FOO_current_ro_fromS(_m->current_ro));
#endif // FOO_USE_SIGFLOAT

  _m->electrical_angle_ro = ((_d[7] & (0xFFU)) << 8) | (_d[6] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->electrical_angle_phys = (sigfloat_t)(FOO_electrical_angle_ro_fromS(_m->electrical_angle_ro));
#endif // FOO_USE_SIGFLOAT

#ifdef FOO_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MOTOR_OUT_ELEC_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MOTOR_OUT_ELEC_foo(&_m->mon1, MOTOR_OUT_ELEC_CANID);
#endif // FOO_USE_DIAG_MONITORS

  return MOTOR_OUT_ELEC_CANID;
}

#ifdef FOO_USE_CANSTRUCT

uint32_t Pack_MOTOR_OUT_ELEC_foo(MOTOR_OUT_ELEC_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < MOTOR_OUT_ELEC_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef FOO_USE_SIGFLOAT
  _m->Ua_ro = FOO_Ua_ro_toS(_m->Ua_phys);
  _m->Ub_ro = FOO_Ub_ro_toS(_m->Ub_phys);
  _m->current_ro = FOO_current_ro_toS(_m->current_phys);
  _m->electrical_angle_ro = FOO_electrical_angle_ro_toS(_m->electrical_angle_phys);
#endif // FOO_USE_SIGFLOAT

  cframe->Data[0] |= (_m->Ua_ro & (0xFFU));
  cframe->Data[1] |= ((_m->Ua_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->Ub_ro & (0xFFU));
  cframe->Data[3] |= ((_m->Ub_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->current_ro & (0xFFU));
  cframe->Data[5] |= ((_m->current_ro >> 8) & (0xFFU));
  cframe->Data[6] |= (_m->electrical_angle_ro & (0xFFU));
  cframe->Data[7] |= ((_m->electrical_angle_ro >> 8) & (0xFFU));

  cframe->MsgId = MOTOR_OUT_ELEC_CANID;
  cframe->DLC = MOTOR_OUT_ELEC_DLC;
  cframe->IDE = MOTOR_OUT_ELEC_IDE;
  return MOTOR_OUT_ELEC_CANID;
}

#else

uint32_t Pack_MOTOR_OUT_ELEC_foo(MOTOR_OUT_ELEC_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < MOTOR_OUT_ELEC_DLC) && (i < 8); _d[i++] = 0);

#ifdef FOO_USE_SIGFLOAT
  _m->Ua_ro = FOO_Ua_ro_toS(_m->Ua_phys);
  _m->Ub_ro = FOO_Ub_ro_toS(_m->Ub_phys);
  _m->current_ro = FOO_current_ro_toS(_m->current_phys);
  _m->electrical_angle_ro = FOO_electrical_angle_ro_toS(_m->electrical_angle_phys);
#endif // FOO_USE_SIGFLOAT

  _d[0] |= (_m->Ua_ro & (0xFFU));
  _d[1] |= ((_m->Ua_ro >> 8) & (0xFFU));
  _d[2] |= (_m->Ub_ro & (0xFFU));
  _d[3] |= ((_m->Ub_ro >> 8) & (0xFFU));
  _d[4] |= (_m->current_ro & (0xFFU));
  _d[5] |= ((_m->current_ro >> 8) & (0xFFU));
  _d[6] |= (_m->electrical_angle_ro & (0xFFU));
  _d[7] |= ((_m->electrical_angle_ro >> 8) & (0xFFU));

  *_len = MOTOR_OUT_ELEC_DLC;
  *_ide = MOTOR_OUT_ELEC_IDE;
  return MOTOR_OUT_ELEC_CANID;
}

#endif // FOO_USE_CANSTRUCT

uint32_t Unpack_MOTOR_OUT_MEC_foo(MOTOR_OUT_MEC_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->shaft_angle_ro = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->shaft_angle_phys = (sigfloat_t)(FOO_shaft_angle_ro_fromS(_m->shaft_angle_ro));
#endif // FOO_USE_SIGFLOAT

  _m->shaft_angle_sp_ro = ((_d[3] & (0xFFU)) << 8) | (_d[2] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->shaft_angle_sp_phys = (sigfloat_t)(FOO_shaft_angle_sp_ro_fromS(_m->shaft_angle_sp_ro));
#endif // FOO_USE_SIGFLOAT

  _m->shaft_velocity_ro = ((_d[5] & (0xFFU)) << 8) | (_d[4] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->shaft_velocity_phys = (sigfloat_t)(FOO_shaft_velocity_ro_fromS(_m->shaft_velocity_ro));
#endif // FOO_USE_SIGFLOAT

#ifdef FOO_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MOTOR_OUT_MEC_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MOTOR_OUT_MEC_foo(&_m->mon1, MOTOR_OUT_MEC_CANID);
#endif // FOO_USE_DIAG_MONITORS

  return MOTOR_OUT_MEC_CANID;
}

#ifdef FOO_USE_CANSTRUCT

uint32_t Pack_MOTOR_OUT_MEC_foo(MOTOR_OUT_MEC_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < MOTOR_OUT_MEC_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef FOO_USE_SIGFLOAT
  _m->shaft_angle_ro = FOO_shaft_angle_ro_toS(_m->shaft_angle_phys);
  _m->shaft_angle_sp_ro = FOO_shaft_angle_sp_ro_toS(_m->shaft_angle_sp_phys);
  _m->shaft_velocity_ro = FOO_shaft_velocity_ro_toS(_m->shaft_velocity_phys);
#endif // FOO_USE_SIGFLOAT

  cframe->Data[0] |= (_m->shaft_angle_ro & (0xFFU));
  cframe->Data[1] |= ((_m->shaft_angle_ro >> 8) & (0xFFU));
  cframe->Data[2] |= (_m->shaft_angle_sp_ro & (0xFFU));
  cframe->Data[3] |= ((_m->shaft_angle_sp_ro >> 8) & (0xFFU));
  cframe->Data[4] |= (_m->shaft_velocity_ro & (0xFFU));
  cframe->Data[5] |= ((_m->shaft_velocity_ro >> 8) & (0xFFU));

  cframe->MsgId = MOTOR_OUT_MEC_CANID;
  cframe->DLC = MOTOR_OUT_MEC_DLC;
  cframe->IDE = MOTOR_OUT_MEC_IDE;
  return MOTOR_OUT_MEC_CANID;
}

#else

uint32_t Pack_MOTOR_OUT_MEC_foo(MOTOR_OUT_MEC_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < MOTOR_OUT_MEC_DLC) && (i < 8); _d[i++] = 0);

#ifdef FOO_USE_SIGFLOAT
  _m->shaft_angle_ro = FOO_shaft_angle_ro_toS(_m->shaft_angle_phys);
  _m->shaft_angle_sp_ro = FOO_shaft_angle_sp_ro_toS(_m->shaft_angle_sp_phys);
  _m->shaft_velocity_ro = FOO_shaft_velocity_ro_toS(_m->shaft_velocity_phys);
#endif // FOO_USE_SIGFLOAT

  _d[0] |= (_m->shaft_angle_ro & (0xFFU));
  _d[1] |= ((_m->shaft_angle_ro >> 8) & (0xFFU));
  _d[2] |= (_m->shaft_angle_sp_ro & (0xFFU));
  _d[3] |= ((_m->shaft_angle_sp_ro >> 8) & (0xFFU));
  _d[4] |= (_m->shaft_velocity_ro & (0xFFU));
  _d[5] |= ((_m->shaft_velocity_ro >> 8) & (0xFFU));

  *_len = MOTOR_OUT_MEC_DLC;
  *_ide = MOTOR_OUT_MEC_IDE;
  return MOTOR_OUT_MEC_CANID;
}

#endif // FOO_USE_CANSTRUCT

uint32_t Unpack_MOTOR_ANGLE_SP_foo(MOTOR_ANGLE_SP_t* _m, const uint8_t* _d, uint8_t dlc_)
{
  (void)dlc_;
  _m->angle_sp_ro = ((_d[1] & (0xFFU)) << 8) | (_d[0] & (0xFFU));
#ifdef FOO_USE_SIGFLOAT
  _m->angle_sp_phys = (sigfloat_t)(FOO_angle_sp_ro_fromS(_m->angle_sp_ro));
#endif // FOO_USE_SIGFLOAT

#ifdef FOO_USE_DIAG_MONITORS
  _m->mon1.dlc_error = (dlc_ < MOTOR_ANGLE_SP_DLC);
  _m->mon1.last_cycle = GetSystemTick();
  _m->mon1.frame_cnt++;

  FMon_MOTOR_ANGLE_SP_foo(&_m->mon1, MOTOR_ANGLE_SP_CANID);
#endif // FOO_USE_DIAG_MONITORS

  return MOTOR_ANGLE_SP_CANID;
}

#ifdef FOO_USE_CANSTRUCT

uint32_t Pack_MOTOR_ANGLE_SP_foo(MOTOR_ANGLE_SP_t* _m, __CoderDbcCanFrame_t__* cframe)
{
  uint8_t i; for (i = 0; (i < MOTOR_ANGLE_SP_DLC) && (i < 8); cframe->Data[i++] = 0);

#ifdef FOO_USE_SIGFLOAT
  _m->angle_sp_ro = FOO_angle_sp_ro_toS(_m->angle_sp_phys);
#endif // FOO_USE_SIGFLOAT

  cframe->Data[0] |= (_m->angle_sp_ro & (0xFFU));
  cframe->Data[1] |= ((_m->angle_sp_ro >> 8) & (0xFFU));

  cframe->MsgId = MOTOR_ANGLE_SP_CANID;
  cframe->DLC = MOTOR_ANGLE_SP_DLC;
  cframe->IDE = MOTOR_ANGLE_SP_IDE;
  return MOTOR_ANGLE_SP_CANID;
}

#else

uint32_t Pack_MOTOR_ANGLE_SP_foo(MOTOR_ANGLE_SP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide)
{
  uint8_t i; for (i = 0; (i < MOTOR_ANGLE_SP_DLC) && (i < 8); _d[i++] = 0);

#ifdef FOO_USE_SIGFLOAT
  _m->angle_sp_ro = FOO_angle_sp_ro_toS(_m->angle_sp_phys);
#endif // FOO_USE_SIGFLOAT

  _d[0] |= (_m->angle_sp_ro & (0xFFU));
  _d[1] |= ((_m->angle_sp_ro >> 8) & (0xFFU));

  *_len = MOTOR_ANGLE_SP_DLC;
  *_ide = MOTOR_ANGLE_SP_IDE;
  return MOTOR_ANGLE_SP_CANID;
}

#endif // FOO_USE_CANSTRUCT

