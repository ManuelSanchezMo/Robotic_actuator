#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// DBC file version
#define VER_FOO_MAJ (0U)
#define VER_FOO_MIN (0U)

// include current dbc-driver compilation config
#include "foo-config.h"

#ifdef FOO_USE_DIAG_MONITORS
// This file must define:
// base monitor struct
// function signature for HASH calculation: (@GetFrameHash)
// function signature for getting system tick value: (@GetSystemTick)
#include "canmonitorutil.h"

#endif // FOO_USE_DIAG_MONITORS


// Driver controller set point
// def @INIT_FB CAN Message (1    0x1)
#define INIT_FB_IDE (0U)
#define INIT_FB_DLC (8U)
#define INIT_FB_CANID (0x1)
// signal: @P_control_ro
#define FOO_P_control_ro_CovFactor (0.125000)
#define FOO_P_control_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.125000)) )
#define FOO_P_control_ro_fromS(x) ( (((x) * (0.125000)) + (0.000000)) )
// signal: @I_control_ro
#define FOO_I_control_ro_CovFactor (0.125000)
#define FOO_I_control_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.125000)) )
#define FOO_I_control_ro_fromS(x) ( (((x) * (0.125000)) + (0.000000)) )
// signal: @D_control_ro
#define FOO_D_control_ro_CovFactor (0.125000)
#define FOO_D_control_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.125000)) )
#define FOO_D_control_ro_fromS(x) ( (((x) * (0.125000)) + (0.000000)) )
// signal: @Volt_limit_ro
#define FOO_Volt_limit_ro_CovFactor (0.125000)
#define FOO_Volt_limit_ro_toS(x) ( (int16_t) (((x) - (-4095.000000)) / (0.125000)) )
#define FOO_Volt_limit_ro_fromS(x) ( (((x) * (0.125000)) + (-4095.000000)) )

typedef struct
{
#ifdef FOO_USE_BITS_SIGNAL

  int16_t P_control_ro;                      //  [-] Bits=16 Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t P_control_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t I_control_ro;                      //  [-] Bits=16 Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t I_control_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t D_control_ro;                      //  [-] Bits=16 Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t D_control_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t Volt_limit_ro;                     //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t Volt_limit_phys;
#endif // FOO_USE_SIGFLOAT

#else

  int16_t P_control_ro;                      //  [-] Bits=16 Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t P_control_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t I_control_ro;                      //  [-] Bits=16 Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t I_control_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t D_control_ro;                      //  [-] Bits=16 Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t D_control_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t Volt_limit_ro;                     //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t Volt_limit_phys;
#endif // FOO_USE_SIGFLOAT

#endif // FOO_USE_BITS_SIGNAL

#ifdef FOO_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // FOO_USE_DIAG_MONITORS

} INIT_FB_t;

// Steering controller set point
// def @MOTOR_OUT_ELEC CAN Message (2    0x2)
#define MOTOR_OUT_ELEC_IDE (0U)
#define MOTOR_OUT_ELEC_DLC (8U)
#define MOTOR_OUT_ELEC_CANID (0x2)
// signal: @Ua_ro
#define FOO_Ua_ro_CovFactor (0.125000)
#define FOO_Ua_ro_toS(x) ( (int16_t) (((x) - (-4095.000000)) / (0.125000)) )
#define FOO_Ua_ro_fromS(x) ( (((x) * (0.125000)) + (-4095.000000)) )
// signal: @Ub_ro
#define FOO_Ub_ro_CovFactor (0.125000)
#define FOO_Ub_ro_toS(x) ( (int16_t) (((x) - (-4095.000000)) / (0.125000)) )
#define FOO_Ub_ro_fromS(x) ( (((x) * (0.125000)) + (-4095.000000)) )
// signal: @current_ro
#define FOO_current_ro_CovFactor (0.125000)
#define FOO_current_ro_toS(x) ( (int16_t) (((x) - (-4095.000000)) / (0.125000)) )
#define FOO_current_ro_fromS(x) ( (((x) * (0.125000)) + (-4095.000000)) )
// signal: @electrical_angle_ro
#define FOO_electrical_angle_ro_CovFactor (0.125000)
#define FOO_electrical_angle_ro_toS(x) ( (int16_t) (((x) - (-4095.000000)) / (0.125000)) )
#define FOO_electrical_angle_ro_fromS(x) ( (((x) * (0.125000)) + (-4095.000000)) )

typedef struct
{
#ifdef FOO_USE_BITS_SIGNAL

  int16_t Ua_ro;                             //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t Ua_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t Ub_ro;                             //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t Ub_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t current_ro;                        //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t current_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t electrical_angle_ro;               //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t electrical_angle_phys;
#endif // FOO_USE_SIGFLOAT

#else

  int16_t Ua_ro;                             //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t Ua_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t Ub_ro;                             //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t Ub_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t current_ro;                        //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t current_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t electrical_angle_ro;               //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t electrical_angle_phys;
#endif // FOO_USE_SIGFLOAT

#endif // FOO_USE_BITS_SIGNAL

#ifdef FOO_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // FOO_USE_DIAG_MONITORS

} MOTOR_OUT_ELEC_t;

// Steer controller feedback
// def @MOTOR_OUT_MEC CAN Message (3    0x3)
#define MOTOR_OUT_MEC_IDE (0U)
#define MOTOR_OUT_MEC_DLC (6U)
#define MOTOR_OUT_MEC_CANID (0x3)
// signal: @shaft_angle_ro
#define FOO_shaft_angle_ro_CovFactor (0.125000)
#define FOO_shaft_angle_ro_toS(x) ( (int16_t) (((x) - (-4095.000000)) / (0.125000)) )
#define FOO_shaft_angle_ro_fromS(x) ( (((x) * (0.125000)) + (-4095.000000)) )
// signal: @shaft_angle_sp_ro
#define FOO_shaft_angle_sp_ro_CovFactor (0.125000)
#define FOO_shaft_angle_sp_ro_toS(x) ( (int16_t) (((x) - (-4095.000000)) / (0.125000)) )
#define FOO_shaft_angle_sp_ro_fromS(x) ( (((x) * (0.125000)) + (-4095.000000)) )
// signal: @shaft_velocity_ro
#define FOO_shaft_velocity_ro_CovFactor (0.125000)
#define FOO_shaft_velocity_ro_toS(x) ( (int16_t) (((x) - (-4095.000000)) / (0.125000)) )
#define FOO_shaft_velocity_ro_fromS(x) ( (((x) * (0.125000)) + (-4095.000000)) )

typedef struct
{
#ifdef FOO_USE_BITS_SIGNAL

  int16_t shaft_angle_ro;                    //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t shaft_angle_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t shaft_angle_sp_ro;                 //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t shaft_angle_sp_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t shaft_velocity_ro;                 //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t shaft_velocity_phys;
#endif // FOO_USE_SIGFLOAT

#else

  int16_t shaft_angle_ro;                    //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t shaft_angle_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t shaft_angle_sp_ro;                 //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t shaft_angle_sp_phys;
#endif // FOO_USE_SIGFLOAT

  int16_t shaft_velocity_ro;                 //  [-] Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t shaft_velocity_phys;
#endif // FOO_USE_SIGFLOAT

#endif // FOO_USE_BITS_SIGNAL

#ifdef FOO_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // FOO_USE_DIAG_MONITORS

} MOTOR_OUT_MEC_t;

// Steer controller feedback 2
// def @MOTOR_ANGLE_SP CAN Message (7    0x7)
#define MOTOR_ANGLE_SP_IDE (0U)
#define MOTOR_ANGLE_SP_DLC (2U)
#define MOTOR_ANGLE_SP_CANID (0x7)
// signal: @angle_sp_ro
#define FOO_angle_sp_ro_CovFactor (0.125000)
#define FOO_angle_sp_ro_toS(x) ( (uint16_t) (((x) - (-4095.000000)) / (0.125000)) )
#define FOO_angle_sp_ro_fromS(x) ( (((x) * (0.125000)) + (-4095.000000)) )

typedef struct
{
#ifdef FOO_USE_BITS_SIGNAL

  uint16_t angle_sp_ro;                      //      Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t angle_sp_phys;
#endif // FOO_USE_SIGFLOAT

#else

  uint16_t angle_sp_ro;                      //      Bits=16 Offset= -4095.000000       Factor= 0.125000       

#ifdef FOO_USE_SIGFLOAT
  sigfloat_t angle_sp_phys;
#endif // FOO_USE_SIGFLOAT

#endif // FOO_USE_BITS_SIGNAL

#ifdef FOO_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // FOO_USE_DIAG_MONITORS

} MOTOR_ANGLE_SP_t;

// Function signatures

uint32_t Unpack_INIT_FB_foo(INIT_FB_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef FOO_USE_CANSTRUCT
uint32_t Pack_INIT_FB_foo(INIT_FB_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INIT_FB_foo(INIT_FB_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // FOO_USE_CANSTRUCT

uint32_t Unpack_MOTOR_OUT_ELEC_foo(MOTOR_OUT_ELEC_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef FOO_USE_CANSTRUCT
uint32_t Pack_MOTOR_OUT_ELEC_foo(MOTOR_OUT_ELEC_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MOTOR_OUT_ELEC_foo(MOTOR_OUT_ELEC_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // FOO_USE_CANSTRUCT

uint32_t Unpack_MOTOR_OUT_MEC_foo(MOTOR_OUT_MEC_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef FOO_USE_CANSTRUCT
uint32_t Pack_MOTOR_OUT_MEC_foo(MOTOR_OUT_MEC_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MOTOR_OUT_MEC_foo(MOTOR_OUT_MEC_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // FOO_USE_CANSTRUCT

uint32_t Unpack_MOTOR_ANGLE_SP_foo(MOTOR_ANGLE_SP_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef FOO_USE_CANSTRUCT
uint32_t Pack_MOTOR_ANGLE_SP_foo(MOTOR_ANGLE_SP_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MOTOR_ANGLE_SP_foo(MOTOR_ANGLE_SP_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // FOO_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
