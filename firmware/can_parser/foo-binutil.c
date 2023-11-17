#include "foo-binutil.h"

#ifdef __DEF_FOO__

foo_rx_t foo_rx;

#endif // __DEF_FOO__

uint32_t foo_Receive(foo_rx_t* _m, const uint8_t* _d, uint32_t _id, uint8_t dlc_)
{
 uint32_t recid = 0;
 if ((_id >= 0x1U) && (_id < 0x3U)) {
  if (_id == 0x1U) {
   recid = Unpack_INIT_FB_foo(&(_m->INIT_FB), _d, dlc_);
  } else if (_id == 0x2U) {
   recid = Unpack_MOTOR_OUT_ELEC_foo(&(_m->MOTOR_OUT_ELEC), _d, dlc_);
  }
 } else {
  if (_id == 0x3U) {
   recid = Unpack_MOTOR_OUT_MEC_foo(&(_m->MOTOR_OUT_MEC), _d, dlc_);
  } else if (_id == 0x7U) {
   recid = Unpack_MOTOR_ANGLE_SP_foo(&(_m->MOTOR_ANGLE_SP), _d, dlc_);
  }
 }

 return recid;
}

