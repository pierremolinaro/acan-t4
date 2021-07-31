//--------------------------------------------------------------------------------------------------
// A Teensy 4.x CAN driver
// by Pierre Molinaro
// https://github.com/pierremolinaro/ACAN_T4
//
//--------------------------------------------------------------------------------------------------

#include <ACAN_T4_T4FD_rootCANClock.h>

//--------------------------------------------------------------------------------------------------
//    DEFAULT CAN CLOCK
//--------------------------------------------------------------------------------------------------

static const ACAN_CAN_ROOT_CLOCK kDefaultCANClock = ACAN_CAN_ROOT_CLOCK::CLOCK_60MHz ;
static const uint32_t kDefaultCANClockDivisor = 1 ; // 1 ... 64

//--------------------------------------------------------------------------------------------------

static ACAN_CAN_ROOT_CLOCK gCANClock = kDefaultCANClock ;
static uint32_t gCANClockDivisor = kDefaultCANClockDivisor ;

//--------------------------------------------------------------------------------------------------
//    SET  CAN CLOCK
//--------------------------------------------------------------------------------------------------

bool setCANRootClock (const ACAN_CAN_ROOT_CLOCK inCANClock,
                      const uint32_t inCANClockDivisor) {
  const bool ok = (inCANClockDivisor >= 1) && (inCANClockDivisor <= 64) ;
  if (ok) {
    gCANClock = inCANClock ;
    gCANClockDivisor = inCANClockDivisor ;
  }
  return ok ;
}

//--------------------------------------------------------------------------------------------------
//    GET  CAN CLOCK
//--------------------------------------------------------------------------------------------------

ACAN_CAN_ROOT_CLOCK getCANRootClock (void) {
  return gCANClock ;
}

//--------------------------------------------------------------------------------------------------

uint32_t getCANRootClockFrequency (void) { // 24 000 000, 60 000 000, 80 000 000
  uint32_t frequency = 24 * 1000 * 1000 ;
  switch (gCANClock) {
  case ACAN_CAN_ROOT_CLOCK::CLOCK_24MHz :
    break ;
  case ACAN_CAN_ROOT_CLOCK::CLOCK_60MHz :
    frequency = 60 * 1000 * 1000 ;
    break ;
//   case ACAN_CAN_ROOT_CLOCK::CLOCK_80MHz :
//     frequency = 80 * 1000 * 1000 ;
//     break ;
  }
  return frequency ;
}

//--------------------------------------------------------------------------------------------------

uint32_t getCANRootClockDivisor (void) {
  return gCANClockDivisor ;
}

//--------------------------------------------------------------------------------------------------
