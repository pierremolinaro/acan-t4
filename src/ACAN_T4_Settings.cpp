//--------------------------------------------------------------------------------------------------
// A Teensy 4.x CAN driver
// by Pierre Molinaro
// https://github.com/pierremolinaro/ACAN_T4
//
//--------------------------------------------------------------------------------------------------

#include <ACAN_T4_Settings.h>

//--------------------------------------------------------------------------------------------------
//    CONSTRUCTOR FOR CAN 2.0B
//--------------------------------------------------------------------------------------------------

ACAN_T4_Settings::ACAN_T4_Settings (const uint32_t inWhishedBitRate,
                                    const uint32_t inTolerancePPM) :
mWhishedBitRate (inWhishedBitRate) {
  const uint32_t CANClockFrequency = getCANRootClockFrequency () ;
  const uint32_t CANClockDivisor = getCANRootClockDivisor () ;
  mWhishedBitRate = inWhishedBitRate ;
  uint32_t TQCount = 25 ; // TQCount: 5 ... 25
  uint32_t smallestError = UINT32_MAX ;
  uint32_t bestBRP = 256 ; // Setting for slowest bitrate
  uint32_t bestTQCount = 25 ; // Setting for slowest bitrate
  uint32_t BRP = CANClockFrequency / (CANClockDivisor * inWhishedBitRate * TQCount) ;
//--- Loop for finding best BRP and best TQCount
  while ((TQCount >= 5) && (BRP <= 256)) {
  //--- Compute error using BRP (caution: BRP should be > 0)
    if (BRP > 0) {
      const uint32_t error = CANClockFrequency - inWhishedBitRate * TQCount * BRP * CANClockDivisor ; // error is always >= 0
      if (error < smallestError) {
        smallestError = error ;
        bestBRP = BRP ;
        bestTQCount = TQCount ;
      }
    }
  //--- Compute error using BRP+1 (caution: BRP+1 should be <= 256)
    if (BRP < 256) {
      const uint32_t error = inWhishedBitRate * TQCount * (BRP + 1) * CANClockDivisor - CANClockFrequency ; // error is always >= 0
      if (error < smallestError) {
        smallestError = error ;
        bestBRP = BRP + 1 ;
        bestTQCount = TQCount ;
      }
    }
  //--- Continue with next value of TQCount
    TQCount -= 1 ;
    BRP = CANClockFrequency / (CANClockDivisor * inWhishedBitRate * TQCount) ;
  }
//--- Set the BRP
  mBitRatePrescaler = uint16_t (bestBRP) ;
//--- Compute PS2
  const uint32_t PS2 = 1 + 2 * bestTQCount / 7 ; // Always 2 <= PS2 <= 8
  mPhaseSegment2 = uint8_t (PS2) ;
//--- Compute the remaining number of TQ once PS2 and SyncSeg are removed
  const uint32_t propSegmentPlusPhaseSegment1 = bestTQCount - PS2 - 1 /* Sync Seg */ ;
//--- Set PS1 to half of remaining TQCount
  const uint32_t PS1 = propSegmentPlusPhaseSegment1 / 2 ; // Always 1 <= PS1 <= 8
  mPhaseSegment1 = uint8_t (PS1) ;
//--- Set PS to what is left
  mPropagationSegment = uint8_t (propSegmentPlusPhaseSegment1 - PS1) ; // Always 1 <= PropSeg <= 8
//--- Set RJW to PS2, with a maximum value of 4
  mRJW = (mPhaseSegment2 >= 4) ? 4 : mPhaseSegment2 ; // Always 2 <= RJW <= 4, and RJW <= mPhaseSegment2
//--- Triple sampling ?
  mTripleSampling = (inWhishedBitRate <= 125000) && (mPhaseSegment1 >= 2) ;
//--- Final check of the configuration
  const uint32_t W = bestTQCount * mWhishedBitRate * mBitRatePrescaler * CANClockDivisor ;
  const uint64_t diff = (CANClockFrequency > W) ? (CANClockFrequency - W) : (W - CANClockFrequency) ;
  const uint64_t ppm = uint64_t (1000 * 1000) ;
  mBitSettingOk = (diff * ppm) <= (uint64_t (W) * inTolerancePPM) ;
} ;

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4_Settings::actualBitRate (void) const {
  const uint32_t CANClockFrequency = getCANRootClockFrequency () ;
  const uint32_t CANClockDivisor = getCANRootClockDivisor () ;
  const uint32_t TQCount = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 + mPhaseSegment2 ;
  return CANClockFrequency / (CANClockDivisor * mBitRatePrescaler * TQCount) ;
}

//--------------------------------------------------------------------------------------------------

bool ACAN_T4_Settings::exactBitRate (void) const {
  const uint32_t CANClockFrequency = getCANRootClockFrequency () ;
  const uint32_t CANClockDivisor = getCANRootClockDivisor () ;
  const uint32_t TQCount = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 + mPhaseSegment2 ;
  return CANClockFrequency == (mBitRatePrescaler * mWhishedBitRate * TQCount * CANClockDivisor) ;
}

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4_Settings::ppmFromWishedBitRate (void) const {
  const uint32_t CANClockFrequency = getCANRootClockFrequency () ;
  const uint32_t CANClockDivisor = getCANRootClockDivisor () ;
  const uint32_t TQCount = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 + mPhaseSegment2 ;
  const uint32_t W = TQCount * mWhishedBitRate * mBitRatePrescaler * CANClockDivisor ;
  const uint64_t diff = (CANClockFrequency > W) ? (CANClockFrequency - W) : (W - CANClockFrequency) ;
  const uint64_t ppm = uint64_t (1000 * 1000) ;
  return uint32_t ((diff * ppm) / (W * uint64_t (CANClockDivisor))) ;
}

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4_Settings::samplePointFromBitStart (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 + mPhaseSegment2 ;
  const uint32_t samplePoint = 1 /* Sync Seg */ + mPropagationSegment + mPhaseSegment1 - mTripleSampling ;
  const uint32_t partPerCent = 100 ;
  return (samplePoint * partPerCent) / TQCount ;
}

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4_Settings::CANBitSettingConsistency (void) const {
  uint32_t errorCode = 0 ; // Means no error
  if (mBitRatePrescaler == 0) {
    errorCode |= kBitRatePrescalerIsZero ;
  }else if (mBitRatePrescaler > 256) {
    errorCode |= kBitRatePrescalerIsGreaterThan256 ;
  }
  if (mPropagationSegment == 0) {
    errorCode |= kPropagationSegmentIsZero ;
  }else if (mPropagationSegment > 8) {
    errorCode |= kPropagationSegmentIsGreaterThan8 ;
  }
  if (mPhaseSegment1 == 0) {
    errorCode |= kPhaseSegment1IsZero ;
  }else if ((mPhaseSegment1 == 1) && mTripleSampling) {
    errorCode |= kPhaseSegment1Is1AndTripleSampling ;
  }else if (mPhaseSegment1 > 8) {
    errorCode |= kPhaseSegment1IsGreaterThan8 ;
  }
  if (mPhaseSegment2 == 0) {
    errorCode |= kPhaseSegment2IsZero ;
  }else if (mPhaseSegment2 > 8) {
    errorCode |= kPhaseSegment2IsGreaterThan8 ;
  }
  if (mRJW == 0) {
    errorCode |= kRJWIsZero ;
  }else if (mRJW > 4) {
    errorCode |= kRJWIsGreaterThan4 ;
  }
  if (mRJW > mPhaseSegment2) {
    errorCode |= kRJWIsGreaterThanPhaseSegment2 ;
  }

  return errorCode ;
}

//--------------------------------------------------------------------------------------------------
