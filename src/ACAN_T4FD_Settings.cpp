//--------------------------------------------------------------------------------------------------
// A Teensy 4.x CAN driver
// by Pierre Molinaro
// https://github.com/pierremolinaro/ACAN_T4
//
//--------------------------------------------------------------------------------------------------

#include <ACAN_T4FD_Settings.h>
#include <ACAN_T4_T4FD_rootCANClock.h>

//--------------------------------------------------------------------------------------------------

static uint32_t min (const uint32_t inA, const uint32_t inB) {
  return (inA < inB) ? inA : inB ;
}

//--------------------------------------------------------------------------------------------------
//    CONSTRUCTOR FOR CANFD
//--------------------------------------------------------------------------------------------------

ACAN_T4FD_Settings::ACAN_T4FD_Settings (const uint32_t inWhishedArbitrationBitRate,
                                        const DataBitRateFactor inDataBitRateFactor,
                                        const uint32_t inTolerancePPM) :
mWhishedArbitrationBitRate (inWhishedArbitrationBitRate) {
  const uint32_t dataBitRate = inWhishedArbitrationBitRate * uint32_t (inDataBitRateFactor) ;
  const uint32_t CANClockFrequency = getCANRootClockFrequency () ;
  const uint32_t CANClockDivisor = getCANRootClockDivisor () ;
  uint32_t dataTQCount = min (48, 129 / uint32_t (inDataBitRateFactor)) ; // TQCount: 5 ... 48
  uint32_t smallestError = UINT32_MAX ;
  uint32_t bestBRP = 1024 ; // Setting for slowest bitrate
  uint32_t bestDataTQCount = dataTQCount ; // Setting for slowest bitrate
  uint32_t BRP = CANClockFrequency / (CANClockDivisor * dataBitRate * dataTQCount) ;
//--- Loop for finding best BRP and best TQCount
  while ((dataTQCount >= 5) && (BRP <= 1024)) {
  //--- Compute error using BRP (caution: BRP should be > 0)
    if (BRP > 0) {
      const uint32_t error = CANClockFrequency - dataBitRate * dataTQCount * BRP * CANClockDivisor ; // error is always >= 0
      if (error < smallestError) {
        smallestError = error ;
        bestBRP = BRP ;
        bestDataTQCount = dataTQCount ;
      }
    }
  //--- Compute error using BRP+1 (caution: BRP+1 should be <= 256)
    if (BRP < 256) {
      const uint32_t error = CANClockDivisor * dataBitRate * dataTQCount * (BRP + 1) - CANClockFrequency ; // error is always >= 0
      if (error < smallestError) {
        smallestError = error ;
        bestBRP = BRP + 1 ;
        bestDataTQCount = dataTQCount ;
      }
    }
  //--- Continue with next value of TQCount
    dataTQCount -= 1 ;
    BRP = CANClockFrequency / (CANClockDivisor * dataBitRate * dataTQCount) ;
  }
//-------------------------- Set the BRP
  mBitRatePrescaler = (uint16_t) bestBRP ;
//-------------------------- Set Data segment lengthes
//--- Compute PS2
  const uint32_t dataPS2 = 2 + bestDataTQCount / 7 ; // Always 2 <= PS2 <= 8
  mDataPhaseSegment2 = uint8_t (dataPS2) ;
//--- Compute the remaining number of TQ once PS2 and SyncSeg are removed
  const uint32_t dataPropSegmentPlusPhaseSegment1 = bestDataTQCount - dataPS2 - 1 /* Sync Seg */ ;
//--- Set PS1 to half of remaining TQCount
  const uint32_t dataPS1 = 1 + dataPropSegmentPlusPhaseSegment1 / 5 ; // Always 1 <= PS1 <= 8
  mDataPhaseSegment1 = uint8_t (dataPS1) ;
//--- Set PS to what is left
  mDataPropagationSegment = uint8_t (dataPropSegmentPlusPhaseSegment1 - dataPS1) ; // Always 1 <= PropSeg <= 64
//--- Set RJW to PS2
  mDataRJW = mDataPhaseSegment2 ;
//-------------------------- Set Arbitration segment lengthes
  const uint32_t bestArbitrationTQCount = bestDataTQCount * uint32_t (inDataBitRateFactor) ;
//--- Compute PS2
  const uint32_t arbitrationPS2 = 2 + 4 * (bestArbitrationTQCount - 1) / 17 ; // Always 2 <= PS2 <= 32
  mArbitrationPhaseSegment2 = uint8_t (arbitrationPS2) ;
//--- Compute the remaining number of TQ once PS2 and SyncSeg are removed
  const uint32_t arbitrationPropSegmentPlusPhaseSegment1 = bestArbitrationTQCount - arbitrationPS2 - 1 /* Sync Seg */ ;
//--- Set PS1 to half of remaining TQCount
  const uint32_t arbitrationPS1 = 1 + 32 * (arbitrationPropSegmentPlusPhaseSegment1 - 1) / 96 ; // Always 1 <= PS1 <= 32
  mArbitrationPhaseSegment1 = uint8_t (arbitrationPS1) ;
//--- Set PS to what is left
  mArbitrationPropagationSegment = uint8_t (arbitrationPropSegmentPlusPhaseSegment1 - arbitrationPS1) ; // Always 1 <= PropSeg <= 8
//--- Set RJW to PS2
  mArbitrationRJW = mArbitrationPhaseSegment2 ;
//--- Triple sampling ?
  mTripleSampling = (mWhishedArbitrationBitRate <= 125000) && (mArbitrationPhaseSegment1 >= 2) ;
//--- Final check of the configuration
  const uint32_t W = bestArbitrationTQCount * mWhishedArbitrationBitRate * mBitRatePrescaler * CANClockDivisor ;
  const uint64_t diff = (CANClockFrequency > W) ? (CANClockFrequency - W) : (W - CANClockFrequency) ;
  const uint64_t ppm = uint64_t (1000 * 1000) ;
  mBitSettingOk = (diff * ppm) <= (uint64_t (W) * inTolerancePPM) ;
} ;

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4FD_Settings::actualArbitrationBitRate (void) const {
  const uint32_t CANClockFrequency = getCANRootClockFrequency () ;
  const uint32_t CANClockDivisor = getCANRootClockDivisor () ;
  const uint32_t TQCount = 1 /* Sync Seg */ + mArbitrationPropagationSegment + mArbitrationPhaseSegment1 + mArbitrationPhaseSegment2 ;
  return CANClockFrequency / (CANClockDivisor * mBitRatePrescaler * TQCount) ;
}

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4FD_Settings::actualDataBitRate (void) const {
  const uint32_t CANClockFrequency = getCANRootClockFrequency () ;
  const uint32_t CANClockDivisor = getCANRootClockDivisor () ;
  const uint32_t TQCount = 1 /* Sync Seg */ + mDataPropagationSegment + mDataPhaseSegment1 + mDataPhaseSegment2 ;
  return CANClockFrequency / (CANClockDivisor * mBitRatePrescaler * TQCount) ;
}

//--------------------------------------------------------------------------------------------------

bool ACAN_T4FD_Settings::exactArbitrationBitRate (void) const {
  const uint32_t CANClockFrequency = getCANRootClockFrequency () ;
  const uint32_t CANClockDivisor = getCANRootClockDivisor () ;
  const uint32_t TQCount = 1 /* Sync Seg */ + mArbitrationPropagationSegment + mArbitrationPhaseSegment1 + mArbitrationPhaseSegment2 ;
  return CANClockFrequency == (mBitRatePrescaler * mWhishedArbitrationBitRate * TQCount * CANClockDivisor) ;
}

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4FD_Settings::ppmFromWishedBitRate (void) const {
  const uint32_t CANClockFrequency = getCANRootClockFrequency () ;
  const uint32_t CANClockDivisor = getCANRootClockDivisor () ;
  const uint32_t TQCount = 1 /* Sync Seg */ + mArbitrationPropagationSegment + mArbitrationPhaseSegment1 + mArbitrationPhaseSegment2 ;
  const uint32_t W = TQCount * mWhishedArbitrationBitRate * mBitRatePrescaler * CANClockDivisor ;
  const uint64_t diff = (CANClockFrequency > W) ? (CANClockFrequency - W) : (W - CANClockFrequency) ;
  const uint64_t ppm = uint64_t (1000 * 1000) ;
  return uint32_t ((diff * ppm) / (W * uint64_t (CANClockDivisor))) ;
}

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4FD_Settings::arbitrationSamplePointFromBitStart (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mArbitrationPropagationSegment + mArbitrationPhaseSegment1 + mArbitrationPhaseSegment2 ;
  const uint32_t samplePoint = 1 /* Sync Seg */ + mArbitrationPropagationSegment + mArbitrationPhaseSegment1 - mTripleSampling ;
  const uint32_t partPerCent = 100 ;
  return (samplePoint * partPerCent) / TQCount ;
}

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4FD_Settings::dataSamplePointFromBitStart (void) const {
  const uint32_t TQCount = 1 /* Sync Seg */ + mDataPropagationSegment + mDataPhaseSegment1 + mDataPhaseSegment2 ;
  const uint32_t samplePoint = 1 /* Sync Seg */ + mDataPropagationSegment + mDataPhaseSegment1 - mTripleSampling ;
  const uint32_t partPerCent = 100 ;
  return (samplePoint * partPerCent) / TQCount ;
}

//--------------------------------------------------------------------------------------------------

uint32_t ACAN_T4FD_Settings::CANFDBitSettingConsistency (void) const {
  uint32_t errorCode = 0 ; // Means no error
  if (mBitRatePrescaler == 0) {
    errorCode |= kBitRatePrescalerIsZero ;
  }else if (mBitRatePrescaler > 1024) {
    errorCode |= kBitRatePrescalerIsGreaterThan1024 ;
  }
  if (mArbitrationPropagationSegment == 0) {
    errorCode |= kArbitrationPropagationSegmentIsZero ;
  }else if (mArbitrationPropagationSegment > 64) {
    errorCode |= kArbitrationPropagationSegmentIsGreaterThan64 ;
  }
  if (mArbitrationPhaseSegment1 == 0) {
    errorCode |= kArbitrationPhaseSegment1IsZero ;
  }else if ((mArbitrationPhaseSegment1 == 1) && mTripleSampling) {
    errorCode |= kArbitrationPhaseSegment1Is1AndTripleSampling ;
  }else if (mArbitrationPhaseSegment1 > 32) {
    errorCode |= kArbitrationPhaseSegment1IsGreaterThan32 ;
  }
  if (mArbitrationPhaseSegment2 < 2) {
    errorCode |= kArbitrationPhaseSegment2IsLowerThan2 ;
  }else if (mArbitrationPhaseSegment2 > 32) {
    errorCode |= kArbitrationPhaseSegment2IsGreaterThan32 ;
  }
  if (mArbitrationRJW == 0) {
    errorCode |= kArbitrationRJWIsZero ;
  }else if (mArbitrationRJW > 32) {
    errorCode |= kArbitrationRJWIsGreaterThan32 ;
  }
  if (mArbitrationRJW > mArbitrationPhaseSegment2) {
    errorCode |= kArbitrationRJWIsGreaterThanPhaseSegment2 ;
  }


  if (mDataPropagationSegment == 0) {
    errorCode |= kDataPropagationSegmentIsZero ;
  }else if (mDataPropagationSegment > 32) {
    errorCode |= kDataPropagationSegmentIsGreaterThan32 ;
  }
  if (mDataPhaseSegment1 == 0) {
    errorCode |= kDataPhaseSegment1IsZero ;
  }else if (mDataPhaseSegment1 > 8) {
    errorCode |= kDataPhaseSegment1IsGreaterThan8 ;
  }
  if (mDataPhaseSegment2 < 2) {
    errorCode |= kDataPhaseSegment2IsLowerThan2 ;
  }else if (mDataPhaseSegment2 > 8) {
    errorCode |= kDataPhaseSegment2IsGreaterThan8 ;
  }
  if (mDataRJW == 0) {
    errorCode |= kDataRJWIsZero ;
  }else if (mDataRJW > 8) {
    errorCode |= kDataRJWIsGreaterThan8 ;
  }
  if (mDataRJW > mDataPhaseSegment2) {
    errorCode |= kDataRJWIsGreaterThanPhaseSegment2 ;
  }
  return errorCode ;
}

//--------------------------------------------------------------------------------------------------

uint32_t MBCount (const ACAN_T4FD_Settings::Payload inPayload) {
  const uint32_t array [4] = {
    64, //  PAYLOAD_8_BYTES -> 64 MB, table 45-27 page 2710
    42, // PAYLOAD_16_BYTES -> 42 MB, table 45-27 page 2710
    24, // PAYLOAD_32_BYTES -> 24 MB, table 45-27 page 2710
    14  // PAYLOAD_64_BYTES -> 14 MB, table 45-27 page 2710
  } ;
  return array [uint32_t (inPayload)] ;
//   uint32_t result = 0 ;
//   switch (inPayload) {
//    case ACAN_T4FD_Settings::PAYLOAD_8_BYTES : // 64 MB, table 44-40 page 2837
//      result = 64 ;
//      break ;
//   case ACAN_T4FD_Settings::PAYLOAD_16_BYTES : // 42 MB, table 44-41 page 2839
//     result = 42 ;
//     break ;
//   case ACAN_T4FD_Settings::PAYLOAD_32_BYTES : // 24 MB, table 44-42 page 2840
//     result = 24 ;
//     break ;
//   case ACAN_T4FD_Settings::PAYLOAD_64_BYTES :  // 14 MB, table 44-43 page 2841
//     result = 14 ;
//     break ;
//   }
//   return result ;
}

//--------------------------------------------------------------------------------------------------
