//--------------------------------------------------------------------------------------------------
// A Teensy 4.x CAN driver
// by Pierre Molinaro
// https://github.com/pierremolinaro/ACAN_T4
//
//--------------------------------------------------------------------------------------------------

#pragma once

//--------------------------------------------------------------------------------------------------

#include <ACAN_T4_T4FD_rootCANClock.h>

//--------------------------------------------------------------------------------------------------

class ACAN_T4_Settings {

//······················································································································
//   Enumerations
//······················································································································

  public: typedef enum : uint8_t {
    NO_PULLUP_NO_PULLDOWN = 0, // PUS = 0, PUE = 0, PKE = 0
    PULLDOWN_100k = 0b0011, // PUS = 0, PUE = 1, PKE = 1
    PULLUP_47k    = 0b0111, // PUS = 1, PUE = 1, PKE = 1
    PULLUP_100k   = 0b1011, // PUS = 2, PUE = 1, PKE = 1
    PULLUP_22k    = 0b1111  // PUS = 3, PUE = 1, PKE = 1
  } RxPinConfiguration ;

//······················································································································

  public: typedef enum : uint8_t {
    IMPEDANCE_R0 = 1,
    IMPEDANCE_R0_DIVIDED_BY_2 = 2,
    IMPEDANCE_R0_DIVIDED_BY_3 = 3,
    IMPEDANCE_R0_DIVIDED_BY_4 = 4,
    IMPEDANCE_R0_DIVIDED_BY_5 = 5,
    IMPEDANCE_R0_DIVIDED_BY_6 = 6,
    IMPEDANCE_R0_DIVIDED_BY_7 = 7
  } TxPinOutputBufferImpedance ;

//······················································································································
//    Constructor for a given baud rate
//······················································································································

  public: explicit ACAN_T4_Settings (const uint32_t inWhishedBitRate,
                                     const uint32_t inTolerancePPM = 1000) ;

//······················································································································
//    Properties
//······················································································································

//--- CAN 2.0B bit timing
  public: uint32_t mWhishedBitRate ; // In kb/s
  public: uint16_t mBitRatePrescaler = 1 ; // 1...256
  public: uint8_t mPropagationSegment = 1 ; // 1...8
  public: uint8_t mPhaseSegment1 = 1 ; // 1...8
  public: uint8_t mPhaseSegment2 = 1 ;  // 2...8
  public: uint8_t mRJW = 1 ; // 1...4
  public: bool mTripleSampling = false ; // true --> triple sampling, false --> single sampling
  public: bool mBitSettingOk = true ; // The above configuration is correct

//--- CANFD bit timing (default values correspond to 250 kb/s, §44.6.2.19, page 2801)
  public: uint8_t mDataPhasePropagationSegment = 8 ; // 1...64
  public: uint8_t mDataPhasePhaseSegment1 = 8 ; // 1...32
  public: uint8_t mDataPhasePhaseSegment2 = 7 ;  // 2...32
  public: uint8_t mDataPhaseRJW = 4 ; // 1...32

//--- Listen only mode
  public: bool mListenOnlyMode = false ; // true --> listen only mode, cannot send any message, false --> normal mode

//--- Self Reception mode
  public: bool mSelfReceptionMode = false ; // true --> sent frame are also received, false --> are not received

//--- Loop Back mode
  public: bool mLoopBackMode = false ; // true --> loop back mode, false --> no loop back

//--- Tx pin configuration
  public: uint8_t mTxPin = 255 ; // 255 means use default pin
  public: TxPinOutputBufferImpedance mTxPinOutputBufferImpedance = IMPEDANCE_R0_DIVIDED_BY_6 ;
  public: bool mTxPinIsOpenCollector = false ; // false --> totem pole, true --> open collector

//--- Rx pin configuration
  public: uint8_t mRxPin = 255 ; // 255 means use default pin
  public: RxPinConfiguration mRxPinConfiguration = PULLUP_47k ;

//--- Receive buffer size
  public: uint16_t mReceiveBufferSize = 256 ;

//--- Transmit buffer size
  public: uint16_t mTransmitBufferSize = 16 ;

//--- Compute actual bitrate
  public: uint32_t actualBitRate (void) const ;

//--- Exact bitrate ?
  public: bool exactBitRate (void) const ;

//--- Distance between actual bitrate and requested bitrate (in ppm, part-per-million)
  public: uint32_t ppmFromWishedBitRate (void) const ;

//--- Distance of sample point from bit start (in ppc, part-per-cent, denoted by %)
  public: uint32_t samplePointFromBitStart (void) const ;

//--- Bit settings are consistent ? (returns 0 if ok)
  public: uint32_t CANBitSettingConsistency (void) const ;

//--- Constants returned by CANBitSettingConsistency
  public: static const uint32_t kBitRatePrescalerIsZero            = 1 <<  0 ;
  public: static const uint32_t kBitRatePrescalerIsGreaterThan256  = 1 <<  1 ;
  public: static const uint32_t kPropagationSegmentIsZero          = 1 <<  2 ;
  public: static const uint32_t kPropagationSegmentIsGreaterThan8  = 1 <<  3 ;
  public: static const uint32_t kPhaseSegment1IsZero               = 1 <<  4 ;
  public: static const uint32_t kPhaseSegment1IsGreaterThan8       = 1 <<  5 ;
  public: static const uint32_t kPhaseSegment2IsZero               = 1 <<  6 ;
  public: static const uint32_t kPhaseSegment2IsGreaterThan8       = 1 <<  7 ;
  public: static const uint32_t kRJWIsZero                         = 1 <<  8 ;
  public: static const uint32_t kRJWIsGreaterThan4                 = 1 <<  9 ;
  public: static const uint32_t kRJWIsGreaterThanPhaseSegment2     = 1 << 10 ;
  public: static const uint32_t kPhaseSegment1Is1AndTripleSampling = 1 << 11 ;
} ;

//--------------------------------------------------------------------------------------------------
