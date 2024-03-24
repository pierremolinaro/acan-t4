//--------------------------------------------------------------------------------------------------
// A Teensy 4.0 CAN driver
// by Pierre Molinaro
// https://github.com/pierremolinaro/ACAN_T4
//
//--------------------------------------------------------------------------------------------------

#pragma once

//--------------------------------------------------------------------------------------------------

#include <ACAN_T4_DataBitRateFactor.h>
#include <ACAN_T4_T4FD_rootCANClock.h>

//--------------------------------------------------------------------------------------------------

class ACAN_T4FD_Settings {

//··································································································
//   Enumerations
//··································································································

  public: typedef enum : uint8_t {
    NO_PULLUP_NO_PULLDOWN = 0, // PUS = 0, PUE = 0, PKE = 0
    PULLDOWN_100k = 0b0011, // PUS = 0, PUE = 1, PKE = 1
    PULLUP_47k    = 0b0111, // PUS = 1, PUE = 1, PKE = 1
    PULLUP_100k   = 0b1011, // PUS = 2, PUE = 1, PKE = 1
    PULLUP_22k    = 0b1111  // PUS = 3, PUE = 1, PKE = 1
  } RxPinConfiguration ;

//··································································································

  public: typedef enum : uint8_t {
    IMPEDANCE_R0 = 1,
    IMPEDANCE_R0_DIVIDED_BY_2 = 2,
    IMPEDANCE_R0_DIVIDED_BY_3 = 3,
    IMPEDANCE_R0_DIVIDED_BY_4 = 4,
    IMPEDANCE_R0_DIVIDED_BY_5 = 5,
    IMPEDANCE_R0_DIVIDED_BY_6 = 6,
    IMPEDANCE_R0_DIVIDED_BY_7 = 7
  } TxPinOutputBufferImpedance ;

//··································································································

  public: typedef enum : uint8_t {
    PAYLOAD_8_BYTES  = 0,
    PAYLOAD_16_BYTES = 1,
    PAYLOAD_32_BYTES = 2,
    PAYLOAD_64_BYTES = 3
  } Payload ;

//··································································································
//    Constructor for a given baud rate
//··································································································

  public: explicit ACAN_T4FD_Settings (const uint32_t inWhishedBitRate,
                                       const DataBitRateFactor inDataBitRateFactor,
                                       const uint32_t inTolerancePPM = 1000) ;

//··································································································
//    Properties
//··································································································

//--- CAN 2.0B bit timing (default values correspond to 250 kb/s, DATA_BITRATE_x1)
  public: uint32_t mWhishedArbitrationBitRate ; // In kb/s
//--- bitrate prescaler is common to arbitration bitrate and data bitrate
  public: uint16_t mBitRatePrescaler = 10 ; // 1...1024
//--- Arbitration segments
  public: uint8_t mArbitrationPropagationSegment = 8 ; // 1...64
  public: uint8_t mArbitrationPhaseSegment1 = 8 ; // 1...32
  public: uint8_t mArbitrationPhaseSegment2 = 7 ;  // 2...32
  public: uint8_t mArbitrationRJW = 4 ; // 1...32
//--- Data segments
  public: uint8_t mDataPropagationSegment = 8 ; // 1...32
  public: uint8_t mDataPhaseSegment1 = 8 ; // 1...8
  public: uint8_t mDataPhaseSegment2 = 7 ;  // 2...8
  public: uint8_t mDataRJW = 4 ; // 1...8

  public: bool mTripleSampling = false ; // true --> triple sampling, false --> single sampling
  public: bool mBitSettingOk = true ; // The above configuration is correct

//--- Payload (used in CANFD mode)
  public : Payload mPayload = PAYLOAD_64_BYTES ;

//--- Number of Rx MBs (used in CANFD mode)
  public : uint8_t mRxCANFDMBCount = 11 ; // 1 ... depends from mPayload, see documentation

//--- Listen only mode
  public: bool mListenOnlyMode = false ; // true --> listen only mode, cannot send any message, false --> normal mode

//--- Self Reception mode
  public: bool mSelfReceptionMode = false ; // true --> sent frame are also received, false --> are not received

//--- Loop Back mode
  public: bool mLoopBackMode = false ; // true --> loop back mode, false --> no loop back

// false --> Do NOT include Stuff Bit Count in CRC Field and use CRC Initialization Vector with all zeros
// true --> Include Stuff Bit Count in CRC Field and use Non-Zero CRC Initialization Vector according to ISO 11898-1:2015
  public: bool mISOCRCEnabled = true ;

//--- Tx pin configuration
  public: TxPinOutputBufferImpedance mTxPinOutputBufferImpedance = IMPEDANCE_R0_DIVIDED_BY_6 ;
  public: bool mTxPinIsOpenCollector = false ; // false --> totem pole, true --> open collector

//--- Rx pin configuration
  public: RxPinConfiguration mRxPinConfiguration = PULLUP_47k ;

//--- Receive buffer size
  public: uint16_t mReceiveBufferSize = 32 ;

//--- Transmit buffer size
  public: uint16_t mTransmitBufferSize = 16 ;

//··································································································
// Accessors
//··································································································

//--- Compute actual bitrate
  public: uint32_t actualArbitrationBitRate (void) const ;
  public: uint32_t actualDataBitRate (void) const ;

//--- Exact bitrate ?
  public: bool exactArbitrationBitRate (void) const ;

//--- Distance between actual bitrate and requested bitrate (in ppm, part-per-million)
  public: uint32_t ppmFromWishedBitRate (void) const ;

//--- Distance of sample point from bit start (in ppc, part-per-cent, denoted by %)
  public: uint32_t arbitrationSamplePointFromBitStart (void) const ;
  public: uint32_t dataSamplePointFromBitStart (void) const ;

//--- Bit settings are consistent ? (returns 0 if ok)
  public: uint32_t CANFDBitSettingConsistency (void) const ;

//··································································································
// Constants returned by CANBitSettingConsistency
//··································································································

  public: static const uint32_t kBitRatePrescalerIsZero                       = 1 <<  0 ;
  public: static const uint32_t kBitRatePrescalerIsGreaterThan1024            = 1 <<  1 ;
  public: static const uint32_t kArbitrationPropagationSegmentIsZero          = 1 <<  2 ;
  public: static const uint32_t kArbitrationPropagationSegmentIsGreaterThan64 = 1 <<  3 ;
  public: static const uint32_t kArbitrationPhaseSegment1IsZero               = 1 <<  4 ;
  public: static const uint32_t kArbitrationPhaseSegment1IsGreaterThan32      = 1 <<  5 ;
  public: static const uint32_t kArbitrationPhaseSegment2IsLowerThan2         = 1 <<  6 ;
  public: static const uint32_t kArbitrationPhaseSegment2IsGreaterThan32      = 1 <<  7 ;
  public: static const uint32_t kArbitrationRJWIsZero                         = 1 <<  8 ;
  public: static const uint32_t kArbitrationRJWIsGreaterThan32                = 1 <<  9 ;
  public: static const uint32_t kArbitrationRJWIsGreaterThanPhaseSegment2     = 1 << 10 ;
  public: static const uint32_t kArbitrationPhaseSegment1Is1AndTripleSampling = 1 << 11 ;

  public: static const uint32_t kDataPropagationSegmentIsZero                 = 1 << 12 ;
  public: static const uint32_t kDataPropagationSegmentIsGreaterThan32        = 1 << 13 ;
  public: static const uint32_t kDataPhaseSegment1IsZero                      = 1 << 14 ;
  public: static const uint32_t kDataPhaseSegment1IsGreaterThan8              = 1 << 15 ;
  public: static const uint32_t kDataPhaseSegment2IsLowerThan2                = 1 << 16 ;
  public: static const uint32_t kDataPhaseSegment2IsGreaterThan8              = 1 << 17 ;
  public: static const uint32_t kDataRJWIsZero                                = 1 << 18 ;
  public: static const uint32_t kDataRJWIsGreaterThan8                        = 1 << 19 ;
  public: static const uint32_t kDataRJWIsGreaterThanPhaseSegment2            = 1 << 20 ;

//··································································································

} ;

//--------------------------------------------------------------------------------------------------

uint32_t MBCount (const ACAN_T4FD_Settings::Payload inPayload) ;

//--------------------------------------------------------------------------------------------------
