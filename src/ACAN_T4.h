//--------------------------------------------------------------------------------------------------
// A simple Arduino Teensy 4.0 CAN driver
// by Pierre Molinaro
// https://github.com/pierremolinaro/ACAN_T4
//
//--------------------------------------------------------------------------------------------------

#pragma once

//--------------------------------------------------------------------------------------------------

#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.0"
#endif

//--------------------------------------------------------------------------------------------------

#include <ACAN_T4_Settings.h>
#include <ACAN_T4FD_Settings.h>
#include <ACAN_T4_CANFDMessage.h>

//--------------------------------------------------------------------------------------------------

typedef enum {kActive, kPassive, kBusOff} tControllerState ;

//--------------------------------------------------------------------------------------------------

class ACANPrimaryFilter {
  public: uint32_t mPrimaryFilterMask ;
  public: uint32_t mPrimaryAcceptanceFilter ;
  public: ACANCallBackRoutine mCallBackRoutine ;

  public: inline ACANPrimaryFilter (const ACANCallBackRoutine inCallBackRoutine) :  // Accept any frame
  mPrimaryFilterMask (0),
  mPrimaryAcceptanceFilter (0),
  mCallBackRoutine (inCallBackRoutine) {
  }

  public: ACANPrimaryFilter (const tFrameKind inKind,
                             const tFrameFormat inFormat, // Accept any identifier
                             const ACANCallBackRoutine inCallBackRoutine = nullptr) ;

  public: ACANPrimaryFilter (const tFrameKind inKind,
                             const tFrameFormat inFormat,
                             const uint32_t inIdentifier,
                             const ACANCallBackRoutine inCallBackRoutine = nullptr) ;

  public: ACANPrimaryFilter (const tFrameKind inKind,
                             const tFrameFormat inFormat,
                             const uint32_t inMask,
                             const uint32_t inAcceptance,
                             const ACANCallBackRoutine inCallBackRoutine = nullptr) ;
} ;

//--------------------------------------------------------------------------------------------------

class ACANSecondaryFilter {
  public: uint32_t mSecondaryAcceptanceFilter ;
  public: ACANCallBackRoutine mCallBackRoutine ;

  public: ACANSecondaryFilter (const tFrameKind inKind,
                               const tFrameFormat inFormat,
                               const uint32_t inIdentifier,
                               const ACANCallBackRoutine inCallBackRoutine = nullptr) ;
} ;

//--------------------------------------------------------------------------------------------------

class ACANFDFilter {
  public: uint32_t mFilterMask ;
  public: uint32_t mAcceptanceMask ;
  public: ACANFDCallBackRoutine mCallBackRoutine ;

  public: ACANFDFilter (const ACANFDCallBackRoutine inCallBackRoutine = nullptr) ; // Receive any frame

  public: ACANFDFilter (const tFrameKind inKind,
                        const tFrameFormat inFormat, // Accept any identifier
                        const ACANFDCallBackRoutine inCallBackRoutine = nullptr) ;

  public: ACANFDFilter (const tFrameKind inKind,
                        const tFrameFormat inFormat,
                        const uint32_t inIdentifier,
                        const ACANFDCallBackRoutine inCallBackRoutine = nullptr) ;

  public: ACANFDFilter (const tFrameKind inKind,
                        const tFrameFormat inFormat,
                        const uint32_t inMask,
                        const uint32_t inAcceptance,
                        const ACANFDCallBackRoutine inCallBackRoutine = nullptr) ;
} ;

//--------------------------------------------------------------------------------------------------

enum class ACAN_T4_Module {CAN1, CAN2, CAN3} ;

//--------------------------------------------------------------------------------------------------

class ACAN_T4 {
//--- Constructor
  private: ACAN_T4 (const uint32_t mFlexcanBaseAddress,
                    const ACAN_T4_Module inModule) ;

//--- begin; returns a result code :
//  0 : Ok
//  other: every bit denotes an error
  public: static const uint32_t kTooMuchPrimaryFilters     = 1 << 31 ;
  public: static const uint32_t kNotConformPrimaryFilter   = 1 << 30 ;
  public: static const uint32_t kTooMuchSecondaryFilters   = 1 << 29 ;
  public: static const uint32_t kNotConformSecondaryFilter = 1 << 28 ;
  public: static const uint32_t kInvalidTxPin              = 1 << 27 ;
  public: static const uint32_t kInvalidRxPin              = 1 << 26 ;
  public: static const uint32_t kCANBitConfiguration       = 1 << 25 ;

//--- CANFD configuration errors
  public: static const uint32_t kCANFDNotAvailableOnCAN1AndCAN2 = 1 << 24 ;
  public: static const uint32_t kTooMuchCANFDFilters       = 1 << 23 ;
  public: static const uint32_t kCANFDInvalidRxMBCountVersusPayload = 1 << 22 ;

  public: uint32_t begin (const ACAN_T4_Settings & inSettings,
                          const ACANPrimaryFilter inPrimaryFilters [] = nullptr,
                          const uint32_t inPrimaryFilterCount = 0,
                          const ACANSecondaryFilter inSecondaryFilters [] = nullptr,
                          const uint32_t inSecondaryFilterCount = 0) ;

  public: uint32_t beginFD (const ACAN_T4FD_Settings & inSettings,
                            const ACANFDFilter inFilters [] = nullptr,
                            const uint32_t inFilterCount = 0) ;

//--- end: stop CAN controller
  public: void end (void) ;

//--- Transmitting messages
  public: bool tryToSend (const CANMessage & inMessage) ;
  public: bool tryToSendFD (const CANFDMessage & inMessage) ;
  public: inline uint32_t transmitBufferSize (void) const { return mTransmitBufferSize ; }
  public: inline uint32_t transmitBufferCount (void) const { return mTransmitBufferCount ; }
  public: inline uint32_t transmitBufferPeakCount (void) const { return mTransmitBufferPeakCount ; }

//--- Transmitting messages and return status (returns 0 if ok)
  public: uint32_t tryToSendReturnStatus (const CANMessage & inMessage) ;
  public: uint32_t tryToSendReturnStatusFD (const CANFDMessage & inMessage) ;
  public: static const uint32_t kTransmitBufferOverflow = 1 << 0 ;
  public: static const uint32_t kNoAvailableMBForSendingRemoteFrame = 1 << 1 ;
  public: static const uint32_t kNoReservedMBForSendingRemoteFrame = 1 << 2 ;
  public: static const uint32_t kMessageLengthExceedsPayload = 1 << 3 ;
  public: static const uint32_t kFlexCANinCAN20BMode = 1 << 4 ;
  public: static const uint32_t kFlexCANinCANFDMode = 1 << 5 ;

//--- Receiving messages
  public: inline bool available (void)   const { return (!mCANFD) && (mReceiveBufferCount > 0) ; }
  public: inline bool availableFD (void) const { return   mCANFD  && (mReceiveBufferCount > 0) ; }
  public: bool receive (CANMessage & outMessage) ;
  public: bool receiveFD (CANFDMessage & outMessage) ;
  public: typedef void (*tFilterMatchCallBack) (const uint32_t inFilterIndex) ;
  public: bool dispatchReceivedMessage (const tFilterMatchCallBack inFilterMatchCallBack = nullptr) ;
  public: bool dispatchReceivedMessageFD (const tFilterMatchCallBack inFilterMatchCallBack = nullptr) ;
  public: inline uint32_t receiveBufferSize (void) const { return mReceiveBufferSize ; }
  public: inline uint32_t receiveBufferCount (void) const { return mReceiveBufferCount ; }
  public: inline uint32_t receiveBufferPeakCount (void) const { return mReceiveBufferPeakCount ; }

//--- FlexCAN controller state
  public: tControllerState controllerState (void) const ;
  public: uint32_t receiveErrorCounter (void) const ;
  public: uint32_t transmitErrorCounter (void) const ;

//--- Call back function array
  private: ACANCallBackRoutine * mCallBackFunctionArray = nullptr ;
  private: ACANFDCallBackRoutine * mCallBackFunctionArrayFD = nullptr ; // null, or size is mRxCANFDMBCount
  private: uint32_t mCallBackFunctionArraySize = 0 ;

//--- Base address
  private: const uint32_t mFlexcanBaseAddress ;
  private: const ACAN_T4_Module mModule ; // Initialized in constructor

//--- CANFD properties
  private : bool mCANFD = false ;
  private : ACAN_T4FD_Settings::Payload mPayload = ACAN_T4FD_Settings::PAYLOAD_64_BYTES ;
  private : uint8_t mRxCANFDMBCount = 12 ;
  public : uint32_t RxCANFDMBCount (void) const { return mRxCANFDMBCount ; }

//--- Filters
  private : uint8_t mActualPrimaryFilterCount = 0 ;
  private : uint8_t mMaxPrimaryFilterCount = 0 ;
  private: uint32_t * mCANFDAcceptanceFilterArray = nullptr ; //

//--- Driver receive buffer
  private: CANMessage * mReceiveBuffer = nullptr ;
  private: CANFDMessage * mReceiveBufferFD = nullptr ;
  private: volatile uint32_t mReceiveBufferSize = 0 ;
  private: volatile uint32_t mReceiveBufferReadIndex = 0 ;
  private: volatile uint32_t mReceiveBufferCount = 0 ;
  private: volatile uint32_t mReceiveBufferPeakCount = 0 ; // == mReceiveBufferSize + 1 if overflow did occur

//--- Driver transmit buffer
  private: CANMessage * mTransmitBuffer = nullptr ;
  private: CANFDMessage * mTransmitBufferFD = nullptr ;
  private: volatile uint32_t mTransmitBufferSize = 0 ;
  private: volatile uint32_t mTransmitBufferReadIndex = 0 ;
  private: volatile uint32_t mTransmitBufferCount = 0 ;
  private: volatile uint32_t mTransmitBufferPeakCount = 0 ; // == mTransmitBufferSize + 1 if tentative overflow did occur

//--- Global status
  private : volatile uint32_t mGlobalStatus = 0 ; // Returns 0 if all is ok
  public : uint32_t globalStatus (void) const { return mGlobalStatus ; }
  public : void resetGlobalStatus (const uint32_t inReset) ;
//--- Global status bit names
  public: static const uint32_t kGlobalStatusInitError     = 1 <<  0 ;
  public: static const uint32_t kGlobalStatusRxFIFOWarning = 1 <<  1 ; // Occurs when the number of messages goes from 4 to 5
  public: static const uint32_t kGlobalStatusRxFIFOOverflow = 1 <<  2 ; // Occurs when RxFIFO overflows
  public: static const uint32_t kGlobalStatusReceiveBufferOverflow = 1 <<  3 ; // Occurs when driver receive buffer overflows

//--- Message interrupt service routine
  public: void message_isr (void) ;

//--- Driver instance
  public: static ACAN_T4 can1 ;
  public: static ACAN_T4 can2 ;
  public: static ACAN_T4 can3 ;

//--- Private methods
  private : uint32_t tryToSendRemoteFrame (const CANMessage & inMessage) ;
  private : uint32_t tryToSendDataFrame (const CANMessage & inMessage) ;
  private : void writeTxRegisters (const CANMessage & inMessage, const uint32_t inMBIndex) ;
  private : uint32_t tryToSendDataFrameFD (const CANFDMessage & inMessage) ;
  private : uint32_t tryToSendRemoteFrameFD (const CANFDMessage & inMessage) ;
  private : void writeTxRegistersFD (const CANFDMessage & inMessage, volatile uint32_t * inMBAddress) ;
  private : void message_isr_receive (void) ;
  private : void message_isr_receiveFD (const uint32_t inReceiveMailboxIndex) ;
  private : void message_isr_FD (void) ;
  private: void readRxRegisters (CANMessage & outMessage) ;
  private : void readRxRegistersFD (CANFDMessage & outMessage, const uint32_t inReceiveMailboxIndex) ;

//--- No copy
  private : ACAN_T4 (const ACAN_T4 &) = delete ;
  private : ACAN_T4 & operator = (const ACAN_T4 &) = delete ;
} ;

//--------------------------------------------------------------------------------------------------

void flexcan_isr_can3 (void) ; // For CANFD

//--------------------------------------------------------------------------------------------------
