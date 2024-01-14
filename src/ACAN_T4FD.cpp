//----------------------------------------------------------------------------------------
// A Teensy 4.x CANFD driver for FLEXCAN3
// by Pierre Molinaro
// https://github.com/pierremolinaro/acan-t4
//
//----------------------------------------------------------------------------------------
//  Teensy 4.0 / 4.1 FlexCAN pins
//
//  FLEXCAN3
//    TX: #31 (default)
//    RX: #30 (default)
//
//----------------------------------------------------------------------------------------

#include <ACAN_T4.h>
#include <algorithm>

//----------------------------------------------------------------------------------------
//   FLEXCAN REGISTERS
//----------------------------------------------------------------------------------------

#define FLEXCAN_MCR(b)      (*((volatile uint32_t *) ((b)+0x00)))
#define FLEXCAN_CTRL1(b)    (*((volatile uint32_t *) ((b)+0x04)))
#define FLEXCAN_TIMER(b)    (*((volatile uint32_t *) ((b)+0x08)))
#define FLEXCAN_ECR(b)      (*((volatile uint32_t *) ((b)+0x1C)))
#define FLEXCAN_ESR1(b)     (*((volatile uint32_t *) ((b)+0x20)))
#define FLEXCAN_IMASK2(b)   (*((volatile uint32_t *) ((b)+0x24)))
#define FLEXCAN_IMASK1(b)   (*((volatile uint32_t *) ((b)+0x28)))
#define FLEXCAN_IFLAG2(b)   (*((volatile uint32_t *) ((b)+0x2C)))
#define FLEXCAN_IFLAG1(b)   (*((volatile uint32_t *) ((b)+0x30)))
#define FLEXCAN_CTRL2(b)    (*((volatile uint32_t *) ((b)+0x34)))
#define FLEXCAN_RXFGMASK(b) (*((volatile uint32_t *) ((b)+0x48)))
#define FLEXCAN_RXFIR(b)    (*((volatile uint32_t *) ((b)+0x4C)))
#define FLEXCAN_CBT(b)      (*((volatile uint32_t *) ((b)+0x50)))
#define FLEXCAN_FDCTRL(b)   (*((volatile uint32_t *) ((b)+0xC00)))
#define FLEXCAN_FDCBT(b)    (*((volatile uint32_t *) ((b)+0xC04)))

// #define FLEXCAN_IDAF(b, n)    (*((volatile uint32_t *) ((b)+0xE0+(n)*4)))
#define FLEXCAN_MB_MASK(b, n) (*((volatile uint32_t *) ((b)+0x880+(n)*4)))

//--- Definitions FLEXCAN_MB_CS
static const uint32_t FLEXCAN_MB_CS_RTR       = 1 << 20 ;
static const uint32_t FLEXCAN_MB_CS_IDE       = 1 << 21 ;
static const uint32_t FLEXCAN_MB_CS_SRR       = 1 << 22 ;
static const uint32_t FLEXCAN_MB_CS_ESI       = 1 << 29 ;
static const uint32_t FLEXCAN_MB_CS_BRS       = 1 << 30 ;
static const uint32_t FLEXCAN_MB_CS_EDL       = 1 << 31 ;

static inline uint32_t FLEXCAN_MB_CS_LENGTH (const uint32_t inLength) { return (inLength & 0xF) << 16 ; }

static inline uint32_t FLEXCAN_MB_CS_CODE (const uint32_t inCode) { return (inCode & 0xF) << 24 ; }

static inline uint32_t FLEXCAN_get_code (const uint32_t inValue) {
  return (inValue >> 24) & 0x0F ;
}

static inline uint32_t FLEXCAN_get_length (const uint32_t inValue) {
  return (inValue >> 16) & 0x0F ;
}

static const uint32_t FLEXCAN_MB_CODE_TX_FULL     = 0x02 ;

static const uint32_t FLEXCAN_MB_CODE_TX_EMPTY    = 0x04 ;

static const uint32_t FLEXCAN_MB_CODE_TX_OVERRUN  = 0x06 ;

static const uint32_t FLEXCAN_MB_CODE_TX_INACTIVE = 0x08 ;

static const uint32_t FLEXCAN_MB_CODE_TX_ONCE     = 0x0C ;

//--- Definitions for FLEXCAN_MCR
static const uint32_t FLEXCAN_MCR_FDEN     = 0x00000800 ;

static const uint32_t FLEXCAN_MCR_IRMQ     = 0x00010000 ;

static const uint32_t FLEXCAN_MCR_SRX_DIS  = 0x00020000 ;

static const uint32_t FLEXCAN_MCR_LPM_ACK  = 0x00100000 ;

static const uint32_t FLEXCAN_MCR_FRZ_ACK  = 0x01000000 ;

static const uint32_t FLEXCAN_MCR_SOFT_RST = 0x02000000 ;

static const uint32_t FLEXCAN_MCR_NOT_RDY  = 0x08000000 ;

static const uint32_t FLEXCAN_MCR_HALT     = 0x10000000 ;

//--- Definitions for FLEXCAN_CTRL
static const uint32_t FLEXCAN_CTRL_LOM = 0x00000008 ;

static const uint32_t FLEXCAN_CTRL_SMP = 0x00000080 ;

static const uint32_t FLEXCAN_CTRL_LPB = 0x00001000 ;

//--- Definitions for FLEXCAN_CBT
static inline uint32_t FLEXCAN_CBT_PROPSEG (const uint32_t inPropSeg) { return inPropSeg << 10 ; }

static inline uint32_t FLEXCAN_CBT_PSEG2 (const uint32_t inPSeg2) { return inPSeg2 << 0 ; }

static inline uint32_t FLEXCAN_CBT_PSEG1 (const uint32_t inPSeg1) { return inPSeg1 << 5 ; }

static inline uint32_t FLEXCAN_CBT_RJW (const uint32_t inRJW) { return inRJW << 16 ; }

static inline uint32_t FLEXCAN_CBT_PRESDIV (const uint32_t inPreDivisor) { return inPreDivisor << 21 ; }


//--- Definitions for FLEXCAN_FDCBT
static inline uint32_t FLEXCAN_FDCBT_PROPSEG (const uint32_t inPropSeg) { return inPropSeg << 10 ; }

static inline uint32_t FLEXCAN_FDCBT_PSEG2 (const uint32_t inPSeg2) { return inPSeg2 << 0 ; }

static inline uint32_t FLEXCAN_FDCBT_PSEG1 (const uint32_t inPSeg1) { return inPSeg1 << 5 ; }

static inline uint32_t FLEXCAN_FDCBT_RJW (const uint32_t inRJW) { return inRJW << 16 ; }

static inline uint32_t FLEXCAN_FDCBT_PRESDIV (const uint32_t inPreDivisor) { return inPreDivisor << 20 ; }

//--- Definitions for FLEXCAN_MB_ID
static const uint32_t FLEXCAN_MB_ID_EXT_MASK = 0x1FFFFFFF ;

static inline uint32_t FLEXCAN_MB_ID_IDSTD (const uint32_t inStandardId) { return (inStandardId & 0x7FF) << 18 ; }

static const uint32_t FLEXCAN_MB_ID_STD_BIT_NO = 18 ;

//--- Definitions for FLEXCAN_FDCTRL (§44.6.2.25, page 2810)
static const uint32_t FLEXCAN_FDCTRL_FDRATE = 1 << 31 ;
static const uint32_t FLEXCAN_FDCTRL_TDCEN  = 1 << 15 ;

static inline uint32_t FLEXCAN_FDCTRL_MBDSR1 (const ACAN_T4FD_Settings::Payload inValue) { return uint32_t (inValue) << 19; }

static inline uint32_t FLEXCAN_FDCTRL_MBDSR0 (const ACAN_T4FD_Settings::Payload inValue) { return uint32_t (inValue) << 16; }

//----------------------------------------------------------------------------------------
//    CANFD LENGTH CODE
//----------------------------------------------------------------------------------------

static const uint8_t CANFD_LENGTH_CODE [16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64} ;

//----------------------------------------------------------------------------------------
//   64-bit ONE
//----------------------------------------------------------------------------------------

static const uint64_t ONE = 1 ;

//----------------------------------------------------------------------------------------
//   MAILBOXES
//----------------------------------------------------------------------------------------

static uint32_t dataWordsForPayload (const ACAN_T4FD_Settings::Payload inPayload) {
  return 2 << uint32_t (inPayload) ;
}

//----------------------------------------------------------------------------------------

static volatile uint32_t * mailboxAddress (const uint32_t inFlexcanBaseAddress,
                                           const ACAN_T4FD_Settings::Payload inPayload,
                                           const uint32_t inMailboxIndex) {
  uint32_t address = inFlexcanBaseAddress + 0x0080 ; // Base address
  switch (inPayload) {
  case ACAN_T4FD_Settings::PAYLOAD_8_BYTES : // 64 MB, table 45-29 page 2711
    address += 16 * inMailboxIndex ;
    break ;
  case ACAN_T4FD_Settings::PAYLOAD_16_BYTES : // 42 MB, table 45-30 page 2713
    address += 24 * inMailboxIndex ;
    if (inMailboxIndex >= 21) {
      address += 8 ;
    }
    break ;
  case ACAN_T4FD_Settings::PAYLOAD_32_BYTES : // 24 MB, table 45-31 page 2714
    address += 40 * inMailboxIndex ;
    if (inMailboxIndex >= 12) {
      address += 32 ;
    }
    break ;
  case ACAN_T4FD_Settings::PAYLOAD_64_BYTES :  // 14 MB, table 45-32 page 2715
    address += 72 * inMailboxIndex ;
    if (inMailboxIndex >= 7) {
      address += 8 ;
    }
    break ;
  }
  return (volatile uint32_t *) address ;
}

//----------------------------------------------------------------------------------------
//    beginFD method
//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::beginFD (const ACAN_T4FD_Settings & inSettings,
                           const ACANFDFilter inFilters [],
                           const uint32_t inFilterCount) {
  uint32_t errorCode = inSettings.CANFDBitSettingConsistency () ; // No error code
//--- No configuration if CAN bit settings are incorrect
  if (!inSettings.mBitSettingOk) {
    errorCode |= kCANBitConfiguration ;
  }
//--- Error if beginFD is called on CAN1 or CAN2
  if (mModule != ACAN_T4_Module::CAN3) {
    errorCode |= kCANFDNotAvailableOnCAN1AndCAN2 ;
  }
//--- Error if inFilterCount > inSettings.mRxCANFDMBCount
  if (inFilterCount > inSettings.mRxCANFDMBCount) {
    errorCode |= kTooMuchCANFDFilters ;
  }
  if (inSettings.mRxCANFDMBCount >= (MBCount (inSettings.mPayload) - 1)) {
    errorCode |= kCANFDInvalidRxMBCountVersusPayload ;
  }
//--- Configure if no error
  if (0 == errorCode) {
    mCANFD = true ;
    mPayload = inSettings.mPayload ;
  //---------- Allocate receive buffer
    mReceiveBufferSize = inSettings.mReceiveBufferSize ;
    mReceiveBufferFD = new CANFDMessage [inSettings.mReceiveBufferSize] ;
  //---------- Allocate transmit buffer
    mTransmitBufferSize = inSettings.mTransmitBufferSize ;
    mTransmitBufferFD = new CANFDMessage [inSettings.mTransmitBufferSize] ;
  //---------- Select clock source (see i.MX RT1060 Processor Reference Manual, Rev. 2, 12/2019, page 1059)
    uint32_t cscmr2 = CCM_CSCMR2 & 0xFFFFFC03 ;
    cscmr2 |= CCM_CSCMR2_CAN_CLK_PODF (getCANRootClockDivisor () - 1) ;
    switch (getCANRootClock ()) {
    case ACAN_CAN_ROOT_CLOCK::CLOCK_24MHz :
      cscmr2 |= CCM_CSCMR2_CAN_CLK_SEL (1) ;
      break ;
    case ACAN_CAN_ROOT_CLOCK::CLOCK_60MHz :
      cscmr2 |= CCM_CSCMR2_CAN_CLK_SEL (0) ;
      break ;
//     case ACAN_CAN_ROOT_CLOCK::CLOCK_80MHz :
//       cscmr2 |= CCM_CSCMR2_CAN_CLK_SEL (2) ;
//       break ;
    }
    CCM_CSCMR2 = cscmr2 ;
  //---------- Vectors
    CCM_CCGR7 |= 0x3C0 ;
    _VectorsRam [16 + IRQ_CAN3] = flexcan_isr_can3 ;
  //---------- Enable CANFD
    const uint32_t lastMailboxIndex = MBCount (inSettings.mPayload) - 1 ;
    FLEXCAN_MCR (mFlexcanBaseAddress) =
      (1 << 30) | // Enable to enter to freeze mode
      (1 << 23) | // FlexCAN is in supervisor mode
      (lastMailboxIndex << 0) // Mailboxes
    ;
    while (FLEXCAN_MCR(mFlexcanBaseAddress) & FLEXCAN_MCR_LPM_ACK) {}
  //---------- Soft reset
    FLEXCAN_MCR(mFlexcanBaseAddress) |= FLEXCAN_MCR_SOFT_RST ;
    while (FLEXCAN_MCR(mFlexcanBaseAddress) & FLEXCAN_MCR_SOFT_RST) {}
  //---------- Wait for freeze ack
    while (!(FLEXCAN_MCR(mFlexcanBaseAddress) & FLEXCAN_MCR_FRZ_ACK)) {}
  //--- FDCTRL (§44.6.2.21, page 2697)
    uint32_t v =
      FLEXCAN_FDCTRL_FDRATE // Bit 31: enable bitrate Switch
      | FLEXCAN_FDCTRL_MBDSR1 (inSettings.mPayload)
      | FLEXCAN_FDCTRL_MBDSR0 (inSettings.mPayload)
    ;
    if (!inSettings.mLoopBackMode) {
      v |= FLEXCAN_FDCTRL_TDCEN // Transceiver Delay Compensation Enable
        | (8 << 8) // Transceiver Delay Compensation Offset
     ;
    }
    FLEXCAN_FDCTRL (mFlexcanBaseAddress) = v ;
  //---------- Can settings
    FLEXCAN_MCR (mFlexcanBaseAddress) |=
      (inSettings.mSelfReceptionMode ? 0 : FLEXCAN_MCR_SRX_DIS) | // Disable self-reception ?
      FLEXCAN_MCR_FDEN | // FDEN: CAN FD operation enable
      FLEXCAN_MCR_IRMQ | // Enable per-mailbox filtering (§56.4.2)
      (lastMailboxIndex << 0) // Mailboxes
    ;
  //---------- CTRL1 (CBT, §44.6.2.3, page 2768)
    FLEXCAN_CTRL1 (mFlexcanBaseAddress) =
//      (1 << 13) | // CAN Engine Clock Source ?
      (inSettings.mTripleSampling ? FLEXCAN_CTRL_SMP : 0) |
      (inSettings.mLoopBackMode ? FLEXCAN_CTRL_LPB : 0) |
      (inSettings.mListenOnlyMode ? FLEXCAN_CTRL_LOM : 0)
    ;
  //---------- Arbitration Can bit timing (CBT, §44.6.2.19, page 2801)
    FLEXCAN_CBT (mFlexcanBaseAddress) =
      (1 << 31) | // Enable this register
      FLEXCAN_CBT_PROPSEG (inSettings.mArbitrationPropagationSegment - 1) |
      FLEXCAN_CBT_RJW (inSettings.mArbitrationRJW - 1) |
      FLEXCAN_CBT_PSEG1 (inSettings.mArbitrationPhaseSegment1 - 1) |
      FLEXCAN_CBT_PSEG2 (inSettings.mArbitrationPhaseSegment2 - 1) |
      FLEXCAN_CBT_PRESDIV (inSettings.mBitRatePrescaler - 1)
    ;
  //---------- Data Can bit timing (FDCBT, §44.6.2.26, page 2813)
    FLEXCAN_FDCBT (mFlexcanBaseAddress) =
      FLEXCAN_FDCBT_PROPSEG (inSettings.mDataPropagationSegment) |
      FLEXCAN_FDCBT_RJW (inSettings.mDataRJW - 1) |
      FLEXCAN_FDCBT_PSEG1 (inSettings.mDataPhaseSegment1 - 1) |
      FLEXCAN_FDCBT_PSEG2 (inSettings.mDataPhaseSegment2 - 1) |
      FLEXCAN_FDCBT_PRESDIV (inSettings.mBitRatePrescaler - 1)
    ;
  //---------- CTRL2 (§44.6.2.14, page 2791)
    FLEXCAN_CTRL2 (mFlexcanBaseAddress) =
      (0 << 19) | // TASD: 0x16 is the default value
      (1 << 17) | // RRS: Received remote request frame is stored
      (1 << 16) | // EACEN: RTR bit in mask is always compared
      (1 << 13) | // Bit Timing Expansion Enable
      (inSettings.mISOCRCEnabled ? (1 << 12) : 0)   // ISO CANFD Enable
    ;
  //---------- Filters
    if (inFilterCount > 0) {
      mCallBackFunctionArrayFD = new ACANFDCallBackRoutine [inSettings.mRxCANFDMBCount] ;
      mCANFDAcceptanceFilterArray = new uint32_t [inSettings.mRxCANFDMBCount] ;
      for (uint32_t i=0 ; i < inFilterCount ; i++) {
        mCallBackFunctionArrayFD [i] = inFilters [i].mCallBackRoutine ;
        FLEXCAN_MB_MASK (mFlexcanBaseAddress, i+1) = inFilters [i].mFilterMask ;
        mCANFDAcceptanceFilterArray [i] = inFilters [i].mAcceptanceMask ;
//         Serial.print ("Filter ") ;
//         Serial.print (i) ;
//         Serial.print (": mask 0x") ;
//         Serial.print (inFilters [i].mFilterMask, HEX) ;
//         Serial.print (", acceptance 0x") ;
//         Serial.println (inFilters [i].mAcceptanceMask, HEX) ;
      }
      for (uint32_t i = inFilterCount ; i < inSettings.mRxCANFDMBCount ; i++) {
        mCallBackFunctionArrayFD [i] = inFilters [inFilterCount - 1].mCallBackRoutine ;
        FLEXCAN_MB_MASK (mFlexcanBaseAddress, i+1) = inFilters [inFilterCount - 1].mFilterMask ;
        mCANFDAcceptanceFilterArray [i] = inFilters [inFilterCount - 1].mAcceptanceMask ;
      }
    }else{
      for (uint32_t i = 1 ; i <= inSettings.mRxCANFDMBCount ; i++) {
        FLEXCAN_MB_MASK (mFlexcanBaseAddress, i) = 0 ; // Accept any
      }
    }
  //--- Make all mailboxes inactives
    for (uint32_t i = 0 ; i < MBCount (mPayload) ; i++) {
      volatile uint32_t * mailBoxAddress = mailboxAddress (mFlexcanBaseAddress, mPayload, i) ;
      mailBoxAddress [0] = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_INACTIVE) ;
    }
  //--- Make Rx mailboxes ready
    mRxCANFDMBCount = inSettings.mRxCANFDMBCount ;
    for (uint32_t i = 1 ; i <= mRxCANFDMBCount ; i++) {
      volatile uint32_t * RxMailBoxAddress = mailboxAddress (mFlexcanBaseAddress, mPayload, i) ;
      uint32_t code = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_EMPTY) ;
      if (mCANFDAcceptanceFilterArray != nullptr) {
        RxMailBoxAddress [1] = mCANFDAcceptanceFilterArray [i-1] & 0x1FFFFFFF ; // Write MB acceptance filter
        if ((mCANFDAcceptanceFilterArray [i-1] & (1 << 31)) != 0) {
          code |= (1 << 20) ; // Filter remote / data
        }
        if ((mCANFDAcceptanceFilterArray [i-1] & (1 << 30)) != 0) {
          code |= (1 << 21) ; // Filter standard / extended
        }
      }
      RxMailBoxAddress [0] = code ;
    }
  //---------- Select TX pin
    uint32_t TxPinConfiguration = IOMUXC_PAD_DSE (inSettings.mTxPinOutputBufferImpedance) ;
    TxPinConfiguration |= (3 << 6) ; // Speed 200 MHz (max)
    TxPinConfiguration |= (1 << 0) ; // Fast Slew Rate
    if (inSettings.mTxPinIsOpenCollector) {
      TxPinConfiguration |= IOMUXC_PAD_ODE ;
    }
    CORE_PIN31_CONFIG = 0x19 ; // Pin #31 SION + ALT9
    CORE_PIN31_PADCONFIG = TxPinConfiguration ; // Pin #31
  //---------- Select RX Pin
    uint32_t RxPinConfiguration = uint32_t (inSettings.mRxPinConfiguration) << 12 ;
    RxPinConfiguration |= (3 << 6) ; // Speed 200 MHz (max)
    RxPinConfiguration |= (1 << 0) ; // Fast Slew Rate
//    RxPinConfiguration |= IOMUXC_PAD_HYS ; // Hysterisis
    IOMUXC_CANFD_IPP_IND_CANRX_SELECT_INPUT = 0x00 ; // Page 1007, select GPIO_EMC_37_ALT9
    CORE_PIN30_CONFIG = 0x19 ; // Pin #30 SION + ALT9
    CORE_PIN30_PADCONFIG = RxPinConfiguration ; // Pin #30
  //---------- Start CAN
    FLEXCAN_MCR (mFlexcanBaseAddress) &= ~FLEXCAN_MCR_HALT ;
  //---------- Wait until exit of freeze mode
    while (FLEXCAN_MCR (mFlexcanBaseAddress) & FLEXCAN_MCR_FRZ_ACK) {}
  //----------  Wait until ready
    while (FLEXCAN_MCR (mFlexcanBaseAddress) & FLEXCAN_MCR_NOT_RDY) {}
  //---------- Enable NVIC interrupts
    NVIC_ENABLE_IRQ (IRQ_CAN3) ;
  //---------- Enable CAN interrupts
    const uint32_t txMBindex = MBCount (mPayload) - 1 ;
    const uint64_t interruptEnableBits =
        (((ONE << mRxCANFDMBCount) - ONE) << 1)  // Frame available in Rx MB interrupt
      | (ONE << txMBindex)  // Tx MB becomes free interrupt
    ;
    FLEXCAN_IMASK1 (mFlexcanBaseAddress) = uint32_t (interruptEnableBits) ;
    FLEXCAN_IMASK2 (mFlexcanBaseAddress) = uint32_t (interruptEnableBits >> 32) ;
  }
//---
  mGlobalStatus = (errorCode == 0) ? 0 : kGlobalStatusInitError ;
//--- Return error code (0 --> no error)
  return errorCode ;
}

//----------------------------------------------------------------------------------------
//   RECEPTION
//----------------------------------------------------------------------------------------

bool ACAN_T4::receiveFD (CANFDMessage & outMessage) {
  noInterrupts () ;
    const bool hasMessage = mCANFD && (mReceiveBufferCount > 0) && ((mGlobalStatus & kGlobalStatusInitError) == 0) ;
    if (hasMessage) {
      outMessage = mReceiveBufferFD [mReceiveBufferReadIndex] ;
      mReceiveBufferReadIndex = (mReceiveBufferReadIndex + 1) % mReceiveBufferSize ;
      mReceiveBufferCount -= 1 ;
    }
  interrupts ()
  return hasMessage ;
}

//----------------------------------------------------------------------------------------

bool ACAN_T4::dispatchReceivedMessageFD (const tFilterMatchCallBack inFilterMatchCallBack) {
  CANFDMessage receivedMessage ;
  const bool hasReceived = receiveFD (receivedMessage) ;
  if (hasReceived) {
    const uint32_t filterIndex = receivedMessage.idx ;
    if (nullptr != inFilterMatchCallBack) {
      inFilterMatchCallBack (filterIndex) ;
    }
    if (filterIndex < mRxCANFDMBCount) {
      ACANFDCallBackRoutine callBackFunction = mCallBackFunctionArrayFD [filterIndex] ;
      if (nullptr != callBackFunction) {
        callBackFunction (receivedMessage) ;
      }
    }
  }
  return hasReceived ;
}

//----------------------------------------------------------------------------------------
//   EMISSION
//----------------------------------------------------------------------------------------

bool ACAN_T4::tryToSendFD (const CANFDMessage & inMessage) {
  return tryToSendReturnStatusFD (inMessage) == 0 ;
}

//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::tryToSendReturnStatusFD (const CANFDMessage & inMessage) {
  uint32_t sendStatus = 0 ;
  if (!mCANFD) {
    sendStatus = kFlexCANinCAN20BMode ;
  }else if ((mGlobalStatus & kGlobalStatusInitError) == 0) {
    if (inMessage.type == CANFDMessage::CAN_REMOTE) {
      sendStatus = tryToSendRemoteFrameFD (inMessage) ;
    }else{
      sendStatus = tryToSendDataFrameFD (inMessage) ;
    }
  }
  return sendStatus ;
}

//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::tryToSendRemoteFrameFD (const CANFDMessage & inMessage) {
  uint32_t sendStatus = 0 ;
  if (mRxCANFDMBCount >= (MBCount (mPayload) - 2)) {
    sendStatus = kNoReservedMBForSendingRemoteFrame ;
  }else{
    bool sent = false ;
    for (uint32_t txMBIndex = mRxCANFDMBCount + 1 ; (txMBIndex < (MBCount (mPayload) - 1)) && !sent ; txMBIndex++) {
      volatile uint32_t * TxMailBoxAddress = mailboxAddress (mFlexcanBaseAddress, mPayload, txMBIndex) ;
      const uint32_t status = (TxMailBoxAddress [0] >> 24) & 0x0F ;
      switch (status) {
      case FLEXCAN_MB_CODE_TX_INACTIVE : // MB has never sent remote frame
      case FLEXCAN_MB_CODE_TX_EMPTY : // MB has sent a remote frame
      case FLEXCAN_MB_CODE_TX_FULL : // MB has sent a remote frame, and received a frame that did not pass any filter
      case FLEXCAN_MB_CODE_TX_OVERRUN : // MB has sent a remote frame, and received frames that did not pass any filter
        writeTxRegistersFD (inMessage, TxMailBoxAddress) ;
        sent = true ;
        break ;
      default:
        break ;
      }
    }
    if (!sent) {
      sendStatus = kNoAvailableMBForSendingRemoteFrame ;
    }
  }
  return sendStatus ;
}

//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::tryToSendDataFrameFD (const CANFDMessage & inMessage) {
  uint32_t sendStatus = 0 ;
  switch (mPayload) {
  case ACAN_T4FD_Settings::PAYLOAD_8_BYTES : // 64 MB, table 44-40 page 2837
    if (inMessage.len > 8) {
      sendStatus = kMessageLengthExceedsPayload ;
    }
    break ;
  case ACAN_T4FD_Settings::PAYLOAD_16_BYTES : // 42 MB, table 44-41 page 2839
    if (inMessage.len > 16) {
      sendStatus = kMessageLengthExceedsPayload ;
    }
    break ;
  case ACAN_T4FD_Settings::PAYLOAD_32_BYTES : // 24 MB, table 44-42 page 2840
    if (inMessage.len > 32) {
      sendStatus = kMessageLengthExceedsPayload ;
    }
    break ;
  case ACAN_T4FD_Settings::PAYLOAD_64_BYTES :  // 14 MB, table 44-43 page 2841
    break ;
  }
  noInterrupts () ;
    if (sendStatus == 0) {
      bool sent = false ;
      const uint32_t TxMailboxIndex = MBCount (mPayload) - 1 ;
      if (mTransmitBufferCount == 0) {
        volatile uint32_t * TxMailBoxAddress = mailboxAddress (mFlexcanBaseAddress, mPayload, TxMailboxIndex) ;
        const uint32_t code = (TxMailBoxAddress [0] >> 24) & 0x0F ;
        if (code == FLEXCAN_MB_CODE_TX_INACTIVE) {
          writeTxRegistersFD (inMessage, TxMailBoxAddress) ;
          sent = true ;
        }
      }
    //--- If no mailboxes available, try to buffer it
      if (!sent) {
        sent = mTransmitBufferCount < mTransmitBufferSize ;
        if (sent) {
          uint32_t transmitBufferWriteIndex = mTransmitBufferReadIndex + mTransmitBufferCount ;
          if (transmitBufferWriteIndex >= mTransmitBufferSize) {
            transmitBufferWriteIndex -= mTransmitBufferSize ;
          }
          mTransmitBufferFD [transmitBufferWriteIndex] = inMessage ;
          mTransmitBufferCount += 1 ;
        //--- Update max count
          if (mTransmitBufferPeakCount < mTransmitBufferCount) {
            mTransmitBufferPeakCount = mTransmitBufferCount ;
          }
        }else{
          mTransmitBufferPeakCount = mTransmitBufferSize + 1 ;
        }
      }
    //---
      if (!sent) {
        sendStatus = kTransmitBufferOverflow ;
      }
    }
  interrupts () ;
//---
  return sendStatus ;
}


//----------------------------------------------------------------------------------------

void ACAN_T4::writeTxRegistersFD (const CANFDMessage & inMessage,
                                  volatile uint32_t * inMBAddress) {
//--- Make Tx box inactive
  inMBAddress [0] = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_INACTIVE) ;
//--- Write identifier
  inMBAddress [1] = inMessage.ext
    ? (inMessage.id & FLEXCAN_MB_ID_EXT_MASK)
    : FLEXCAN_MB_ID_IDSTD (inMessage.id)
  ;
//--- Write data (registers are big endian, values should be swapped)
  for (uint32_t i=0 ; i < dataWordsForPayload (mPayload) ; i++) {
    inMBAddress [i + 2] = __builtin_bswap32 (inMessage.data32 [i]) ;
  }
//--- Send message
  uint32_t lengthCode ;
  if (inMessage.len > 48) {
    lengthCode = 15 ;
  }else if (inMessage.len > 32) {
    lengthCode = 14 ;
  }else if (inMessage.len > 24) {
    lengthCode = 13 ;
  }else if (inMessage.len > 20) {
    lengthCode = 12 ;
  }else if (inMessage.len > 16) {
    lengthCode = 11 ;
  }else if (inMessage.len > 12) {
    lengthCode = 10 ;
  }else if (inMessage.len > 8) {
    lengthCode = 9 ;
  }else{
    lengthCode = inMessage.len ;
  }
  uint32_t command = FLEXCAN_MB_CS_LENGTH (lengthCode) ;
  switch (inMessage.type) {
  case CANFDMessage::CAN_REMOTE :
    command |= FLEXCAN_MB_CS_RTR ;
    break ;
  case CANFDMessage::CAN_DATA :
    break ;
  case CANFDMessage::CANFD_NO_BIT_RATE_SWITCH :
    command |= FLEXCAN_MB_CS_EDL ;
    break ;
  case CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH :
    command |= FLEXCAN_MB_CS_EDL | FLEXCAN_MB_CS_BRS ;
    break ;
  }
  if (inMessage.ext) {
    command |= FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE ;
  }
  command |= FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_ONCE) ;
  inMBAddress [0] = command ;
//--- Workaround for ERR005829 Chip errata (see Chip Errata for the i.MX RT1060, IMXRT1060CE, Rev. 1, 06/2019
  volatile uint32_t * mailBox0Address = mailboxAddress (mFlexcanBaseAddress, mPayload, 0) ;
  mailBox0Address [0] = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_INACTIVE) ;
  mailBox0Address [0] = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_INACTIVE) ;
}

//----------------------------------------------------------------------------------------
//   MESSAGE INTERRUPT SERVICE ROUTINES
//----------------------------------------------------------------------------------------

void ACAN_T4::readRxRegistersFD (CANFDMessage & outMessage,
                                 const uint32_t inReceiveMailboxIndex) {
  volatile uint32_t * RxMailBoxAddress = mailboxAddress (mFlexcanBaseAddress, mPayload, inReceiveMailboxIndex) ;
//--- Wait while MB is busy
  uint32_t controlField = RxMailBoxAddress [0] ;
  while ((controlField & (1 << 24)) != 0) {
    controlField = RxMailBoxAddress [0] ;
  }
//--- Get identifier, ext and len
  const uint32_t lengthCode = FLEXCAN_get_length (controlField) ;
  outMessage.len = CANFD_LENGTH_CODE [lengthCode] ;
  outMessage.ext = (controlField & FLEXCAN_MB_CS_IDE) != 0 ;
//--- Frame format
  if ((controlField & FLEXCAN_MB_CS_RTR) != 0) { // RTR ?
    outMessage.type = CANFDMessage::CAN_REMOTE ;
  }else if ((controlField & FLEXCAN_MB_CS_EDL) == 0) { // No EDL ?
    outMessage.type = CANFDMessage::CAN_DATA ;
  }else if ((controlField & FLEXCAN_MB_CS_BRS) == 0) { // No BRS ?
    outMessage.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ;
  }else{
    outMessage.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
  }
  outMessage.id = RxMailBoxAddress [1] & FLEXCAN_MB_ID_EXT_MASK ;
  if (!outMessage.ext) {
    outMessage.id >>= FLEXCAN_MB_ID_STD_BIT_NO ;
  }
//-- Get data (registers are big endian, values should be swapped)
  for (uint32_t i=0 ; i < dataWordsForPayload (mPayload) ; i++) {
    outMessage.data32 [i] = __builtin_bswap32 (RxMailBoxAddress [i + 2]) ;
  }
//--- Set receive mailbox index (minus one, as Mailbox #0 is unused)
   outMessage.idx = uint8_t (inReceiveMailboxIndex - 1) ;
//--- Make Mailbox ready to receive an other frame
  uint32_t code = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_EMPTY) ;
  if (mCANFDAcceptanceFilterArray != nullptr) {
    const uint32_t acceptanceFilter = mCANFDAcceptanceFilterArray [inReceiveMailboxIndex-1] ;
    RxMailBoxAddress [1] = acceptanceFilter & 0x1FFFFFFF ; // Write MB acceptance filter
    if ((acceptanceFilter & (1 << 31)) != 0) {
      code |= (1 << 20) ; // Filter remote / data
    }
    if ((acceptanceFilter & (1 << 30)) != 0) {
      code |= (1 << 21) ; // Filter standard / extended
    }
  }
  RxMailBoxAddress [0] = code ;
}

//----------------------------------------------------------------------------------------

void ACAN_T4::message_isr_receiveFD (const uint32_t inReceiveMailboxIndex) {
  CANFDMessage message ;
  readRxRegistersFD (message, inReceiveMailboxIndex) ;
  if (mReceiveBufferCount == mReceiveBufferSize) { // Overflow! Receive buffer is full
    mReceiveBufferPeakCount = mReceiveBufferSize + 1 ; // Mark overflow
    mGlobalStatus |= kGlobalStatusReceiveBufferOverflow ;
  }else{
    uint32_t receiveBufferWriteIndex = mReceiveBufferReadIndex + mReceiveBufferCount ;
    if (receiveBufferWriteIndex >= mReceiveBufferSize) {
      receiveBufferWriteIndex -= mReceiveBufferSize ;
    }
    mReceiveBufferFD [receiveBufferWriteIndex] = message ;
    mReceiveBufferCount += 1 ;
    if (mReceiveBufferCount > mReceiveBufferPeakCount) {
      mReceiveBufferPeakCount = mReceiveBufferCount ;
    }
  }
}

//----------------------------------------------------------------------------------------

void ACAN_T4::message_isr_FD (void) {
  uint64_t status = FLEXCAN_IFLAG2 (mFlexcanBaseAddress) ;
  status <<= 32 ;
  status |= FLEXCAN_IFLAG1 (mFlexcanBaseAddress) ;
//--- Frames have been received in Rx mailboxes ?
  uint32_t receiveStatus = status & ((ONE << mRxCANFDMBCount) - ONE) ;
  while (receiveStatus != 0) {
    const uint32_t receiveMailboxIndex = uint32_t (__builtin_ctz (receiveStatus)) ;
    receiveStatus &= ~ (1 << receiveMailboxIndex) ;
    message_isr_receiveFD (receiveMailboxIndex) ;
  }
//--- Tx Mailbox becomes free ?
  const uint32_t TxMailboxIndex = MBCount (mPayload) - 1 ;
  if ((status & (ONE << TxMailboxIndex)) != 0) {
    volatile uint32_t * TxMailBoxAddress = mailboxAddress (mFlexcanBaseAddress, mPayload, TxMailboxIndex) ;
    if (mTransmitBufferCount == 0) {
      TxMailBoxAddress [0] = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_INACTIVE) ; // Inactive MB
    }else{ // There is a frame in the queue to send
      writeTxRegistersFD (mTransmitBufferFD [mTransmitBufferReadIndex], TxMailBoxAddress);
      mTransmitBufferReadIndex = (mTransmitBufferReadIndex + 1) % mTransmitBufferSize ;
      mTransmitBufferCount -= 1 ;
    }
  }
//--- Writing its value back to itself clears all flags
  FLEXCAN_IFLAG1 (mFlexcanBaseAddress) = uint32_t (status) ;
  FLEXCAN_IFLAG2 (mFlexcanBaseAddress) = uint32_t (status >> 32) ;
//--- Read the Free Running Timer (recommended, see page 2704)
  const uint32_t unused __attribute__((unused)) = FLEXCAN_TIMER (mFlexcanBaseAddress) ;
}

//----------------------------------------------------------------------------------------
//    CANFD Filter (format A)
//----------------------------------------------------------------------------------------

static uint32_t defaultMask (const tFrameFormat inFormat) {
  return (inFormat == kExtended) ? 0x1FFFFFFF : 0x7FF ;
}

//----------------------------------------------------------------------------------------

static uint32_t computeFilterMask (const tFrameFormat inFormat,
                                   const uint32_t inMask) {
//--- THIS IS VERY STRANGE!!! Following the reference manual, the mask should be :
//    ((inFormat == kStandard) ? (inMask << 19) : (inMask << 1))
  return
    (1 << 31) | // Test RTR bit
    (1 << 30) | // Test IDE bit
    ((inFormat == kStandard) ? (inMask << 18) : (inMask << 0))
  ;
}

//----------------------------------------------------------------------------------------

static uint32_t computeAcceptanceMask (const tFrameKind inKind,
                                       const tFrameFormat inFormat,
                                       const uint32_t inAcceptance) {
  return
    ((inKind == kRemote) ? (1 << 31) : 0) | // Accepts remote or data frames ?
    ((inFormat == kExtended) ? (1 << 30) : 0) | // Accepts standard or extended frames ?
    ((inFormat == kStandard) ? (inAcceptance << 18) : inAcceptance)
  ;
}

//----------------------------------------------------------------------------------------

ACANFDFilter::ACANFDFilter (const ACANFDCallBackRoutine inCallBackRoutine) :
mFilterMask (0),
mAcceptanceMask (0),
mCallBackRoutine (inCallBackRoutine) {
}

//----------------------------------------------------------------------------------------

ACANFDFilter::ACANFDFilter (const tFrameKind inKind,
                            const tFrameFormat inFormat,
                            const ACANFDCallBackRoutine inCallBackRoutine) :
mFilterMask (computeFilterMask (inFormat, 0)),
mAcceptanceMask (computeAcceptanceMask (inKind, inFormat, 0)),
mCallBackRoutine (inCallBackRoutine) {
}

//----------------------------------------------------------------------------------------

ACANFDFilter::ACANFDFilter (const tFrameKind inKind,
                            const tFrameFormat inFormat,
                            const uint32_t inIdentifier,
                            const ACANFDCallBackRoutine inCallBackRoutine) :
mFilterMask (computeFilterMask (inFormat, defaultMask (inFormat))),
mAcceptanceMask (computeAcceptanceMask (inKind, inFormat, inIdentifier)),
mCallBackRoutine (inCallBackRoutine) {
}

//----------------------------------------------------------------------------------------

ACANFDFilter::ACANFDFilter (const tFrameKind inKind,
                            const tFrameFormat inFormat,
                            const uint32_t inMask,
                            const uint32_t inAcceptance,
                            const ACANFDCallBackRoutine inCallBackRoutine) :
mFilterMask (computeFilterMask (inFormat, inMask)),
mAcceptanceMask (computeAcceptanceMask (inKind, inFormat, inAcceptance)),
mCallBackRoutine (inCallBackRoutine) {
}

//----------------------------------------------------------------------------------------
