//----------------------------------------------------------------------------------------
// A Teensy 4.x CAN 2.0B driver
// by Pierre Molinaro
// https://github.com/pierremolinaro/ACAN_T4
//
//----------------------------------------------------------------------------------------
//  Teensy 4.x FlexCAN pins
//
//  FLEXCAN1
//    TX: #22 (default), #11
//    RX: #23 (default), #13
//  FLEXCAN2
//    TX: #1 (default)
//    RX: #0 (default)
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

#define FLEXCAN_MCR(b)                   (*((volatile uint32_t *) ((b)+0x00)))
#define FLEXCAN_CTRL1(b)                 (*((volatile uint32_t *) ((b)+0x04)))
#define FLEXCAN_ECR(b)                   (*((volatile uint32_t *) ((b)+0x1C)))
#define FLEXCAN_ESR1(b)                  (*((volatile uint32_t *) ((b)+0x20)))
#define FLEXCAN_IMASK2(b)                (*((volatile uint32_t *) ((b)+0x24)))
#define FLEXCAN_IMASK1(b)                (*((volatile uint32_t *) ((b)+0x28)))
#define FLEXCAN_IFLAG2(b)                (*((volatile uint32_t *) ((b)+0x2C)))
#define FLEXCAN_IFLAG1(b)                (*((volatile uint32_t *) ((b)+0x30)))
#define FLEXCAN_CTRL2(b)                 (*((volatile uint32_t *) ((b)+0x34)))
#define FLEXCAN_RXFGMASK(b)              (*((volatile uint32_t *) ((b)+0x48)))
#define FLEXCAN_RXFIR(b)                 (*((volatile uint32_t *) ((b)+0x4C)))
#define FLEXCAN_MBn_CS(b, n)             (*((volatile uint32_t *) ((b)+0x80+(n)*16)))
#define FLEXCAN_MBn_ID(b, n)             (*((volatile uint32_t *) ((b)+0x84+(n)*16)))
#define FLEXCAN_MBn_WORD0(b, n)          (*((volatile uint32_t *) ((b)+0x88+(n)*16)))
#define FLEXCAN_MBn_WORD1(b, n)          (*((volatile uint32_t *) ((b)+0x8C+(n)*16)))
#define FLEXCAN_IDAF(b, n)               (*((volatile uint32_t *) ((b)+0xE0+(n)*4)))
#define FLEXCAN_MB_MASK(b, n)            (*((volatile uint32_t *) ((b)+0x880+(n)*4)))

//--- Definitions FLEXCAN_MB_CS
static const uint32_t FLEXCAN_MB_CS_RTR       = 0x00100000 ;
static const uint32_t FLEXCAN_MB_CS_IDE       = 0x00200000 ;
static const uint32_t FLEXCAN_MB_CS_SRR       = 0x00400000 ;
static const uint32_t FLEXCAN_MB_CS_CODE_MASK = 0x0F000000 ;
static const uint32_t FLEXCAN_MB_CS_DLC_MASK  = 0x000F0000 ;

static inline uint32_t FLEXCAN_MB_CS_LENGTH (const uint32_t inLength) { return (inLength & 0xF) << 16 ; }

static inline uint32_t FLEXCAN_MB_CS_CODE (const uint32_t inCode) { return (inCode & 0xF) << 24 ; }

static inline uint32_t FLEXCAN_get_code (const uint32_t inValue) {
  return (inValue & FLEXCAN_MB_CS_CODE_MASK) >> 24 ;
}

static inline uint32_t FLEXCAN_get_length (const uint32_t inValue) {
  return (inValue & FLEXCAN_MB_CS_DLC_MASK) >> 16 ;
}

static const uint32_t FLEXCAN_MB_CODE_TX_FULL     = 0x02 ;

static const uint32_t FLEXCAN_MB_CODE_TX_EMPTY    = 0x04 ;

static const uint32_t FLEXCAN_MB_CODE_TX_OVERRUN  = 0x06 ;

static const uint32_t FLEXCAN_MB_CODE_TX_INACTIVE = 0x08 ;

static const uint32_t FLEXCAN_MB_CODE_TX_ONCE     = 0x0C ;

//--- Definitions for FLEXCAN_MCR
static const uint32_t FLEXCAN_MCR_IRMQ     = 0x00010000 ;

static const uint32_t FLEXCAN_MCR_SRX_DIS  = 0x00020000 ;

static const uint32_t FLEXCAN_MCR_LPM_ACK  = 0x00100000 ;

static const uint32_t FLEXCAN_MCR_FRZ_ACK  = 0x01000000 ;

static const uint32_t FLEXCAN_MCR_SOFT_RST = 0x02000000 ;

static const uint32_t FLEXCAN_MCR_NOT_RDY  = 0x08000000 ;

static const uint32_t FLEXCAN_MCR_HALT     = 0x10000000 ;

static const uint32_t FLEXCAN_MCR_FEN      = 0x20000000 ;

//--- Definitions for FLEXCAN_CTRL
static const uint32_t FLEXCAN_CTRL_LOM = 0x00000008 ;

static const uint32_t FLEXCAN_CTRL_SMP = 0x00000080 ;

static const uint32_t FLEXCAN_CTRL_LPB = 0x00001000 ;

static inline uint32_t FLEXCAN_CTRL_PROPSEG (const uint32_t inPropSeg) { return inPropSeg << 0 ; }

static inline uint32_t FLEXCAN_CTRL_PSEG2 (const uint32_t inPSeg2) { return inPSeg2 << 16 ; }

static inline uint32_t FLEXCAN_CTRL_PSEG1 (const uint32_t inPSeg1) { return inPSeg1 << 19 ; }

static inline uint32_t FLEXCAN_CTRL_RJW (const uint32_t inRJW) { return inRJW << 22 ; }

static inline uint32_t FLEXCAN_CTRL_PRESDIV (const uint32_t inPreDivisor) { return inPreDivisor << 24 ; }

//--- Definitions for FLEXCAN_MB_ID
static const uint32_t FLEXCAN_MB_ID_EXT_MASK = 0x1FFFFFFF ;

static inline uint32_t FLEXCAN_MB_ID_IDSTD (const uint32_t inStandardId) { return (inStandardId & 0x7FF) << 18 ; }

static const uint32_t FLEXCAN_MB_ID_STD_BIT_NO = 18 ;

//----------------------------------------------------------------------------------------
//    CAN Filter
//----------------------------------------------------------------------------------------

static uint32_t defaultMask (const tFrameFormat inFormat) {
  return (inFormat == kExtended) ? 0x1FFFFFFF : 0x7FF ;
}

//----------------------------------------------------------------------------------------

static uint32_t computeFilterMask (const tFrameFormat inFormat,
                                   const uint32_t inMask) {
  return
    (1 << 31) | // Test RTR bit
    (1 << 30) | // Test IDE bit
    ((inFormat == kStandard) ? (inMask << 19) : (inMask << 1)) // Test identifier
  ;
}

//----------------------------------------------------------------------------------------

static uint32_t computeAcceptanceFilter (const tFrameKind inKind,
                                         const tFrameFormat inFormat,
                                         const uint32_t inMask,
                                         const uint32_t inAcceptance) {
  const uint32_t acceptanceConformanceError = (inFormat == kStandard)
    ? (inAcceptance > 0x7FF)
    : (inAcceptance > 0x1FFFFFFF)
  ;
  const uint32_t maskConformanceError = (inFormat == kStandard)
    ? (inMask > 0x7FF)
    : (inMask > 0x1FFFFFFF)
  ;
//--- inMask & inAcceptance sould be equal to inAcceptance
  const uint32_t maskAndAcceptanceCompabilityError = (inMask & inAcceptance) != inAcceptance ;
//---
  return
    ((inKind == kRemote) ? (1 << 31) : 0) | // Accepts remote or data frames ?
    ((inFormat == kExtended) ? (1 << 30) : 0) | // Accepts standard or extended frames ?
    ((inFormat == kStandard) ? (inAcceptance << 19) : (inAcceptance << 1)) |
    acceptanceConformanceError | // Bit 0 is not used by hardware --> we use it for signaling conformance error
    maskConformanceError | maskAndAcceptanceCompabilityError
  ;
}

//----------------------------------------------------------------------------------------

ACANPrimaryFilter::ACANPrimaryFilter (const tFrameKind inKind,
                                      const tFrameFormat inFormat,
                                      const ACANCallBackRoutine inCallBackRoutine) :
mPrimaryFilterMask (computeFilterMask (inFormat, 0)),
mPrimaryAcceptanceFilter (computeAcceptanceFilter (inKind, inFormat, defaultMask (inFormat), 0)),
mCallBackRoutine (inCallBackRoutine) {
}

//----------------------------------------------------------------------------------------

ACANPrimaryFilter::ACANPrimaryFilter (const tFrameKind inKind,
                                      const tFrameFormat inFormat,
                                      const uint32_t inIdentifier,
                                      const ACANCallBackRoutine inCallBackRoutine) :
mPrimaryFilterMask (computeFilterMask (inFormat, (inFormat == kExtended) ? 0x1FFFFFFF : 0x7FF)),
mPrimaryAcceptanceFilter (computeAcceptanceFilter (inKind, inFormat, defaultMask (inFormat), inIdentifier)),
mCallBackRoutine (inCallBackRoutine) {
}

//----------------------------------------------------------------------------------------

ACANPrimaryFilter::ACANPrimaryFilter (const tFrameKind inKind,
                                      const tFrameFormat inFormat,
                                      const uint32_t inMask,
                                      const uint32_t inAcceptance,
                                      const ACANCallBackRoutine inCallBackRoutine) :
mPrimaryFilterMask (computeFilterMask (inFormat, inMask)),
mPrimaryAcceptanceFilter (computeAcceptanceFilter (inKind, inFormat, inMask, inAcceptance)),
mCallBackRoutine (inCallBackRoutine) {
}

//----------------------------------------------------------------------------------------

ACANSecondaryFilter::ACANSecondaryFilter (const tFrameKind inKind,
                                          const tFrameFormat inFormat,
                                          const uint32_t inIdentifier,
                                          const ACANCallBackRoutine inCallBackRoutine) :
mSecondaryAcceptanceFilter (computeAcceptanceFilter (inKind, inFormat, defaultMask (inFormat), inIdentifier)),
mCallBackRoutine (inCallBackRoutine) {
}

//----------------------------------------------------------------------------------------
//    FlexCAN Mailboxes configuration
//----------------------------------------------------------------------------------------

static const uint32_t MB_COUNT = 64 ; // MB count is fixed by hardware
static const uint32_t TX_MAILBOX_INDEX = 63 ;

//----------------------------------------------------------------------------------------
// FlexCAN is configured for FIFO reception (MCR.FEN bit is set)
// The CTRL2.RFFN field defines the number of Rx FIFO filters

// RFFN | MB used by Filters | Rx Individual Masks     | Rx Acceptance Filters | Total Filter count
//    0 |    8 (0 ...  7)    |  8 (RXIMR0 ...  RXIMR7) |  0                    |   8
//    1 |   10 (0 ...  9)    | 10 (RXIMR0 ...  RXIMR9) |  6 (10 ...  15)       |  16
//    2 |   12 (0 ... 11)    | 12 (RXIMR0 ... RXIMR11) | 12 (12 ...  23)       |  24
//    3 |   14 (0 ... 13)    | 14 (RXIMR0 ... RXIMR13) | 18 (14 ...  31)       |  32
//    4 |   16 (0 ... 15)    | 16 (RXIMR0 ... RXIMR15) | 24 (16 ...  39)       |  40
//    5 |   18 (0 ... 17)    | 18 (RXIMR0 ... RXIMR17) | 30 (18 ...  47)       |  48
//    6 |   20 (0 ... 19)    | 20 (RXIMR0 ... RXIMR19) | 36 (20 ...  55)       |  56
//    7 |   22 (0 ... 21)    | 22 (RXIMR0 ... RXIMR21) | 42 (22 ...  63)       |  64
//    8 |   24 (0 ... 23)    | 24 (RXIMR0 ... RXIMR23) | 48 (24 ...  71)       |  72
//    9 |   26 (0 ... 25)    | 26 (RXIMR0 ... RXIMR25) | 54 (26 ...  79)       |  80
//   10 |   28 (0 ... 27)    | 28 (RXIMR0 ... RXIMR27) | 60 (28 ...  87)       |  88
//   11 |   30 (0 ... 29)    | 30 (RXIMR0 ... RXIMR29) | 66 (30 ...  95)       |  96
//   12 |   32 (0 ... 31)    | 32 (RXIMR0 ... RXIMR31) | 72 (32 ... 103)       | 104
//   13 |   34 (0 ... 33)    | 32 (RXIMR0 ... RXIMR31) | 80 (32 ... 111)       | 112
//   14 |   36 (0 ... 35)    | 32 (RXIMR0 ... RXIMR31) | 88 (32 ... 119)       | 120
//   15 |   38 (0 ... 37)    | 32 (RXIMR0 ... RXIMR31) | 96 (32 ... 127)       | 128
//----------------------------------------------------------------------------------------

static const uint32_t RFFN = 15 ;

static const uint32_t MAX_PRIMARY_FILTER_COUNT = (RFFN <= 12) ? (8 + 2 * RFFN) : 32 ;

static const uint32_t MAX_SECONDARY_FILTER_COUNT = (RFFN <= 12) ? (6 * RFFN) : (8 * RFFN - 24) ;

static const uint32_t TOTAL_FILTER_COUNT = MAX_PRIMARY_FILTER_COUNT + MAX_SECONDARY_FILTER_COUNT ;

static const uint32_t FIRST_MB_AVAILABLE_FOR_SENDING = 8 + 2 * RFFN ;


//----------------------------------------------------------------------------------------
//   Interrupt service routine
//----------------------------------------------------------------------------------------

void flexcan_isr_can1 (void) {
  ACAN_T4::can1.message_isr () ;
}

//----------------------------------------------------------------------------------------

void flexcan_isr_can2 (void) {
  ACAN_T4::can2.message_isr () ;
}

//----------------------------------------------------------------------------------------

void flexcan_isr_can3 (void) {
  ACAN_T4::can3.message_isr () ;
}

//----------------------------------------------------------------------------------------
//    Constructor
//----------------------------------------------------------------------------------------

ACAN_T4::ACAN_T4 (const uint32_t inFlexcanBaseAddress,
                  const ACAN_T4_Module inModule) :
mFlexcanBaseAddress (inFlexcanBaseAddress),
mModule (inModule) {
}

//----------------------------------------------------------------------------------------
//    end
//----------------------------------------------------------------------------------------

void ACAN_T4::end (void) {
//---------- Disable NVIC interrupt
  switch (mModule) {
  case ACAN_T4_Module::CAN1 : NVIC_DISABLE_IRQ (IRQ_CAN1) ; break ;
  case ACAN_T4_Module::CAN2 : NVIC_DISABLE_IRQ (IRQ_CAN2) ; break ;
  case ACAN_T4_Module::CAN3 : NVIC_DISABLE_IRQ (IRQ_CAN3) ; break ;
  }
//--- Enter freeze mode
  FLEXCAN_MCR (mFlexcanBaseAddress) |= (FLEXCAN_MCR_HALT);
  while (!(FLEXCAN_MCR (mFlexcanBaseAddress) & FLEXCAN_MCR_FRZ_ACK)) ;
//--- Free receive buffer
  delete [] mReceiveBuffer ; mReceiveBuffer = nullptr ;
  delete [] mReceiveBufferFD ; mReceiveBufferFD = nullptr ;
  mReceiveBufferSize = 0 ;
  mReceiveBufferReadIndex = 0 ;
  mReceiveBufferCount = 0 ;
  mReceiveBufferPeakCount = 0 ;
  mGlobalStatus = 0 ;
//--- Free transmit buffer
  delete [] mTransmitBuffer ; mTransmitBuffer = nullptr ;
  delete [] mReceiveBufferFD ; mReceiveBufferFD = nullptr ;
  mTransmitBufferSize = 0 ;
  mTransmitBufferReadIndex = 0 ;
  mTransmitBufferCount = 0 ;
  mTransmitBufferPeakCount = 0 ;
//--- Free callback function array
  delete [] mCallBackFunctionArray ; mCallBackFunctionArray = nullptr ;
  delete [] mCallBackFunctionArrayFD ; mCallBackFunctionArrayFD = nullptr ;
  mCallBackFunctionArraySize = 0 ;
//--- Free CANFD array
  delete [] mCANFDAcceptanceFilterArray ; mCANFDAcceptanceFilterArray = nullptr ;
}

//----------------------------------------------------------------------------------------
//    begin method
//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::begin (const ACAN_T4_Settings & inSettings,
                         const ACANPrimaryFilter inPrimaryFilters [],
                         const uint32_t inPrimaryFilterCount,
                         const ACANSecondaryFilter inSecondaryFilters [],
                         const uint32_t inSecondaryFilterCount) {
  uint32_t errorCode = inSettings.CANBitSettingConsistency () ; // No error code
//--- No configuration if CAN bit settings are incorrect
  if (!inSettings.mBitSettingOk) {
    errorCode |= kCANBitConfiguration ;
  }
  if (0 == errorCode) {
  //---------- Allocate receive buffer
    mReceiveBufferSize = inSettings.mReceiveBufferSize ;
    mReceiveBuffer = new CANMessage [inSettings.mReceiveBufferSize] ;
  //---------- Allocate transmit buffer
    mTransmitBufferSize = inSettings.mTransmitBufferSize ;
    mTransmitBuffer = new CANMessage [inSettings.mTransmitBufferSize] ;
  //---------- Filter count
    const uint32_t primaryFilterCount = std::min (inPrimaryFilterCount, MAX_PRIMARY_FILTER_COUNT) ;
    const uint32_t secondaryFilterCount = std::min (inSecondaryFilterCount, MAX_SECONDARY_FILTER_COUNT) ;
  //---------- Allocate call back function array
    mCallBackFunctionArraySize = primaryFilterCount + secondaryFilterCount ;
    if (mCallBackFunctionArraySize > 0) {
      mCallBackFunctionArray = new ACANCallBackRoutine [mCallBackFunctionArraySize] ;
      for (uint32_t i=0 ; i<primaryFilterCount ; i++) {
        mCallBackFunctionArray [i] = inPrimaryFilters [i].mCallBackRoutine ;
      }
      for (uint32_t i=0 ; i<secondaryFilterCount ; i++) {
        mCallBackFunctionArray [i + primaryFilterCount] = inSecondaryFilters [i].mCallBackRoutine ;
      }
    }
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
    switch (mModule) {
    case ACAN_T4_Module::CAN1 :
      CCM_CCGR0 |= 0x3C000 ;
      _VectorsRam [16 + IRQ_CAN1] = flexcan_isr_can1 ;
      break ;
    case ACAN_T4_Module::CAN2 :
      CCM_CCGR0 |= 0x3C0000 ;
      _VectorsRam [16 + IRQ_CAN2] = flexcan_isr_can2 ;
      break ;
    case ACAN_T4_Module::CAN3 :
      CCM_CCGR7 |= 0x3C0 ;
      _VectorsRam [16 + IRQ_CAN3] = flexcan_isr_can3 ;
     break ;
    }
  //---------- Enable CAN
    FLEXCAN_MCR (mFlexcanBaseAddress) =
      (1 << 30) | // Enable to enter to freeze mode
      (1 << 23) | // FlexCAN is in supervisor mode
      ((MB_COUNT - 1) << 0) // Mailboxes
    ;
    while (FLEXCAN_MCR(mFlexcanBaseAddress) & FLEXCAN_MCR_LPM_ACK) {}
  //---------- Soft reset
    FLEXCAN_MCR(mFlexcanBaseAddress) |= FLEXCAN_MCR_SOFT_RST;
    while (FLEXCAN_MCR(mFlexcanBaseAddress) & FLEXCAN_MCR_SOFT_RST) {}
  //---------- Wait for freeze ack
    while (!(FLEXCAN_MCR(mFlexcanBaseAddress) & FLEXCAN_MCR_FRZ_ACK)) {}
  //---------- Can settings
    FLEXCAN_MCR (mFlexcanBaseAddress) |=
      (inSettings.mSelfReceptionMode ? 0 : FLEXCAN_MCR_SRX_DIS) | // Disable self-reception ?
      FLEXCAN_MCR_FEN  | // Set RxFIFO mode
      FLEXCAN_MCR_IRMQ   // Enable per-mailbox filtering (§56.4.2)
      | ((MB_COUNT - 1) << 0) // Mailboxes
    ;
  //---------- Can bit timing (CTRL1)
    FLEXCAN_CTRL1 (mFlexcanBaseAddress) =
      FLEXCAN_CTRL_PROPSEG (inSettings.mPropagationSegment - 1) |
      FLEXCAN_CTRL_RJW (inSettings.mRJW - 1) |
      FLEXCAN_CTRL_PSEG1 (inSettings.mPhaseSegment1 - 1) |
      FLEXCAN_CTRL_PSEG2 (inSettings.mPhaseSegment2 - 1) |
      FLEXCAN_CTRL_PRESDIV (inSettings.mBitRatePrescaler - 1) |
      (inSettings.mTripleSampling ? FLEXCAN_CTRL_SMP : 0) |
      (inSettings.mLoopBackMode ? FLEXCAN_CTRL_LPB : 0) |
      (inSettings.mListenOnlyMode ? FLEXCAN_CTRL_LOM : 0)
    ;
  //---------- CTRL2
    FLEXCAN_CTRL2 (mFlexcanBaseAddress) =
      (RFFN << 24) | // Number of RxFIFO
      (0x16 << 19) | // TASD: 0x16 is the default value
      (   0 << 18) | // MRP: Matching starts from RxFIFO and continues on mailboxes
      (   1 << 17) | // RRS: Remote request frame is stored
      (   1 << 16)   // EACEN: RTR bit in mask is always compared
    ;
  //---------- Setup RxFIFO filters
  //--- Default mask
    uint32_t defaultFilterMask = 0 ; // By default, accept any frame
    uint32_t defaultAcceptanceFilter = 0 ;
    if (inPrimaryFilterCount > 0) {
      defaultFilterMask = inPrimaryFilters [0].mPrimaryFilterMask ;
      defaultAcceptanceFilter = inPrimaryFilters [0].mPrimaryAcceptanceFilter ;
    }else if (inSecondaryFilterCount > 0) {
      defaultFilterMask = ~1 ;
      defaultAcceptanceFilter = inSecondaryFilters [0].mSecondaryAcceptanceFilter ;
    }
  //--- Setup primary filters (individual filters in FlexCAN vocabulary)
    if (inPrimaryFilterCount > MAX_PRIMARY_FILTER_COUNT) {
      errorCode |= kTooMuchPrimaryFilters ; // Error, too much primary filters
    }
    mActualPrimaryFilterCount = (uint8_t) primaryFilterCount ;
    mMaxPrimaryFilterCount = (uint8_t) MAX_PRIMARY_FILTER_COUNT ;
    for (uint32_t i=0 ; i<primaryFilterCount ; i++) {
      const uint32_t mask = inPrimaryFilters [i].mPrimaryFilterMask ;
      const uint32_t acceptance = inPrimaryFilters [i].mPrimaryAcceptanceFilter ;
      FLEXCAN_MB_MASK (mFlexcanBaseAddress, i) = mask ;
      FLEXCAN_IDAF (mFlexcanBaseAddress, i) = acceptance ;
      if ((acceptance & 1) != 0) {
        errorCode |= kNotConformPrimaryFilter ;
      }
    }
    for (uint32_t i = primaryFilterCount ; i<MAX_PRIMARY_FILTER_COUNT ; i++) {
      FLEXCAN_MB_MASK (mFlexcanBaseAddress, i) = defaultFilterMask ;
      FLEXCAN_IDAF (mFlexcanBaseAddress, i) = defaultAcceptanceFilter ;
    }
  //--- Setup secondary filters (filter mask for Rx individual acceptance filter)
    FLEXCAN_RXFGMASK (mFlexcanBaseAddress) = (inSecondaryFilterCount > 0) ? (~1) : defaultFilterMask ;
    if (inSecondaryFilterCount > MAX_SECONDARY_FILTER_COUNT) {
      errorCode |= kTooMuchSecondaryFilters ;
    }
    for (uint32_t i=0 ; i<secondaryFilterCount ; i++) {
      const uint32_t acceptance = inSecondaryFilters [i].mSecondaryAcceptanceFilter ;
      FLEXCAN_IDAF (mFlexcanBaseAddress, i + MAX_PRIMARY_FILTER_COUNT) = acceptance ;
      if ((acceptance & 1) != 0) { // Bit 0 is the error flag
        errorCode |= kNotConformSecondaryFilter ;
      }
      // Serial.print ("Sec ") ; Serial.print (i) ; Serial.print (" : 0x") ; Serial.println (acceptance, HEX) ;
    }
    for (uint32_t i = MAX_PRIMARY_FILTER_COUNT + secondaryFilterCount ; i<TOTAL_FILTER_COUNT ; i++) {
      FLEXCAN_IDAF (mFlexcanBaseAddress, i) = (inSecondaryFilterCount > 0)
        ? inSecondaryFilters [0].mSecondaryAcceptanceFilter
        : defaultAcceptanceFilter
      ;
    }
  //---------- Make all other MB inactive
    for (uint32_t i = FIRST_MB_AVAILABLE_FOR_SENDING ; i < MB_COUNT ; i++) {
      FLEXCAN_MB_MASK (mFlexcanBaseAddress, i) = 0 ;
      FLEXCAN_MBn_CS (mFlexcanBaseAddress, i) = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_INACTIVE) ;
    }
  //---------- Select TX pin
    uint32_t TxPinConfiguration = IOMUXC_PAD_DSE (inSettings.mTxPinOutputBufferImpedance) ;
    if (inSettings.mTxPinIsOpenCollector) {
      TxPinConfiguration |= IOMUXC_PAD_ODE ;
    }
    // Serial.print ("Tx 0x") ; Serial.println (TxPinConfiguration, HEX) ;
    switch (mModule) {
    case ACAN_T4_Module::CAN1 :
      if ((inSettings.mTxPin == 255) || (inSettings.mTxPin == 22)) {
        CORE_PIN22_CONFIG = 0x12 ; // Pin #22 SION + ALT2
        CORE_PIN22_PADCONFIG = TxPinConfiguration ; // Pin #22
      }else if (inSettings.mTxPin == 11) {
        CORE_PIN11_CONFIG = 0x12 ; // Pin #11 SION + ALT2, IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 10.7.77 page 506
        CORE_PIN11_PADCONFIG = TxPinConfiguration ; // Pin #11
      }else{
        errorCode |= kInvalidTxPin ;
      }
      break ;
    case ACAN_T4_Module::CAN2 :
      if ((inSettings.mTxPin == 255) || (inSettings.mTxPin == 1)) {
        CORE_PIN1_CONFIG = 0x10 ; // Pin #1 SION + ALT0
        CORE_PIN1_PADCONFIG = TxPinConfiguration ; // Pin #1
      }else{
        errorCode |= kInvalidTxPin ;
      }
      break ;
    case ACAN_T4_Module::CAN3 :
      if ((inSettings.mTxPin == 255) || (inSettings.mTxPin == 31)) {
        CORE_PIN31_CONFIG = 0x19 ; // Pin #31 SION + ALT9
        CORE_PIN31_PADCONFIG = TxPinConfiguration ; // Pin #31
      }else{
        errorCode |= kInvalidTxPin ;
      }
      break ;
    }
  //---------- Select RX Pin
    uint32_t RxPinConfiguration = uint32_t (inSettings.mRxPinConfiguration) << 12 ;
    RxPinConfiguration |= IOMUXC_PAD_HYS ; // Hysterisis
    //Serial.print ("Rx 0x") ; Serial.println (RxPinConfiguration, HEX) ;
    switch (mModule) {
    case ACAN_T4_Module::CAN1 :
      if ((inSettings.mRxPin == 255) || (inSettings.mRxPin == 23)) {
        IOMUXC_FLEXCAN1_RX_SELECT_INPUT = 0x02 ; // 10.7.271 page 797
        CORE_PIN23_CONFIG = 0x12 ; // Pin #23 SION + ALT2
        CORE_PIN23_PADCONFIG = RxPinConfiguration ; // Pin #23
      }else if (inSettings.mRxPin == 13) {
        IOMUXC_CANFD_IPP_IND_CANRX_SELECT_INPUT = 0x03 ;// 10.7.271 page 797
        CORE_PIN13_CONFIG = 0x12 ; // Pin #13 SION + ALT2
        CORE_PIN13_PADCONFIG = RxPinConfiguration ; // Pin #13
      }else{
        errorCode |= kInvalidRxPin ;
      }
      break ;
    case ACAN_T4_Module::CAN2 :
      if ((inSettings.mRxPin == 255) || (inSettings.mRxPin == 0)) {
        IOMUXC_FLEXCAN2_RX_SELECT_INPUT = 0x01 ;
        CORE_PIN0_CONFIG = 0x10 ; // Pin #0 SION + ALT0
        CORE_PIN0_PADCONFIG = RxPinConfiguration ; // Pin #0
      }else{
        errorCode |= kInvalidRxPin ;
      }
      break ;
    case ACAN_T4_Module::CAN3 :
      if ((inSettings.mRxPin == 255) || (inSettings.mRxPin == 30)) {
        IOMUXC_CANFD_IPP_IND_CANRX_SELECT_INPUT = 0x00 ;
        CORE_PIN30_CONFIG = 0x19 ; // Pin #30 SION + ALT9
        CORE_PIN30_PADCONFIG = RxPinConfiguration ; // Pin #30
      }else{
        errorCode |= kInvalidRxPin ;
      }
      break ;
    }
  //---------- Start CAN
    FLEXCAN_MCR (mFlexcanBaseAddress) &= ~FLEXCAN_MCR_HALT ;
  //---------- Wait until exit of freeze mode
    while (FLEXCAN_MCR (mFlexcanBaseAddress) & FLEXCAN_MCR_FRZ_ACK) {}
  //----------  Wait until ready
    while (FLEXCAN_MCR (mFlexcanBaseAddress) & FLEXCAN_MCR_NOT_RDY) {}
  //---------- Enable NVIC interrupts
    switch (mModule) {
    case ACAN_T4_Module::CAN1 :
      NVIC_ENABLE_IRQ (IRQ_CAN1) ;
      break ;
    case ACAN_T4_Module::CAN2 :
      NVIC_ENABLE_IRQ (IRQ_CAN2) ;
      break ;
    case ACAN_T4_Module::CAN3 :
      NVIC_ENABLE_IRQ (IRQ_CAN3) ;
      break ;
    }
  //---------- Enable CAN interrupts
    FLEXCAN_IMASK1 (mFlexcanBaseAddress) =
      (1 << 7) | // RxFIFO Overflow
      (1 << 6) | // RxFIFO Warning: number of messages in FIFO goes from 4 to 5
      (1 << 5)   // Frame available in RxFIFO
    ;
    FLEXCAN_IMASK2 (mFlexcanBaseAddress) = (1U << (TX_MAILBOX_INDEX - 32)) ; // (data frame sending)
  }
//---
  mGlobalStatus = (errorCode == 0) ? 0 : kGlobalStatusInitError ;
//--- Return error code (0 --> no error)
  return errorCode ;
}

//----------------------------------------------------------------------------------------
//   RECEPTION
//----------------------------------------------------------------------------------------

bool ACAN_T4::receive (CANMessage & outMessage) {
  noInterrupts () ;
    const bool hasMessage = (mReceiveBufferCount > 0) && ((mGlobalStatus & kGlobalStatusInitError) == 0) ;
    if (hasMessage) {
      outMessage = mReceiveBuffer [mReceiveBufferReadIndex] ;
      mReceiveBufferReadIndex = (mReceiveBufferReadIndex + 1) % mReceiveBufferSize ;
      mReceiveBufferCount -= 1 ;
    }
  interrupts ()
  return hasMessage ;
}

//----------------------------------------------------------------------------------------

bool ACAN_T4::dispatchReceivedMessage (const tFilterMatchCallBack inFilterMatchCallBack) {
  CANMessage receivedMessage ;
  const bool hasReceived = (!mCANFD) && receive (receivedMessage) ;
  if (hasReceived) {
    const uint32_t filterIndex = receivedMessage.idx ;
    if (nullptr != inFilterMatchCallBack) {
      inFilterMatchCallBack (filterIndex) ;
    }
    if (filterIndex < mCallBackFunctionArraySize) {
      ACANCallBackRoutine callBackFunction = mCallBackFunctionArray [filterIndex] ;
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

bool ACAN_T4::tryToSend (const CANMessage & inMessage) {
  return tryToSendReturnStatus (inMessage) == 0 ;
}

//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::tryToSendReturnStatus (const CANMessage & inMessage) {
  uint32_t sendStatus = 0 ;
  if (mCANFD) {
    sendStatus = kFlexCANinCANFDMode ;
  }else if ((mGlobalStatus & kGlobalStatusInitError) == 0) {
    if (inMessage.rtr) { // Remote
      sendStatus = tryToSendRemoteFrame (inMessage) ;
    }else{ // Data
      sendStatus = tryToSendDataFrame (inMessage) ;
    }
  }
  return sendStatus ;
}

//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::tryToSendRemoteFrame (const CANMessage & inMessage) {
  bool sent = false ;
  for (uint32_t index = FIRST_MB_AVAILABLE_FOR_SENDING ; (index < TX_MAILBOX_INDEX) && !sent ; index++) {
    const uint32_t status = FLEXCAN_get_code (FLEXCAN_MBn_CS (mFlexcanBaseAddress, index)) ;
    switch (status) {
    case FLEXCAN_MB_CODE_TX_INACTIVE : // MB has never sent remote frame
    case FLEXCAN_MB_CODE_TX_EMPTY : // MB has sent a remote frame
    case FLEXCAN_MB_CODE_TX_FULL : // MB has sent a remote frame, and received a frame that did not pass any filter
    case FLEXCAN_MB_CODE_TX_OVERRUN : // MB has sent a remote frame, and received several frames that did not pass any filter
      writeTxRegisters (inMessage, index) ;
      sent = true ;
      break ;
    default:
      break ;
    }
  }
  return sent ? 0 : kNoAvailableMBForSendingRemoteFrame ;
}

//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::tryToSendDataFrame (const CANMessage & inMessage) {
  bool sent = false ;
  noInterrupts () ;
  //--- Find an available mailbox
    if (mTransmitBufferCount == 0) {
      const uint32_t code = FLEXCAN_get_code (FLEXCAN_MBn_CS (mFlexcanBaseAddress, TX_MAILBOX_INDEX)) ;
      if (code == FLEXCAN_MB_CODE_TX_INACTIVE) {
        writeTxRegisters (inMessage, TX_MAILBOX_INDEX) ;
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
        mTransmitBuffer [transmitBufferWriteIndex] = inMessage ;
        mTransmitBufferCount += 1 ;
      //--- Update max count
        if (mTransmitBufferPeakCount < mTransmitBufferCount) {
          mTransmitBufferPeakCount = mTransmitBufferCount ;
        }
      }else{
        mTransmitBufferPeakCount = mTransmitBufferSize + 1 ;
      }
    }
  interrupts () ;
  return sent ? 0 : kTransmitBufferOverflow ;
}
      //----------------------------------------------------------------------------------------

void ACAN_T4::writeTxRegisters (const CANMessage & inMessage, const uint32_t inMBIndex) {
//--- Make Tx box inactive
  FLEXCAN_MBn_CS (mFlexcanBaseAddress, inMBIndex) = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_INACTIVE) ;
//--- Write identifier
  FLEXCAN_MBn_ID (mFlexcanBaseAddress, inMBIndex) = inMessage.ext
    ? (inMessage.id & FLEXCAN_MB_ID_EXT_MASK)
    : FLEXCAN_MB_ID_IDSTD (inMessage.id)
  ;
//--- Write data (registers are big endian, values should be swapped)
  FLEXCAN_MBn_WORD0 (mFlexcanBaseAddress, inMBIndex) = __builtin_bswap32 (inMessage.data32 [0]) ;
  FLEXCAN_MBn_WORD1 (mFlexcanBaseAddress, inMBIndex) = __builtin_bswap32 (inMessage.data32 [1]) ;
//--- Send message
  const uint8_t length = (inMessage.len <= 8) ? inMessage.len : 8 ;
  uint32_t command = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_ONCE) | FLEXCAN_MB_CS_LENGTH (length) ;
  if (inMessage.rtr) {
    command |= FLEXCAN_MB_CS_RTR ;
  }
  if (inMessage.ext) {
    command |= FLEXCAN_MB_CS_SRR | FLEXCAN_MB_CS_IDE ;
  }
  FLEXCAN_MBn_CS (mFlexcanBaseAddress, inMBIndex) = command ;
}

//----------------------------------------------------------------------------------------
//   MESSAGE INTERRUPT SERVICE ROUTINES
//----------------------------------------------------------------------------------------

void ACAN_T4::readRxRegisters (CANMessage & outMessage) {
//--- Get identifier, ext, rtr and len
  const uint32_t dlc = FLEXCAN_MBn_CS (mFlexcanBaseAddress, 0) ;
  outMessage.len = FLEXCAN_get_length (dlc) ;
  if (outMessage.len > 8) {
    outMessage.len = 8 ;
  }
  outMessage.ext = (dlc & FLEXCAN_MB_CS_IDE) != 0 ;
  outMessage.rtr = (dlc & FLEXCAN_MB_CS_RTR) != 0 ;
  outMessage.id  = FLEXCAN_MBn_ID (mFlexcanBaseAddress, 0) & FLEXCAN_MB_ID_EXT_MASK ;
  if (!outMessage.ext) {
    outMessage.id >>= FLEXCAN_MB_ID_STD_BIT_NO ;
  }
//-- Get data (registers are big endian, values should be swapped)
  outMessage.data32 [0] = __builtin_bswap32 (FLEXCAN_MBn_WORD0 (mFlexcanBaseAddress, 0)) ;
  outMessage.data32 [1] = __builtin_bswap32 (FLEXCAN_MBn_WORD1 (mFlexcanBaseAddress, 0)) ;
//--- Zero unused data entries
  for (uint32_t i = outMessage.len ; i < 8 ; i++) {
    outMessage.data [i] = 0 ;
  }
//--- Get filter index
  outMessage.idx = uint8_t (FLEXCAN_RXFIR (mFlexcanBaseAddress)) ;
  if (outMessage.idx >= MAX_PRIMARY_FILTER_COUNT) {
    outMessage.idx -= MAX_PRIMARY_FILTER_COUNT - mActualPrimaryFilterCount ;
  }
}

//----------------------------------------------------------------------------------------

void ACAN_T4::message_isr_receive (void) {
  CANMessage message ;
  readRxRegisters (message) ;
  if (mReceiveBufferCount == mReceiveBufferSize) { // Overflow! Receive buffer is full
    mReceiveBufferPeakCount = mReceiveBufferSize + 1 ; // Mark overflow
    mGlobalStatus |= kGlobalStatusReceiveBufferOverflow ;
  }else{
    uint32_t receiveBufferWriteIndex = mReceiveBufferReadIndex + mReceiveBufferCount ;
    if (receiveBufferWriteIndex >= mReceiveBufferSize) {
      receiveBufferWriteIndex -= mReceiveBufferSize ;
    }
    mReceiveBuffer [receiveBufferWriteIndex] = message ;
    mReceiveBufferCount += 1 ;
    if (mReceiveBufferCount > mReceiveBufferPeakCount) {
      mReceiveBufferPeakCount = mReceiveBufferCount ;
    }
  }
}

//----------------------------------------------------------------------------------------

void ACAN_T4::message_isr (void) {
  if (mCANFD) {
    message_isr_FD () ;
  }else{
    const uint32_t status1 = FLEXCAN_IFLAG1 (mFlexcanBaseAddress) ;
  //--- A trame has been received in RxFIFO ?
    if ((status1 & (1 << 5)) != 0) {
      message_isr_receive () ;
    }
  //--- RxFIFO warning ? It occurs when the number of messages goes from 4 to 5
    if ((status1 & (1 << 6)) != 0) {
      mGlobalStatus |= kGlobalStatusRxFIFOWarning ;
    }
  //--- RxFIFO Overflow ?
    if ((status1 & (1 << 7)) != 0) {
      mGlobalStatus |= kGlobalStatusRxFIFOOverflow ;
    }
  //--- Writing its value back to itself clears all flags
    FLEXCAN_IFLAG1 (mFlexcanBaseAddress) = status1 ;
  //--- Handle Tx mailbox
    const uint32_t status2 = FLEXCAN_IFLAG2 (mFlexcanBaseAddress) ;
    if ((status2 & (1 << (TX_MAILBOX_INDEX - 32))) != 0) {
      if (mTransmitBufferCount == 0) {
        FLEXCAN_MBn_CS (mFlexcanBaseAddress, TX_MAILBOX_INDEX) = FLEXCAN_MB_CS_CODE (FLEXCAN_MB_CODE_TX_INACTIVE) ; // Inactive MB
      }else{ // There is a frame in the queue to send
        const uint32_t code = FLEXCAN_get_code (FLEXCAN_MBn_CS (mFlexcanBaseAddress, TX_MAILBOX_INDEX));
        if (code == FLEXCAN_MB_CODE_TX_INACTIVE) {
          writeTxRegisters (mTransmitBuffer [mTransmitBufferReadIndex], TX_MAILBOX_INDEX);
          mTransmitBufferReadIndex = (mTransmitBufferReadIndex + 1) % mTransmitBufferSize ;
          mTransmitBufferCount -= 1 ;
        }
      }
      FLEXCAN_IFLAG2 (mFlexcanBaseAddress) = status2 ;
    }
  }
}

//----------------------------------------------------------------------------------------
//   Controller state
//----------------------------------------------------------------------------------------

tControllerState ACAN_T4::controllerState (void) const {
  uint32_t state = (FLEXCAN_ESR1 (mFlexcanBaseAddress) >> 4) & 0x03 ;
//--- Bus-off state is value 2 or value 3
  if (state == 3) {
    state = 2 ;
  }
  return (tControllerState) state ;
}

//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::receiveErrorCounter (void) const {
  return FLEXCAN_ECR (mFlexcanBaseAddress) >> 8 ;
}

//----------------------------------------------------------------------------------------

uint32_t ACAN_T4::transmitErrorCounter (void) const {
//--- In bus-off state, TXERRCNT field of CANx_ECR register does not reflect transmit error count: we force 256
  const tControllerState state = controllerState () ;
  return (state == kBusOff) ? 256 : (FLEXCAN_ECR (mFlexcanBaseAddress) & 0xFF) ;
}

//----------------------------------------------------------------------------------------

void ACAN_T4::resetGlobalStatus (const uint32_t inReset) {
  const uint32_t reset = inReset & ~ 1 ; // Bit 0 (kGlobalStatusInitError) cannot be reseted
  mGlobalStatus &= ~ reset ;
}

//----------------------------------------------------------------------------------------
//    FlexCAN Register access
//----------------------------------------------------------------------------------------

static const uint32_t FLEXCAN1_BASE = 0x401D0000 ;
static const uint32_t FLEXCAN2_BASE = 0x401D4000 ;
static const uint32_t FLEXCAN3_BASE = 0x401D8000 ;

//----------------------------------------------------------------------------------------
//   Driver as global variable
//----------------------------------------------------------------------------------------

ACAN_T4 ACAN_T4::can1 (FLEXCAN1_BASE, ACAN_T4_Module::CAN1) ;
ACAN_T4 ACAN_T4::can2 (FLEXCAN2_BASE, ACAN_T4_Module::CAN2) ;
ACAN_T4 ACAN_T4::can3 (FLEXCAN3_BASE, ACAN_T4_Module::CAN3) ;

//----------------------------------------------------------------------------------------
