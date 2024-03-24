// Receive filter demo for Teensy 4.x

// This demo runs on Teensy 4.x, using CAN1 in looppback mode: no external hardware required.
// It shows how to define reception primary filters (up to 32)
// and reception secondary filters (up to 96)

//-----------------------------------------------------------------

#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.x"
#endif

//-----------------------------------------------------------------

#include <ACAN_T4.h>

//-----------------------------------------------------------------
//   PRIMARY FILTERS
//-----------------------------------------------------------------

static const uint32_t PRIMARY_FILTER_COUNT = 32 ; // Maximum: 32
static const uint32_t SECONDARY_FILTER_COUNT = 96 ; // Max 96
static const uint32_t FILTER_COUNT = PRIMARY_FILTER_COUNT + SECONDARY_FILTER_COUNT ;
static uint32_t gFilterReceptionCount [FILTER_COUNT] ;
static uint32_t gFilterReceptionErrorCount = 0 ;

static void handleFilterReception (const CANMessage & inMessage) {
  if (inMessage.idx < FILTER_COUNT) {
    gFilterReceptionCount [inMessage.idx] += 1 ;
  }else{
    Serial.print ("Filter index error (") ;
    Serial.print (inMessage.idx) ;
    Serial.println (")") ;
    gFilterReceptionErrorCount += 1 ;
  }
}
//-----------------------------------------------------------------

void setup () {
  for (uint32_t i = 0 ; i<FILTER_COUNT ; i++) {
    gFilterReceptionCount [i] = 0 ;
  }
  pinMode (LED_BUILTIN, OUTPUT) ;
  Serial.begin (9600) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  Serial.println ("Receive filters demo") ;
  ACAN_T4_Settings settings (1000 * 1000) ;
  settings.mLoopBackMode = true ; // Loop back mode
  settings.mSelfReceptionMode = true ; // Required for loop back mode
  const ACANPrimaryFilter primaryFilters [PRIMARY_FILTER_COUNT] = {
    ACANPrimaryFilter (kData,   kStandard,    0x00, handleFilterReception), //  0
    ACANPrimaryFilter (kRemote, kStandard,    0x00, handleFilterReception), //  1
    ACANPrimaryFilter (kData,   kExtended,    0x00, handleFilterReception), //  2
    ACANPrimaryFilter (kRemote, kExtended,    0x00, handleFilterReception), //  3
    ACANPrimaryFilter (kData,   kStandard, 1 <<  0, handleFilterReception), //  4
    ACANPrimaryFilter (kRemote, kStandard, 1 <<  0, handleFilterReception), //  5
    ACANPrimaryFilter (kData,   kExtended, 1 <<  0, handleFilterReception), //  6
    ACANPrimaryFilter (kRemote, kExtended, 1 <<  0, handleFilterReception), //  7
    ACANPrimaryFilter (kData,   kStandard, 1 <<  1, handleFilterReception), //  8
    ACANPrimaryFilter (kRemote, kStandard, 1 <<  1, handleFilterReception), //  9
    ACANPrimaryFilter (kData,   kExtended, 1 <<  1, handleFilterReception), // 10
    ACANPrimaryFilter (kRemote, kExtended, 1 <<  1, handleFilterReception), // 11
    ACANPrimaryFilter (kData,   kStandard, 1 <<  2, handleFilterReception), // 12
    ACANPrimaryFilter (kRemote, kStandard, 1 <<  2, handleFilterReception), // 13
    ACANPrimaryFilter (kData,   kExtended, 1 <<  2, handleFilterReception), // 14
    ACANPrimaryFilter (kRemote, kExtended, 1 <<  2, handleFilterReception), // 15
    ACANPrimaryFilter (kData,   kStandard, 1 <<  3, handleFilterReception), // 16
    ACANPrimaryFilter (kRemote, kStandard, 1 <<  3, handleFilterReception), // 17
    ACANPrimaryFilter (kData,   kExtended, 1 <<  3, handleFilterReception), // 18
    ACANPrimaryFilter (kRemote, kExtended, 1 <<  3, handleFilterReception), // 19
    ACANPrimaryFilter (kData,   kStandard, 1 <<  4, handleFilterReception), // 20
    ACANPrimaryFilter (kRemote, kStandard, 1 <<  4, handleFilterReception), // 21
    ACANPrimaryFilter (kData,   kExtended, 1 <<  4, handleFilterReception), // 22
    ACANPrimaryFilter (kRemote, kExtended, 1 <<  4, handleFilterReception), // 23
    ACANPrimaryFilter (kData,   kStandard, 1 <<  5, handleFilterReception), // 24
    ACANPrimaryFilter (kRemote, kStandard, 1 <<  5, handleFilterReception), // 25
    ACANPrimaryFilter (kData,   kExtended, 1 <<  5, handleFilterReception), // 26
    ACANPrimaryFilter (kRemote, kExtended, 1 <<  5, handleFilterReception), // 27
    ACANPrimaryFilter (kData,   kStandard, 1 <<  6, handleFilterReception), // 28
    ACANPrimaryFilter (kRemote, kStandard, 1 <<  6, handleFilterReception), // 29
    ACANPrimaryFilter (kData,   kExtended, 1 <<  6, handleFilterReception), // 30
    ACANPrimaryFilter (kRemote, kExtended, 1 <<  6, handleFilterReception)  // 31
  } ;
  const ACANSecondaryFilter secondaryFilters [SECONDARY_FILTER_COUNT] = {
    ACANSecondaryFilter (kData,   kStandard, 1 <<  7, handleFilterReception), //  0, idx == 32
    ACANSecondaryFilter (kRemote, kStandard, 1 <<  7, handleFilterReception), //  1, idx == 33
    ACANSecondaryFilter (kData,   kExtended, 1 <<  7, handleFilterReception), //  2, idx == 34
    ACANSecondaryFilter (kRemote, kExtended, 1 <<  7, handleFilterReception), //  3, idx == 35
    ACANSecondaryFilter (kData,   kStandard, 1 <<  8, handleFilterReception), //  4, idx == 36
    ACANSecondaryFilter (kRemote, kStandard, 1 <<  8, handleFilterReception), //  5, idx == 37
    ACANSecondaryFilter (kData,   kExtended, 1 <<  8, handleFilterReception), //  6, idx == 38
    ACANSecondaryFilter (kRemote, kExtended, 1 <<  8, handleFilterReception), //  7, idx == 39
    ACANSecondaryFilter (kData,   kStandard, 1 <<  9, handleFilterReception), //  8, idx == 40
    ACANSecondaryFilter (kRemote, kStandard, 1 <<  9, handleFilterReception), //  9, idx == 41
    ACANSecondaryFilter (kData,   kExtended, 1 <<  9, handleFilterReception), // 10, idx == 42
    ACANSecondaryFilter (kRemote, kExtended, 1 <<  9, handleFilterReception), // 11, idx == 43
    ACANSecondaryFilter (kData,   kStandard, 1 << 10, handleFilterReception), // 12, idx == 44
    ACANSecondaryFilter (kRemote, kStandard, 1 << 10, handleFilterReception), // 13, idx == 45
    ACANSecondaryFilter (kData,   kExtended, 1 << 10, handleFilterReception), // 14, idx == 46
    ACANSecondaryFilter (kRemote, kExtended, 1 << 10, handleFilterReception), // 15, idx == 47
    ACANSecondaryFilter (kData,   kExtended, 1 << 11, handleFilterReception), // 16, idx == 48
    ACANSecondaryFilter (kRemote, kExtended, 1 << 11, handleFilterReception), // 17, idx == 49
    ACANSecondaryFilter (kData,   kExtended, 1 << 12, handleFilterReception), // 18, idx == 50
    ACANSecondaryFilter (kRemote, kExtended, 1 << 12, handleFilterReception), // 19, idx == 51
    ACANSecondaryFilter (kData,   kExtended, 1 << 13, handleFilterReception), // 20, idx == 52
    ACANSecondaryFilter (kRemote, kExtended, 1 << 13, handleFilterReception), // 21, idx == 53
    ACANSecondaryFilter (kData,   kExtended, 1 << 14, handleFilterReception), // 22, idx == 54
    ACANSecondaryFilter (kRemote, kExtended, 1 << 14, handleFilterReception), // 23, idx == 55
    ACANSecondaryFilter (kData,   kExtended, 1 << 15, handleFilterReception), // 24, idx == 56
    ACANSecondaryFilter (kRemote, kExtended, 1 << 15, handleFilterReception), // 25, idx == 57
    ACANSecondaryFilter (kData,   kExtended, 1 << 16, handleFilterReception), // 26, idx == 58
    ACANSecondaryFilter (kRemote, kExtended, 1 << 16, handleFilterReception), // 27, idx == 59
    ACANSecondaryFilter (kData,   kExtended, 1 << 17, handleFilterReception), // 28, idx == 60
    ACANSecondaryFilter (kRemote, kExtended, 1 << 17, handleFilterReception), // 29, idx == 61
    ACANSecondaryFilter (kData,   kExtended, 1 << 18, handleFilterReception), // 30, idx == 62
    ACANSecondaryFilter (kRemote, kExtended, 1 << 18, handleFilterReception), // 31, idx == 63
    ACANSecondaryFilter (kData,   kExtended, 1 << 19, handleFilterReception), // 32, idx == 64
    ACANSecondaryFilter (kRemote, kExtended, 1 << 19, handleFilterReception), // 33, idx == 65
    ACANSecondaryFilter (kData,   kExtended, 1 << 20, handleFilterReception), // 34, idx == 66
    ACANSecondaryFilter (kRemote, kExtended, 1 << 20, handleFilterReception), // 35, idx == 67
    ACANSecondaryFilter (kData,   kExtended, 1 << 21, handleFilterReception), // 36, idx == 68
    ACANSecondaryFilter (kRemote, kExtended, 1 << 21, handleFilterReception), // 37, idx == 69
    ACANSecondaryFilter (kData,   kExtended, 1 << 22, handleFilterReception), // 38, idx == 70
    ACANSecondaryFilter (kRemote, kExtended, 1 << 22, handleFilterReception), // 39, idx == 71
    ACANSecondaryFilter (kData,   kExtended, 1 << 23, handleFilterReception), // 40, idx == 72
    ACANSecondaryFilter (kRemote, kExtended, 1 << 23, handleFilterReception), // 41, idx == 73
    ACANSecondaryFilter (kData,   kExtended, 1 << 24, handleFilterReception), // 42, idx == 74
    ACANSecondaryFilter (kRemote, kExtended, 1 << 24, handleFilterReception), // 43, idx == 75
    ACANSecondaryFilter (kData,   kExtended, 1 << 25, handleFilterReception), // 44, idx == 76
    ACANSecondaryFilter (kRemote, kExtended, 1 << 25, handleFilterReception), // 45, idx == 77
    ACANSecondaryFilter (kData,   kExtended, 1 << 26, handleFilterReception), // 46, idx == 78
    ACANSecondaryFilter (kRemote, kExtended, 1 << 26, handleFilterReception), // 47, idx == 79
    ACANSecondaryFilter (kData,   kExtended, 1 << 27, handleFilterReception), // 48, idx == 80
    ACANSecondaryFilter (kRemote, kExtended, 1 << 27, handleFilterReception), // 49, idx == 81
    ACANSecondaryFilter (kData,   kExtended, 1 << 28, handleFilterReception), // 50, idx == 82
    ACANSecondaryFilter (kRemote, kExtended, 1 << 28, handleFilterReception), // 51, idx == 83
    ACANSecondaryFilter (kData,   kExtended, 3 << 11, handleFilterReception), // 52, idx == 84
    ACANSecondaryFilter (kRemote, kExtended, 3 << 11, handleFilterReception), // 53, idx == 85
    ACANSecondaryFilter (kData,   kExtended, 3 << 12, handleFilterReception), // 54, idx == 86
    ACANSecondaryFilter (kRemote, kExtended, 3 << 12, handleFilterReception), // 55, idx == 87
    ACANSecondaryFilter (kData,   kExtended, 3 << 13, handleFilterReception), // 56, idx == 88
    ACANSecondaryFilter (kRemote, kExtended, 3 << 13, handleFilterReception), // 57, idx == 89
    ACANSecondaryFilter (kData,   kExtended, 3 << 14, handleFilterReception), // 58, idx == 90
    ACANSecondaryFilter (kRemote, kExtended, 3 << 14, handleFilterReception), // 59, idx == 91
    ACANSecondaryFilter (kData,   kExtended, 3 << 15, handleFilterReception), // 60, idx == 92
    ACANSecondaryFilter (kRemote, kExtended, 3 << 15, handleFilterReception), // 61, idx == 93
    ACANSecondaryFilter (kData,   kExtended, 3 << 16, handleFilterReception), // 62, idx == 94
    ACANSecondaryFilter (kRemote, kExtended, 3 << 16, handleFilterReception), // 63, idx == 95
    ACANSecondaryFilter (kData,   kExtended, 3 << 17, handleFilterReception), // 64, idx == 96
    ACANSecondaryFilter (kRemote, kExtended, 3 << 17, handleFilterReception), // 65, idx == 97
    ACANSecondaryFilter (kData,   kExtended, 3 << 18, handleFilterReception), // 66, idx == 98
    ACANSecondaryFilter (kRemote, kExtended, 3 << 18, handleFilterReception), // 67, idx == 99
    ACANSecondaryFilter (kData,   kExtended, 3 << 19, handleFilterReception), // 68, idx == 100
    ACANSecondaryFilter (kRemote, kExtended, 3 << 19, handleFilterReception), // 69, idx == 101
    ACANSecondaryFilter (kData,   kExtended, 3 << 20, handleFilterReception), // 70, idx == 102
    ACANSecondaryFilter (kRemote, kExtended, 3 << 20, handleFilterReception), // 71, idx == 103
    ACANSecondaryFilter (kData,   kExtended, 3 << 21, handleFilterReception), // 72, idx == 104
    ACANSecondaryFilter (kRemote, kExtended, 3 << 21, handleFilterReception), // 73, idx == 105
    ACANSecondaryFilter (kData,   kExtended, 3 << 22, handleFilterReception), // 74, idx == 106
    ACANSecondaryFilter (kRemote, kExtended, 3 << 22, handleFilterReception), // 75, idx == 107
    ACANSecondaryFilter (kData,   kExtended, 3 << 23, handleFilterReception), // 76, idx == 108
    ACANSecondaryFilter (kRemote, kExtended, 3 << 23, handleFilterReception), // 77, idx == 109
    ACANSecondaryFilter (kData,   kExtended, 3 << 24, handleFilterReception), // 78, idx == 110
    ACANSecondaryFilter (kRemote, kExtended, 3 << 24, handleFilterReception), // 79, idx == 111
    ACANSecondaryFilter (kData,   kExtended, 3 << 25, handleFilterReception), // 80, idx == 112
    ACANSecondaryFilter (kRemote, kExtended, 3 << 25, handleFilterReception), // 81, idx == 113
    ACANSecondaryFilter (kData,   kExtended, 3 << 26, handleFilterReception), // 82, idx == 114
    ACANSecondaryFilter (kRemote, kExtended, 3 << 26, handleFilterReception), // 83, idx == 115
    ACANSecondaryFilter (kData,   kExtended, 3 << 27, handleFilterReception), // 84, idx == 116
    ACANSecondaryFilter (kRemote, kExtended, 3 << 27, handleFilterReception), // 85, idx == 117
    ACANSecondaryFilter (kData,   kExtended, 7 << 11, handleFilterReception), // 86, idx == 118
    ACANSecondaryFilter (kRemote, kExtended, 7 << 11, handleFilterReception), // 87, idx == 119
    ACANSecondaryFilter (kData,   kExtended, 7 << 12, handleFilterReception), // 88, idx == 120
    ACANSecondaryFilter (kRemote, kExtended, 7 << 12, handleFilterReception), // 89, idx == 121
    ACANSecondaryFilter (kData,   kExtended, 7 << 13, handleFilterReception), // 90, idx == 122
    ACANSecondaryFilter (kRemote, kExtended, 7 << 13, handleFilterReception), // 91, idx == 123
    ACANSecondaryFilter (kData,   kExtended, 7 << 14, handleFilterReception), // 92, idx == 124
    ACANSecondaryFilter (kRemote, kExtended, 7 << 14, handleFilterReception), // 93, idx == 125
    ACANSecondaryFilter (kData,   kExtended, 7 << 15, handleFilterReception), // 94, idx == 126
    ACANSecondaryFilter (kRemote, kExtended, 7 << 15, handleFilterReception), // 95, idx == 127
  } ;
  const uint32_t errorCode = ACAN_T4::can1.begin (settings,
                                                  primaryFilters, PRIMARY_FILTER_COUNT,
                                                  secondaryFilters, SECONDARY_FILTER_COUNT) ;
  if (0 == errorCode) {
    Serial.println ("can1 ok") ;
  }else{
    Serial.print ("Error can1: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

static uint32_t gBlinkDate = 2000 ;
static uint32_t gSendIndex = 0 ;
static const uint32_t FIRST_STEP_SEND_COUNT  = 2047 * 4 ;
static const uint32_t SECOND_STEP_SEND_COUNT = 18 * 2 ;
static const uint32_t THIRD_STEP_SEND_COUNT  = 17 * 2 ;
static const uint32_t FOURTH_STEP_SEND_COUNT = 6 * 2 ;

//-----------------------------------------------------------------

void loop () {
  if (gBlinkDate <= millis ()) {
    gBlinkDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print (gSendIndex) ;
    bool allOk = true ;
    for (uint32_t i = 0 ; i<FILTER_COUNT ; i++) {
      if (gFilterReceptionCount [i] != 1) {
        Serial.print (" ") ;
        Serial.print (i) ;
        Serial.print (":") ;
        Serial.print (gFilterReceptionCount [i]) ;
        allOk = false ;     
      }
    }
    Serial.println (allOk ? " all ok" : "") ;
  }
  CANMessage message ;
//--- Send message
  if (gSendIndex < FIRST_STEP_SEND_COUNT) {
    message.id = gSendIndex / 4 ;
    message.rtr = (gSendIndex & 1) != 0 ;
    message.ext = (gSendIndex & 2) != 0 ;
    const bool ok = ACAN_T4::can1.tryToSend (message) ;
    if (ok) {
      gSendIndex += 1 ;
    }
  }else if (gSendIndex < (FIRST_STEP_SEND_COUNT + SECOND_STEP_SEND_COUNT)) {
    message.id = 1 << (11 + (gSendIndex - FIRST_STEP_SEND_COUNT) / 2) ;
    message.rtr = (gSendIndex & 1) != 0 ;
    message.ext = true ;
    const bool ok = ACAN_T4::can1.tryToSend (message) ;
    if (ok) {
     // Serial.print ("Send ") ; Serial.print (message.id, HEX) ; Serial.print (" ") ; Serial.println (message.rtr) ;
     gSendIndex += 1 ;
    }
  }else if (gSendIndex < (FIRST_STEP_SEND_COUNT + SECOND_STEP_SEND_COUNT + THIRD_STEP_SEND_COUNT)) {
    message.id = 3 << (11 + (gSendIndex - FIRST_STEP_SEND_COUNT - SECOND_STEP_SEND_COUNT) / 2) ;
    message.rtr = (gSendIndex & 1) != 0 ;
    message.ext = true ;
    const bool ok = ACAN_T4::can1.tryToSend (message) ;
    if (ok) {
      // Serial.print ("Send ") ; Serial.print (message.id, HEX) ; Serial.print (" ") ; Serial.println (message.rtr) ;
      gSendIndex += 1 ;
    }    
  }else if (gSendIndex < (FIRST_STEP_SEND_COUNT + SECOND_STEP_SEND_COUNT + THIRD_STEP_SEND_COUNT + FOURTH_STEP_SEND_COUNT)) {
    message.id = 7 << (11 + (gSendIndex - FIRST_STEP_SEND_COUNT - SECOND_STEP_SEND_COUNT - THIRD_STEP_SEND_COUNT) / 2) ;
    message.rtr = (gSendIndex & 1) != 0 ;
    message.ext = true ;
    const bool ok = ACAN_T4::can1.tryToSend (message) ;
    if (ok) {
      // Serial.print ("Send ") ; Serial.print (message.id, HEX) ; Serial.print (" ") ; Serial.println (message.rtr) ;
      gSendIndex += 1 ;
    }    
  }
//--- Received message handling
  ACAN_T4::can1.dispatchReceivedMessage () ;
}
