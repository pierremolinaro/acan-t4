// LoopBackDemo for Teensy 4.0 CAN3 in CANFD mode

// The FlexCAN module is configured in loop back mode:
//   it internally receives every CAN frame it sends.

// No external hardware required.

// The sketch sends random frames as fast as possible, and receives them.

//-----------------------------------------------------------------

#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.0"
#endif

//-----------------------------------------------------------------

#include <ACAN_T4.h>

//-----------------------------------------------------------------

static uint32_t gBlinkDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;
static const uint32_t MAX_SENT_COUNT = 100 * 1000 ;

static const uint32_t RECEIVE_MAILBOX_MAX_COUNT = 62 ;
static uint32_t gReceiveCountPerMailbox [RECEIVE_MAILBOX_MAX_COUNT] ;

//-----------------------------------------------------------------

static void handle_received_message (const CANFDMessage & inMessage) {
  gReceivedCount += 1 ;
  gReceiveCountPerMailbox [inMessage.idx] += 1 ;
}

//-----------------------------------------------------------------

void setup () {
  pinMode (LED_BUILTIN, OUTPUT) ;
  Serial.begin (9600) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  Serial.println ("CAN3FD loopback test") ;
  ACAN_T4FD_Settings settings (1000 * 1000, DataBitRateFactor::x6) ;
  settings.mLoopBackMode = true ;
  settings.mSelfReceptionMode = true ;
//  settings.mPayload = ACAN_T4FD_Settings::PAYLOAD_8_BYTES ;
//  settings.mRxCANFDMBCount = 60 ;
  const ACANFDFilter filters [] = {
    ACANFDFilter (kData, kStandard, 0x123, handle_received_message), // Standard data, identifier == 0x123
    ACANFDFilter (kRemote, kExtended, 0x12345678, handle_received_message), // Extended remote, identifier == 0x12345678
    ACANFDFilter (kData, kStandard, 0x70F, 0x305, handle_received_message), // Standard data, identifier == 0x305, 0x315, ..., 0x3F5
    ACANFDFilter (kData, kStandard, handle_received_message), // Receive all standard data frames
    ACANFDFilter (handle_received_message) // Receive all
  } ;
  const uint32_t errorCode = ACAN_T4::can3.beginFD (settings, filters, 5) ;
  if (0 == errorCode) {
    Serial.println ("can3 ok") ;
  }else{
    Serial.print ("Error can3: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

void loop () {
  if (gBlinkDate <= millis ()) {
    gBlinkDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("Sent : ") ;
    Serial.print (gSentCount) ;
    Serial.print (", received: ") ;
    Serial.print (gReceivedCount) ;
    Serial.print (" [") ;
    uint32_t totalReceived = 0 ;
    for (uint32_t i=0 ; i<ACAN_T4::can3.RxCANFDMBCount () ; i++) {
      if (i > 0) {
        Serial.print (" ") ;
      }
      Serial.print (gReceiveCountPerMailbox [i]) ;
      totalReceived += gReceiveCountPerMailbox [i] ;
    }
    Serial.print ("], total: ") ; Serial.println (totalReceived) ;
  }
  if (gSentCount < MAX_SENT_COUNT) {
    CANFDMessage sentMessageFD ;
    switch (rand () & 0x3) {
    case 0 :
      sentMessageFD.type = CANFDMessage::CAN_REMOTE ;
      sentMessageFD.len = uint8_t (rand () % 8) ;
      break ;
    case 1 :
       sentMessageFD.type = CANFDMessage::CAN_DATA ;
       sentMessageFD.len = uint8_t (rand () % 8) ;
      break ;
    case 2 :
      sentMessageFD.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
      sentMessageFD.len = uint8_t (rand () % 65) ;
      break ;
    default :
      sentMessageFD.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ;
      sentMessageFD.len = uint8_t (rand () % 65) ;
      break ;
    }
    sentMessageFD.ext = (rand () & 0x1) == 0 ;
    sentMessageFD.id = rand () & (sentMessageFD.ext ? 0x1FFFFFFF : 0x7FF) ;
    sentMessageFD.pad () ;
    if (sentMessageFD.type != CANFDMessage::CAN_REMOTE) {
      for (uint8_t i=0 ; i < sentMessageFD.len ; i++) {
        sentMessageFD.data [i] = uint8_t (rand ()) ;
      }
    }
    const bool ok = ACAN_T4::can3.tryToSendFD (sentMessageFD) ;
    if (ok) {
      gSentCount += 1 ;
    }
  }
  ACAN_T4::can3.dispatchReceivedMessageFD () ;
  CANFDMessage messageFD ;
  while (ACAN_T4::can3.receiveFD (messageFD)) {
    gReceivedCount += 1 ;
    gReceiveCountPerMailbox [messageFD.idx] += 1 ;
  }
}

//-----------------------------------------------------------------
