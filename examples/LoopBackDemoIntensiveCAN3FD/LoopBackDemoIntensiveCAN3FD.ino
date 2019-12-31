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
  const uint32_t errorCode = ACAN_T4::can3.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("can3 ok") ;
  }else{
    Serial.print ("Error can3: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

static uint32_t gBlinkDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;
static const uint32_t MAX_SENT_COUNT = 100 * 1000 ;

static const uint32_t RECEIVE_MAILBOX_COUNT = 11 ;
static uint32_t gReceiveCountPerMailbox [RECEIVE_MAILBOX_COUNT] ;

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
    for (uint32_t i=0 ; i<RECEIVE_MAILBOX_COUNT ; i++) {
      if (i > 0) {
        Serial.print (" ") ;
      }
      Serial.print (gReceiveCountPerMailbox [i]) ;
    }
    Serial.println ("]") ;
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
    sentMessageFD.id = rand () & 0x7FF ;
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
  CANFDMessage messageFD ;
  while (ACAN_T4::can3.receiveFD (messageFD)) {
    gReceivedCount += 1 ;
    gReceiveCountPerMailbox [messageFD.idx] += 1 ;
  }
}

//-----------------------------------------------------------------
