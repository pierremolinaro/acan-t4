// LoopBackDemo for Teensy 4.x CAN3 in CANFD mode

// The FlexCAN module is configured in loop back mode:
//   it internally receives every CAN frame it sends.

// No external hardware required.

//-----------------------------------------------------------------

#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.x"
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
  ACAN_T4FD_Settings settings (125 * 1000, DataBitRateFactor::x1) ;
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
static uint32_t gSendDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;
static uint32_t gReceiveErrorCount = 0 ;

static CANFDMessage gSentMessageFD ;
static bool gReadDone = true ;

//-----------------------------------------------------------------

void loop () {
  if (gBlinkDate <= millis ()) {
    gBlinkDate += 500 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  if (gReadDone && (gSendDate <= millis ())) {
    switch (rand () & 0x3) {
    case 0 :
      gSentMessageFD.type = CANFDMessage::CAN_REMOTE ;
      gSentMessageFD.len = uint8_t (rand () % 8) ;
      break ;
    case 1 :
      gSentMessageFD.type = CANFDMessage::CAN_DATA ;
      gSentMessageFD.len = uint8_t (rand () % 8) ;
      break ;
    case 2 :
      gSentMessageFD.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
      gSentMessageFD.len = uint8_t (rand () % 65) ;
      break ;
    default :
      gSentMessageFD.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ;
      gSentMessageFD.len = uint8_t (rand () % 65) ;
      break ;
    }
    gSentMessageFD.id = rand () & 0x7FF ;
    gSentMessageFD.pad () ;
    if (gSentMessageFD.type != CANFDMessage::CAN_REMOTE) {
      for (uint8_t i=0 ; i < gSentMessageFD.len ; i++) {
        gSentMessageFD.data [i] = uint8_t (rand ()) ;
      }
    }
    const bool ok = ACAN_T4::can3.tryToSendFD (gSentMessageFD) ;
    if (ok) {
      gReadDone = false ;
      gSendDate += 2000 ;
      gSentCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentCount) ;
    }
  }
  CANFDMessage messageFD ;
  if (ACAN_T4::can3.receiveFD (messageFD)) {
    gReadDone = true ;
    gReceivedCount += 1 ;
    uint32_t errorCode = 0 ;
    if (gSentMessageFD.id != messageFD.id) {
      errorCode = 1 ;
    }
    if (gSentMessageFD.type != messageFD.type) {
      errorCode |= 1 ;
    }
    if (gSentMessageFD.len != messageFD.len) {
      errorCode |= 4 ;
    }else if (messageFD.type != CANFDMessage::CAN_REMOTE){
      for (uint8_t i = 0 ; i < messageFD.len ; i++) {
        if (gSentMessageFD.data [i] != messageFD.data [i]) {
          errorCode |= 8 ;
        }
      }
    }
    Serial.print ("Received: ") ;
    Serial.print (gReceivedCount) ;
    if (errorCode == 0) {
      Serial.print (", ok, total receive errors: ") ;
      Serial.println (gReceiveErrorCount) ;
    }else{
      gReceiveErrorCount += 1 ;
      Serial.print (", error 0x") ; Serial.println (errorCode, HEX) ;
      Serial.print ("0x") ; Serial.print (gSentMessageFD.id, HEX) ;
      Serial.print (", ") ; Serial.print (gSentMessageFD.type) ;
      Serial.print (", ") ; Serial.print (gSentMessageFD.len) ;
      if (gSentMessageFD.type != CANFDMessage::CAN_REMOTE) {
        for (uint8_t i = 0 ; i < gSentMessageFD.len ; i++) {
          Serial.print (" ") ; Serial.print (gSentMessageFD.data [i], HEX) ;
         }
      }
      Serial.println () ;
      Serial.print ("0x") ; Serial.print (messageFD.id, HEX) ;
      Serial.print (", ") ; Serial.print (messageFD.type) ;
      Serial.print (", ") ; Serial.print (messageFD.len) ;
      if (messageFD.type != CANFDMessage::CAN_REMOTE) {
        for (uint8_t i = 0 ; i < messageFD.len ; i++) {
          Serial.print (" ") ; Serial.print (messageFD.data [i], HEX) ;
        }
      }
      Serial.println () ;
    }
  }
}

//-----------------------------------------------------------------
