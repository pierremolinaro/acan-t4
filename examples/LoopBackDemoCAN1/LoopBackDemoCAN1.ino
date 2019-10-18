// LoopBackDemo for Teensy 4.0 CAN1

// The FlexCAN module is configured in loop back mode:
//   it internally receives every CAN frame it sends.

// No external hardware required.

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
  Serial.println ("CAN1 loopback test") ;
  ACAN_T4_Settings settings (125 * 1000) ; // 125 kbit/s
  settings.mLoopBackMode = true ;
  settings.mSelfReceptionMode = true ;
  const uint32_t errorCode = ACAN_T4::can1.begin (settings) ;
  if (0 == errorCode) {
    Serial.println ("can1 ok") ;
  }else{
    Serial.print ("Error can1: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

static uint32_t gBlinkDate = 0 ;
static uint32_t gSendDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;

//-----------------------------------------------------------------

void loop () {
  if (gBlinkDate <= millis ()) {
    gBlinkDate += 500 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  CANMessage message ;
  if (gSendDate <= millis ()) {
    message.id = 0x542 ;
    const bool ok = ACAN_T4::can1.tryToSend (message) ;
    if (ok) {
      gSendDate += 2000 ;
      gSentCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentCount) ;
    }
  }
  if (ACAN_T4::can1.receive (message)) {
    gReceivedCount += 1 ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedCount) ;
  }
}
