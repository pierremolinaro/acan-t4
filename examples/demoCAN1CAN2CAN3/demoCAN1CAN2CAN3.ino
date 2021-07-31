// Demo for Teensy 4.x CAN1, CAN2 and CAN3

// Just connect together all CAN pins:
//   - CRX1 (#23), CTX1 (#22), CRX2 (#0), CTX2 (#1), CRX3 (#30), CTX3 (#31)

//-----------------------------------------------------------------
// CAN1, CAN2 and CAN3 are configured in CAN2.0B.
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
  Serial.println ("CAN1-CAN2-CAN3 test") ;
  ACAN_T4_Settings settings (1000 * 1000) ;
  settings.mTxPinIsOpenCollector = true ;
  settings.mRxPinConfiguration = ACAN_T4_Settings::PULLUP_22k ;
  Serial.print ("CAN Root Clock frequency: ") ;
  Serial.print (getCANRootClockFrequency ()) ;
  Serial.println (" Hz") ;
  Serial.print ("CAN Root Clock divisor: ") ;
  Serial.println (getCANRootClockDivisor ()) ;
  Serial.print ("Bitrate prescaler: ") ;
  Serial.println (settings.mBitRatePrescaler) ;
  Serial.print ("Propagation Segment: ") ;
  Serial.println (settings.mPropagationSegment) ;
  Serial.print ("Phase segment 1: ") ;
  Serial.println (settings.mPhaseSegment1) ;
  Serial.print ("Phase segment 2: ") ;
  Serial.println (settings.mPhaseSegment2) ;
  Serial.print ("RJW: ") ;
  Serial.println (settings.mRJW) ;
  Serial.print ("Triple Sampling: ") ;
  Serial.println (settings.mTripleSampling ? "yes" : "no") ;
  Serial.print ("Actual bitrate: ") ;
  Serial.print (settings.actualBitRate ()) ;
  Serial.println (" bit/s") ;
  Serial.print ("Exact bitrate ? ") ;
  Serial.println (settings.exactBitRate () ? "yes" : "no") ;
  Serial.print ("Distance from wished bitrate: ") ;
  Serial.print (settings.ppmFromWishedBitRate ()) ;
  Serial.println (" ppm") ;
  Serial.print ("Sample point: ") ;
  Serial.print (settings.samplePointFromBitStart ()) ;
  Serial.println ("%") ;
  uint32_t errorCode = ACAN_T4::can1.begin (settings) ;
  if (0 == errorCode) {
    Serial.println ("can1 ok") ;
  }else{
    Serial.print ("Error can1: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
  errorCode = ACAN_T4::can2.begin (settings) ;
  if (0 == errorCode) {
    Serial.println ("can2 ok") ;
  }else{
    Serial.print ("Error can2: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
  errorCode = ACAN_T4::can3.begin (settings) ;
  if (0 == errorCode) {
    Serial.println ("can3 ok") ;
  }else{
    Serial.print ("Error can3: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
  delay (2) ;
}

//-----------------------------------------------------------------

static uint32_t gBlinkDate = 0 ;
static uint32_t gSentCount1 = 0 ;
static uint32_t gReceivedCount1 = 0 ;
static uint32_t gSentCount2 = 0 ;
static uint32_t gReceivedCount2 = 0 ;
static uint32_t gSentCount3 = 0 ;
static uint32_t gReceivedCount3 = 0 ;

static const uint32_t SEND_COUNT = 50 * 1000 ;

//-----------------------------------------------------------------

void loop () {
  if (gBlinkDate <= millis ()) {
    gBlinkDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("CAN1: ") ;
    Serial.print (gSentCount1) ;
    Serial.print (" / ") ;
    Serial.print (gReceivedCount1) ;
    Serial.print (" / 0x") ;
    Serial.print (ACAN_T4::can1.globalStatus (), HEX) ;
    Serial.print (" / ") ;
    Serial.print (ACAN_T4::can1.receiveBufferPeakCount ()) ;
    Serial.print (", CAN2: ") ;
    Serial.print (gSentCount2) ;
    Serial.print (" / ") ;
    Serial.print (gReceivedCount2) ;
    Serial.print (" / 0x") ;
    Serial.print (ACAN_T4::can2.globalStatus (), HEX) ;
    Serial.print (" / ") ;
    Serial.print (ACAN_T4::can2.receiveBufferPeakCount ()) ;
    Serial.print (", CAN3: ") ;
    Serial.print (gSentCount3) ;
    Serial.print (" / ") ;
    Serial.print (gReceivedCount3) ;
    Serial.print (" / 0x") ;
    Serial.print (ACAN_T4::can3.globalStatus (), HEX) ;
    Serial.print (" / ") ;
    Serial.println (ACAN_T4::can3.receiveBufferPeakCount ()) ;
  }
  CANMessage message ;
//  message.rtr = true ;
//--- CAN3 send
  if (gSentCount3 < SEND_COUNT) {
    message.id = 0 ; //(micros () % 682) * 3 + 0 ;
    const bool ok = ACAN_T4::can3.tryToSend (message) ;
    if (ok) {
      gSentCount3 += 1 ;
    }
  }
//--- CAN1 send
  if (gSentCount1 < SEND_COUNT) {
    message.id = 1 ; // (micros () % 682) * 3 + 1 ;
    const bool ok = ACAN_T4::can1.tryToSend (message) ;
    if (ok) {
      gSentCount1 += 1 ;
    }
  }
//--- CAN2 send
  if (gSentCount2 < SEND_COUNT) {
    message.id = 2 ; // (micros () % 682) * 3 + 2 ;
    const bool ok = ACAN_T4::can2.tryToSend (message) ;
    if (ok) {
      gSentCount2 += 1 ;
    }
  }
//--- CAN1 receive
  if (ACAN_T4::can1.receive (message)) {
    gReceivedCount1 += 1 ;
  }
//--- CAN2 receive
  if (ACAN_T4::can2.receive (message)) {
    gReceivedCount2 += 1 ;
  }
//--- CAN3 receive
  if (ACAN_T4::can3.receive (message)) {
    gReceivedCount3 += 1 ;
  }
}
