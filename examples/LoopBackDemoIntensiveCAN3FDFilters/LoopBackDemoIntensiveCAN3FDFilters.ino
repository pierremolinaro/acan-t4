//-----------------------------------------------------------------
// LoopBackDemo for Teensy 4.x CAN3 in CANFD mode
//
// The FlexCAN module is configured in loop back mode:
//   it internally receives every CAN frame it sends.
// No external hardware required.
//-----------------------------------------------------------------
//
// The sketch sends and receives 16384 frames with aff combinations of:
//   - 2048 identifiers, from 0 to 2047 (all standard identifiers)
//   - 4 types: remote, can data, canfd with bit rate switch, canfd without bit rate switch
//   - 2 formats: standard or extended
//
// 8 filters are provided.
// Filter 0 receives standard data frame with identifier equal to 0x123.
//   3 frames are received: can data, canfd with bit rate switch, canfd without bit rate switch.
// Filter 1 receives extended remote frame with identifier equal to 0x678.
//   1 frames is received.
// Filter 2 receives standard data frame with identifier verifying (identifier & 0x70F) == 0x305.
//   So 16 identifiers are accepted: 0x305, 0x315, ..., 0x3E5, 0x3F5
//   16 * 3 frames are received: can data, canfd with bit rate switch, canfd without bit rate switch.
// Filter 3 receives any standard data frame, that are not accepted by a previous filter
//   There are 2048 * 3 = 6144 standard data frames,
//      minus 3 frames received by filter 0
//      minus 48 frames received by filter 2
//    --> filter 3 receives 6093 frames
// Filter 4 receives any standard removed frame, that are not accepted by a previous filter
//   There are 2048 standard remote frames, none is handled by a previous filter
//    --> filter 4 receives 2048 frames
// Filter 5 receives any extended data frame, none is handled by a previous filter
//   There are 2048 * 3 = 6144 extended data frames,
//    --> filter 5 receives 6144 frames
// Filter 6 receives any extended remote frame, but 1 is handled by filter 1
//    --> filter 6 receives 2047 frames
// Filter 7 receives any frame, but all are handled by previous filters
//    --> filter 7 receives no frame
//
//-----------------------------------------------------------------

#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.x"
#endif

//-----------------------------------------------------------------

#include <ACAN_T4.h>

//-----------------------------------------------------------------

static uint32_t gBlinkDate = 0 ;
static uint32_t gReceivedCount = 0 ;
static const uint32_t MAX_SENT_COUNT = 2048 * 2 * 4 ;

static const uint32_t RECEIVE_MAILBOX_MAX_COUNT = 62 ;
static uint32_t gReceiveCountPerMailbox [RECEIVE_MAILBOX_MAX_COUNT] ;

//-----------------------------------------------------------------

static void handle_received_message_filter_0 (const CANFDMessage & inMessage) {
  gReceivedCount += 1 ;
  gReceiveCountPerMailbox [inMessage.idx] += 1 ;
  if (inMessage.idx != 0) {
    Serial.println ("Filter 0 receive idx error") ;
  }
  if (inMessage.type == CANFDMessage::CAN_REMOTE) {
    Serial.println ("Filter 0 receive rtr error") ;
  }
  if (inMessage.ext) {
    Serial.println ("Filter 0 receive ext error") ;
  }
  if (inMessage.id != 0x123) {
    Serial.print ("Filter 0 receive identifier error: 0x") ;
    Serial.println (inMessage.id, HEX) ;
  }
}

//-----------------------------------------------------------------

static void handle_received_message_filter_1 (const CANFDMessage & inMessage) {
  gReceivedCount += 1 ;
  gReceiveCountPerMailbox [inMessage.idx] += 1 ;
  if (inMessage.idx != 1) {
    Serial.println ("Filter 1 receive idx error") ;
  }
  if (inMessage.type != CANFDMessage::CAN_REMOTE) {
    Serial.print ("Filter 1 receive type error: ") ;
    Serial.println (inMessage.type) ;
  }
  if (!inMessage.ext) {
    Serial.println ("Filter 1 receive ext error") ;
  }
  if (inMessage.id != 0x678) {
    Serial.print ("Filter 1 receive identifier error: 0x") ;
    Serial.println (inMessage.id, HEX) ;
  }
}

//-----------------------------------------------------------------

static void handle_received_message_filter_2 (const CANFDMessage & inMessage) {
  gReceivedCount += 1 ;
  gReceiveCountPerMailbox [inMessage.idx] += 1 ;
  if (inMessage.idx != 2) {
    Serial.println ("Filter 2 receive idx error") ;
  }
  if (inMessage.type == CANFDMessage::CAN_REMOTE) {
    Serial.print ("Filter 2 receive type error: ") ;
    Serial.println (inMessage.type) ;
  }
  if (inMessage.ext) {
    Serial.println ("Filter 2 receive ext error") ;
  }
  if ((inMessage.id & 0x70F) != 0x305) {
    Serial.print ("Filter 2 receive identifier error: 0x") ;
    Serial.println (inMessage.id, HEX) ;
  }
}

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
  settings.mPayload = ACAN_T4FD_Settings::PAYLOAD_8_BYTES ;
//  settings.mRxCANFDMBCount = 60 ;
  const ACANFDFilter filters [] = {
    ACANFDFilter (kData, kStandard, 0x123, handle_received_message_filter_0), // #0, Standard data, identifier == 0x123
    ACANFDFilter (kRemote, kExtended, 0x678, handle_received_message_filter_1), // #1, Extended remote, identifier == 0x12345678
    ACANFDFilter (kData, kStandard, 0x70F, 0x305, handle_received_message_filter_2), // #2, Standard data, identifier == 0x305, 0x315, ..., 0x3F5
    ACANFDFilter (kData, kStandard, handle_received_message), // #3, Receive all standard data frames
    ACANFDFilter (kRemote, kStandard, handle_received_message), // #4, Receive all standard remote frames
    ACANFDFilter (kData, kExtended, handle_received_message), // #5, Receive all extended data frames
    ACANFDFilter (kRemote, kExtended, handle_received_message), // #6, Receive all extended remote frames
    ACANFDFilter (handle_received_message) // #7, Receive all: here none, all frames have been accepted
  } ;
  const uint32_t errorCode = ACAN_T4::can3.beginFD (settings, filters, 8) ;
  if (0 == errorCode) {
    Serial.println ("can3 ok") ;
  }else{
    Serial.print ("Error can3: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

static uint32_t gSentFrameDescription = 0 ;

//-----------------------------------------------------------------

void loop () {
  if (gBlinkDate <= millis ()) {
    gBlinkDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("Sent : ") ;
    Serial.print (gSentFrameDescription) ;
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
  if (gSentFrameDescription < MAX_SENT_COUNT) {
    CANFDMessage sentMessageFD ;
    switch (gSentFrameDescription & 0x3) {
    case 0 :
      sentMessageFD.type = CANFDMessage::CAN_REMOTE ;
      break ;
    case 1 :
       sentMessageFD.type = CANFDMessage::CAN_DATA ;
      break ;
    case 2 :
      sentMessageFD.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
      break ;
    default :
      sentMessageFD.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ;
      break ;
    }
    sentMessageFD.ext = (gSentFrameDescription & 0x4) == 0 ;
    sentMessageFD.id = gSentFrameDescription >> 3 ;
    const bool ok = ACAN_T4::can3.tryToSendFD (sentMessageFD) ;
    if (ok) {
      gSentFrameDescription += 1 ;
    }
  }
  ACAN_T4::can3.dispatchReceivedMessageFD () ;
}

//-----------------------------------------------------------------
