// Demo for Teensy 4.x CAN3 in CANFD mode

// The CAN3 module should be connected to a MCP2517FD or MCP2518FD.
// This demo uses SPI1

// Hardware connections :
//   Teensy           MCP2517FD
//   #27 (SCK1)       #10 (SCK)
//   #26 (MOSI1)      #11 (SDI)
//    #1 (MISO1)      #12 (SDO)
//    #4              #13 (nCS)
//    #5               #4 (INT)

//-----------------------------------------------------------------

#ifndef __IMXRT1062__
  #error "This sketch should be compiled for Teensy 4.x"
#endif

//-----------------------------------------------------------------

#include <ACAN_T4.h>
#include <ACAN2517FD.h>
#include <SPI.h>

//-----------------------------------------------------------------

static const byte MCP2517_CS  = 4 ; // CS input of MCP2517
static const byte MCP2517_INT = 5 ; // INT output of MCP2517

//-----------------------------------------------------------------

static const uint32_t ARBITRATION_BIT_RATE = 1000 * 1000 ;
const DataBitRateFactor DATA_BIT_RATE_FACTOR = DataBitRateFactor::x2 ;

//-----------------------------------------------------------------
//  ACAN2517FD Driver object
//-----------------------------------------------------------------

ACAN2517FD can2517FD (MCP2517_CS, SPI1, MCP2517_INT) ;

//-----------------------------------------------------------------

void setup () {
  pinMode (LED_BUILTIN, OUTPUT) ;
  Serial.begin (9600) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  Serial.println ("CAN3FD/MCP2517FD test") ;
//--- Set up CAN3
  ACAN_T4FD_Settings can3Settings (ARBITRATION_BIT_RATE, DATA_BIT_RATE_FACTOR) ;
//  can3Settings.mSelfReceptionMode = true ;
//  can3Settings.mLoopBackMode = true ;
  can3Settings.mTxPinIsOpenCollector = true ;
  can3Settings.mRxPinConfiguration = ACAN_T4FD_Settings::PULLUP_22k ;
  uint32_t errorCode = ACAN_T4::can3.beginFD (can3Settings) ;
  if (0 == errorCode) {
    Serial.println ("can3 ok") ;
  }else{
    Serial.print ("Error can3: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
//--- Setup MCP2517FD
  ACAN2517FDSettings settings (ACAN2517FDSettings::OSC_40MHz, ARBITRATION_BIT_RATE, DATA_BIT_RATE_FACTOR) ;
  settings.mTXCANIsOpenDrain = true ;
  SPI1.begin () ;
  errorCode = can2517FD.begin (settings, [] { can2517FD.isr () ; }) ;
  if (errorCode == 0) {
   Serial.println ("MCP2517FD ok") ;
  }else{
    Serial.print ("Error MCP2517FD: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//-----------------------------------------------------------------

static uint32_t gBlinkDate = 0 ;

static uint32_t gCan3SentCount = 0 ;
static uint32_t gCan3ReceivedCount = 0 ;

static uint32_t g2517SentCount = 0 ;
static uint32_t g2517ReceivedCount = 0 ;

static const uint32_t SEND_COUNT = 100 * 1000 ;

//-----------------------------------------------------------------

void loop () {
  if (gBlinkDate <= millis ()) {
    gBlinkDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("CAN3FD: ") ;
    Serial.print (gCan3SentCount) ;
    Serial.print (" / ") ;
    Serial.print (gCan3ReceivedCount) ;
    Serial.print (" / 0x") ;
    Serial.print (ACAN_T4::can3.globalStatus (), HEX) ;
    Serial.print (", MCP2517FD: ") ;
    Serial.print (g2517SentCount) ;
    Serial.print (" / ") ;
    Serial.print (g2517ReceivedCount) ;
    Serial.print (" / 0x") ;
    Serial.println (can2517FD.errorCounters (), HEX) ;
   }
  CANFDMessage message ; // By default: standard data CANFD frame, zero length
  if (gCan3SentCount < SEND_COUNT) {
    message.id = uint32_t (rand ()) & 0x7FE ;
    message.len = uint32_t (rand ()) % 65 ;
    message.pad () ;
    message.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH ;
    const bool ok = ACAN_T4::can3.tryToSendFD (message) ;
    if (ok) {
      gCan3SentCount += 1 ;
    }
  }
  if (ACAN_T4::can3.receiveFD (message)) {
    gCan3ReceivedCount += 1 ;
  }
  if (g2517SentCount < SEND_COUNT) {
    message.id = (uint32_t (rand ()) & 0x7FE) | 1  ;
    message.len = uint32_t (rand ()) % 65  ;
    message.pad () ;
    message.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH ;
    const bool ok = can2517FD.tryToSend (message) ;
    if (ok) {
      g2517SentCount += 1 ;
    }
  }
  if (can2517FD.receive (message)) {
    g2517ReceivedCount += 1 ;
  }
}

//-----------------------------------------------------------------
