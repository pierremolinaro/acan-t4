## CAN Library for Teensy 4.0 / 4.1


It handles *Controller Area Network* (CAN) for CAN1, CAN2 and CAN3, and *Controller Area Network with Flexible Data* (CANFD) for CAN3.

### Compatibility with the ACAN2515 and ACAN2517 libraries

This library is fully compatible with the MCP2515 CAN Controller ACAN2515 library https://github.com/pierremolinaro/acan2515 and the MCP2517FD, MCP2518FD CAN Controllers ACAN2517 library https://github.com/pierremolinaro/acan2517, it uses a very similar API and the same `CANMessage` class for handling messages.

### Compatibility with the ACAN2517FD library

This library is fully compatible with the MCP2517FD, MCP2518FD CAN Controllers ACAN2517FD library https://github.com/pierremolinaro/acan2517FD, it uses a very similar API and the same `CANFDMessage` class for handling messages.

### ACAN library description
ACAN is a driver for the three FlexCAN modules of the Teensy 4.0 / 4.1 microcontroller. It supports alternate pins for CAN1.

The driver supports many bit rates, as standard 62.5 kbit/s, 125 kbit/s, 250 kbit/s, 500 kbit/s, and 1 Mbit/s. An efficient CAN bit timing calculator finds settings for them, but also for exotic bit rates as 833 kbit/s. If the wished bit rate cannot be achieved, the `begin` method does not configure the hardware and returns an error code.

> Driver API is fully described by the PDF file in the `ACAN_T4/extras` directory.

### Demo Sketches

> The demo sketches are in the `ACAN_T4/examples` directory.

The `LoopBackDemoCAN1` demo sketch shows how configure the `CAN1`module, and how to send and revceive frames.

Configuration is a four-step operation.

1. Instanciation of the `settings` object : the constructor has one parameter: the wished CAN bit rate. The `settings` is fully initialized.
2. You can override default settings. Here, we set the `mLoopBackMode` and `mSelfReceptionMode` properties to true, enabling to run demo code without any additional hardware (no CAN transceiver needed). We can also for example change the receive buffer size by setting the `mReceiveBufferSize` property.
3. Calling the `begin` method configures the driver and starts CAN bus participation. Any message can be sent, any frame on the bus is received. No default filter to provide.
4. You check the `errorCode` value to detect configuration error(s).

```cpp
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
```

Now, an example of the `loop` function. As we have selected loop back mode, every sent frame is received.

```cpp
static uint32_t gBlinkDate = 0 ;
static uint32_t gSendDate = 0 ;
static uint32_t gSentCount = 0 ;
static uint32_t gReceivedCount = 0 ;

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
```

`CANMessage` is the class that defines a CAN message. The `message` object is fully initialized by the default constructor. Here, we set the `id` to `0x542` for sending a standard data frame, without data, with this identifier.

The `ACAN_T4::can1.tryToSend` tries to send the message. It returns `true` if the message has been sucessfully added to the driver transmit buffer.

The `gSendDate` variable handles sending a CAN message every 2000 ms.

`ACAN_T4::can1.receive` returns `true` if a message has been received, and assigned to the `message`argument.

### Use of Optional Reception Filtering

The hardware defines two kinds of filters: *primary* and *secondary* filters. You can define up to 32 *primary* filters and 96 *secondary* filters.

This an setup example:

```cpp
  ACANSettings settings (125 * 1000) ;
  ...
   const ACANPrimaryFilter primaryFilters [] = {
    ACANPrimaryFilter (kData, kExtended, 0x123456, handle_myMessage_0)
  } ;
  const ACANSecondaryFilter secondaryFilters [] = {
    ACANSecondaryFilter (kData, kStandard, 0x234, handle_myMessage_1),
    ACANSecondaryFilter (kRemote, kStandard, 0x542, handle_myMessage_2)
  } ;
  const uint32_t errorCode = ACAN_T4::can1.begin (settings,
                                               primaryFilters, 
                                               1, // Primary filter array size
                                               secondaryFilters,
                                               2) ; // Secondary filter array size
```
For example, the first filter catches extended data frames, with an identifier equal to `0x123456`. When a such frame is received, the `handle_myMessage_0` function is called. In order to achieve this by-filter dispatching, you should call `ACAN_T4::can1.dispatchReceivedMessage` instead of `ACAN_T4::can1.receive` in the `loop`function:


```cpp
void loop () {
  ACAN_T4::can1.dispatchReceivedMessage () ; // Do not use ACAN_T4::can1.receive any more
  ...
}
```
