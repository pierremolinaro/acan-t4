## CAN Library for Teensy 4.0

It handles *Controller Area Network* (CAN) for CAN1, CAN2 and CAN3, and *Controller Area Network with Flexible Data* (CANFD) for CAN3. **Note CANFD support is experimental.**

### Compatibility with the ACAN2515 and ACAN2517 libraries

This library is fully compatible with the MCP2515 CAN Controller ACAN2515 library [https://github.com/pierremolinaro/acan2515]() and the MCP2517FD, MCP2518FD CAN Controllers ACAN2517 library [https://github.com/pierremolinaro/acan2517](), it uses a very similar API and the same `CANMessage` class for handling messages.

### Compatibility with the ACAN2517FD library

This library is fully compatible with the MCP2517FD, MCP2518FD CAN Controllers ACAN2517FD library [https://github.com/pierremolinaro/acan2517FD](), it uses a very similar API and the same `CANFDMessage` class for handling messages.


## ACAN-T4 library description (CAN mode)
The driver supports many bit rates, as standard 62.5 kbit/s, 125 kbit/s, 250 kbit/s, 500 kbit/s, and 1 Mbit/s. An efficient CAN bit timing calculator finds settings for them, but also for exotic bit rates as 833 kbit/s. If the wished bit rate cannot be achieved, the `begin` method does not configure the hardware and returns an error code.

> Driver API is fully described by the PDF file in the `extras` directory.

### Demo Sketch for CAN

> The demo sketch is in the `examples/LoopBackDemoCAN1` directory.

Configuration is a four-step operation.

1. Instanciation of the `settings` object : the constructor has one parameter: the wished CAN bit rate. The `settings` is fully initialized.
2. You can override default settings. Here, we set the `mLoopBackMode` and `mSelfReceptionMode` properties to true, enabling to run demo code without any additional hardware (no CAN transceiver needed). We can also for example change the receive buffer size by setting the `mReceiveBufferSize` property.
3. Calling the `begin` method configures the driver and starts CAN bus participation. Any message can be sent, any frame on the bus is received. No default filter to provide.
4. You check the `errorCode` value to detect configuration error(s).

```cpp
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

The hardware defines two kinds of filters: *primary* and *secondary* filters. Depending the driver configuration, you can have up to 14 *primary* filters and 18 *secondary* filters.

This an setup example:

```cpp
void setup () {
  ACAN_T4_Settings settings (125 * 1000) ;
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

## ACAN-T4 library description (CANFD mode)

**Only CAN3 supports CANFD.**


The driver supports many arbitration bit rates, as standard 62.5 kbit/s, 125 kbit/s, 250 kbit/s, 500 kbit/s, and 1 Mbit/s. The data bit rate is expressed by a multiplative factor from arbitration bit rate: for example `DATA_BITRATE_x4`. An efficient CAN bit timing calculator finds settings for them, but also for exotic bit arbitration rates as 833 kbit/s. If the wished bit rate cannot be achieved, the `beginFD` method does not configure the hardware and returns an error code.

> Driver API is fully described by the PDF file in the `extras` directory.

### Demo Sketch for CAN

> The demo sketch is in the `examples/LoopBackDemoCAN3FD` directory.

Configuration is a four-step operation.

1. Instanciation of the `settings` object : the constructor has one parameter: the wished CAN bit rate. The `settings` is fully initialized.
2. You can override default settings. Here, we set the `mLoopBackMode` and `mSelfReceptionMode` properties to true, enabling to run demo code without any additional hardware (no CANFD transceiver needed). We can also for example change the receive buffer size by setting the `mReceiveBufferSize` property.
3. Calling the `beginFD` method configures the driver and starts CAN bus participation. Any message can be sent, any frame on the bus is received. No default filter to provide.
4. You check the `errorCode` value to detect configuration error(s).

```cpp
 void setup () {
  pinMode (LED_BUILTIN, OUTPUT) ;
  Serial.begin (9600) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  Serial.println ("CAN3FD loopback test") ;
  ACAN_T4FD_Settings settings (125 * 1000, ACAN_T4FD_Settings::DATA_BITRATE_x4) ;
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
  CANFDMessage message ; // By default: standard data CANFD frame, zero length
  if (gSendDate <= millis ()) {
    message.id = 0x123 ;
    const bool ok = ACAN_T4::can3.tryToSendFD (message) ;
    if (ok) {
      gSendDate += 2000 ;
      gSentCount += 1 ;
      Serial.print ("Sent: ") ;
      Serial.println (gSentCount) ;
    }
  }
  if (ACAN_T4::can3.receiveFD (message)) {
    gReceivedCount += 1 ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedCount) ;
  }
}
```
`CANFDMessage` is the class that defines a CANFD message. The `message` object is fully initialized by the default constructor. Here, we set the `id` to `0x542` for sending a standard data frame with bit rate switch, without data, with this identifier.

The `ACAN_T4::can3.tryToSendFD` tries to send the message. It returns `true` if the message has been sucessfully added to the driver transmit buffer.

The `gSendDate` variable handles sending a CAN message every 2000 ms.

`ACAN_T4::can3.receiveFD` returns `true` if a message has been received, and assigned to the `message`argument.

### Use of Optional Reception Filtering

The hardware defines one kind of CANFD filters. Depending the driver configuration, you can have up to 62 filters. By default, there are 12 filters.

This an setup example (sketch `LoopBackDemoIntensiveCAN3FDFilters`):

```cpp
void setup () {
  pinMode (LED_BUILTIN, OUTPUT) ;
  Serial.begin (9600) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
  Serial.println ("CAN3FD loopback test") ;
  ACAN_T4FD_Settings settings (1000 * 1000, ACAN_T4FD_Settings::DATA_BITRATE_x6) ;
  settings.mLoopBackMode = true ;
  settings.mSelfReceptionMode = true ;
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
```

For example, the first filter catches standard data frames, with an identifier equal to `0x123`. When a such frame is received, the `handle_myMessage_0` function is called. In order to achieve this by-filter dispatching, you should call `ACAN_T4::can3.dispatchReceivedMessageFD` instead of `ACAN_T4::can3.receiveFD` in the `loop`function:


```cpp
void loop () {
  ACAN_T4::can3.dispatchReceivedMessageFD () ; // Do not use ACAN_T4::can3.receiveFD any more
  ...
}
```
