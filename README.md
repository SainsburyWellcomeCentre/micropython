# The SWC MicroPython fork
This fork of the MicroPython project extends the `rp2` port to provide low level functionality required by [MicroHarp](https://github.com/SainsburyWellcomeCentre/microharp). In addition to the features described in the [MicroPython documentation](https://docs.micropython.org/en/v1.18) the fork provides the following modules and class extensions:

## `usbcdc`
**class usbcdc - USB Communications Device Class**  
usbcdc implements the standard UART duplex serial communications protocol using the target's USB port. usbcdc objects can be created and initialised using:
```
from usbcdc import usbcdc
stream = usbcdc(1)                # init with given interface id
```
`class usbcdc(itf_id, ...)`  
Construct a usbcdc object with the given interface id. Set to 1 unless REPL is disabled, in which case 0 may also be used. Additional keyword-only parameters are:
- *timeout* specifies the time to wait for the first character (in ms).
- *timeout_char* specifies the time to wait between characters (in ms).

Note that the line configuration parameters, including baudrate, are defined by the connected USB host. usbcdc objects support the print function, through which these are reported.

A usbcdc object acts like a `stream` object and reading and writing is done using the standard stream methods.

## `harpsync`
**class harpsync - harp synchronisation interface class**  
harpsync implements a [harp synchronisation clock](https://github.com/harp-tech/protocol/blob/master/Synchronization%20Clock%201.0%201.0%2020200712.pdf) slave interface. harpsync objects can be created and initialised using:
```
from harpsync import harpsync
sync = harpsync(0)                # init with given UART id
```
`class harpsync(id, ...)`  
Construct a harpsync object with the given UART id, may be 0 or 1. Additional keyword-only parameters are:
- *tx* specifies the TX pin to use.
- *rx* specifies the RX pin to use.
- *invert* specifies which lines to invert.
- *calib* specifies the latency calibration (in us).

The default latency calibration is 1 s - 240 us = 999760 us. This gives a timing offset with CF harp modules of mean=0 us, S.D.=100 us.

Note that the line configuration parameters, including baudrate, are defined by the protocol. harpsync objects support the print function, through which these are reported.

`harpsync.read()`  
Returns a tuple of integers (seconds, 32 * microseconds) containing the current harp synchronisation time.

`harpsync.write(seconds)`  
Set the harp synchrpnisation time to exactly seconds. This will be automatically overwritten if a harp synchronisation clock device is connected, if not the synchronisation time will free run from the set time.

## `machine`
**class UART – duplex serial communication bus**  
UART has been extended with the following method:

`UART.txwait()`  
Returns when the UART transmit fifo and registers are empty, and the final stop bit has been shifted out. This can be used to enable transceiver control for half-duplex protocols, for example the Dynamixel protocol.
