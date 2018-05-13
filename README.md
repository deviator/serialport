# SerialPort

[![Build Status](https://travis-ci.org/deviator/serialport.svg?branch=master)](https://travis-ci.org/deviator/serialport)
[![Build status](https://ci.appveyor.com/api/projects/status/64m852qc8j3re1y1?svg=true)](https://ci.appveyor.com/project/deviator/serialport)
[![codecov](https://codecov.io/gh/deviator/serialport/branch/master/graph/badge.svg)](https://codecov.io/gh/deviator/serialport)
[![Dub](https://img.shields.io/dub/v/serialport.svg)](http://code.dlang.org/packages/serialport)
[![Downloads](https://img.shields.io/dub/dt/serialport.svg)](http://code.dlang.org/packages/serialport)
[![License](https://img.shields.io/dub/l/serialport.svg)](http://code.dlang.org/packages/serialport)

[Docs](http://serialport.dpldocs.info/serialport.html) generated by [adrdox](https://github.com/adamdruppe/adrdox).

Library provides versatile work with serial port for Linux, Windows and MacOS.

## Simple usage

```d
auto com = new SerialPortBlk("/dev/ttyUSB0", 19200);

// setting parameters example
com.config = SPConfig(9600, DataBits.data8, Parity.none, StopBits.one)
com.set(19200).set(DataBits.data8);
com.stopBits = StopBits.two;
// set 9600 baudrate, 8 data bits, no parity and one stop bit
com.set("9600:8N1");

auto cnt = com.write(someDataArray);
// cnt must be equal someDataArray.length

// res1 is slice of bufferForReading with readed data
auto res1 = com.read(bufferForReading);
```

See also example: [monitor](example/monitor).

Class `SerialPort` provides non-blocking `read` (immediatlly return
data in system serial port buffer) and `write` (return writed bytes
count at the first onset).

Class `SerialPortBlk` provides blocking `read` and `write`.

If you want use library in fibers it provides `SerialPortFR` (Fiber Ready),
where `read` and `write` is loops with calling sleep function. Loops algorithms
use `Fiber.yield` if available, or `Thread.yield` as failback. If you want
redefine this behavior, you can set `void delegate(Duration) sleepFunc` field
directly or through last parameter of ctor.

`write` method of `SerialPortBlk` and `SerialPortFR` can throw `TimeoutException`
if it can't finish write all data to serial port during
`timeout = writeTimeout + writeTimeoutMult * data.length`.

`read` method in `SerialPortBlk` and `SerialPortFR` can throw `TimeoutException`.

Also, `read` method of thouse can throw `TimeoutException`, but here the
behavior can be different. See [library configuration](#library-configurations).

## Library configurations

Receive data time schema:

```
---|-------|--------------|-------|--> t
 call      |              |       |
 read      |              |       |
   |       |<----data receive---->|
   |       |=====   ====  | ======|
   |       |              |
   |       |<-readedData->|
   |                      |
   |<---readTimeoutSum--->|
   |                   return
   |<---read work time--->|
```

where `readTimeoutSum = readTimeout + readTimeoutMult * dataBuffer.length;`

### `readAvailable` (default configuration)

```d
if (readedData.length == 0)
    throw TimeoutException(port);
```

### `readAllOrThrow`

```d
if (readedData.length < dataBuffer.length)
    throw TimeoutException(port);
```

For using this configuration set in your `dub.sdl`
`subConfiguration "serialport" "readAllOrThrow"`

## `SerialPortFR.readAll` method

Undepend of configuration `SerialPortFR` has `readAll` method

    void[] readAll(void[] arr, Duration timeout=1.seconds, Duration frameGap=50.msecs)

It reads in loop from serial port while silent time is less what `frameGap` and
throws `TimeoutException` only if timeout is expires and no data was readed.

```
---|-----|-----|------------|-----|------------> t
 call    |     |            |     |
readAll  |     |            |     |
   |     |     |            |     |
   |     |<---------data receive---------->|
   |     |=== =====   ======|     |   |== =|
   |     |     |  |   |     |     |
   |<-timeout->|  |   |     |     |
   |     |<-1->|  |<2>|     |<-3->|
   |     |                  |     |
   |     |<---readedData--->|     |
   |                           return
   |<------readAll work time----->|

(1) if readedData.length > 0 then continue reading
    else throw TimeoutException
(2) silent time, if silent < frameGap then continue reading
(3) else if silent > frameGap then stop reading
    and return readedData
```

It's useful if you don't know how much data can come:

* allocate buffer for reading (4kB for example)
* call `readAll` and get data frame

## Warning

**unix systems allow only standard speeds:**

**[0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400]**

At expiration of read timeout throws `TimeoutException` if no bytes readed.
If readed bytes count != 0 wait frame end gap and if no new bytes return readed.

Reading and writing loops algorithms use `Fiber.yield` if available,
or `Thread.yield` otherwise. If you want redefine this behavior, you can set
`void delegate() yieldFunc` field of `SerialPort` through ctor or directly.

## Tests

### Real hardware

Two paired USB->UART (FTDI FT232RL) uses for tests on linux and windows.

### CI

For linux and OSX tested (socat as tty pipe creator)

* ldc
* ldc-beta
* ldc-1.8.0
* dmd
* dmd-nightly
* dmd-2.079.1
* dmd-2.078.3

For windows tested (build only, see [note](#note)) fox x86 and x64

* dmd beta
* dmd stable
* ldc beta
* ldc stable

See [.travis.yml](.travis.yml) [.appveyor.yml](.appveyor.yml)

### NOTE

1. Windows not full tested by CI (no real test with virtual com ports)
    because I did not configure to adjust the work com0com program 
    https://help.appveyor.com/discussions/questions/427-how-can-i-use-com0com 