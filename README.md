### SerialPort

Library provides simple work with serial port for Posix and Windows.

Simple usage:

```d
auto com = new SerialPort("/dev/ttyUSB0", 19200);

com.write(someDataArry);

//                        write timeout
com.write(someDataArray, 500.dur!"usecs");

auto res1 = com.read(bufferForReading);

//                                     read timeout     frame end gap
auto res2 = com.read(bufferForReading, 500.dur!"msecs", 20.dur!"msecs");
```

Returned `resN` is slice of `bufferForReading`.

#### Warning: unix systems allow only standart speeds
#### [0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400]

Reading and writing algorithms use `Fiber.yield` if available, or `Thread.yield`
otherwise. If you want redefine this behavior, you can set
`void delegate() yieldFunc` field of `SerialPort` through ctor or directly.

Setting parameters examples:
```d
com.config = SerialPort.Config(9600, Parity.none, DataBits.data8, StopBits.one)
com.set(19200).set(DataBits.data8);
com.stopBits = StopBits.two;
```
