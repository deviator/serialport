///
module serialport.fiberready;

import serialport.base;

import std.traits : isSomeFunction,
                    FunctionAttribute,
                    functionAttributes;

/++ Serial Port Fiber Ready
 +/
class SerialPortFR : SerialPort
{
protected:

    /++ Preform pause

        If sleepFunc isn't null call it. Else use `Thread.sleep` or
        `Fiber.yield` if code executes in fiber.

        Params:
            dt = sleep time
     +/
    void sleep(Duration dt) @nogc
    {
        if (_sleepFunc is null) msleep(dt);
        else _sleepFunc(dt);
    }

    /++ Calc pause for sleep in read and write loops
     +/
    Duration ioPause() @nogc
    {
        auto cfg = config;
        auto cnt = 1 + // start bit
                   cast(int)cfg.dataBits +
                   (cfg.parity == Parity.none ? 0 : 1) +
                   (cfg.stopBits == StopBits.one ? 1 :
                    cfg.stopBits == StopBits.onePointFive ? 1.5 : 2) +
                    1.5 // reserve
                    ;
        return (cast(ulong)(cnt / cfg.baudRate * 1e6) + 100/+reserve+/).usecs;
    }

    void delegate(Duration) @nogc _sleepFunc;

public:
    /// assume @nogc
    deprecated
    alias SleepFunc = void delegate(Duration);

    ///
    alias SleepFuncNoGC = void delegate(Duration) @nogc;

    /// extended delegate for perform sleep
    deprecated("sleep function must be @nogc")
    void sleepFunc(SleepFunc dlg) @property
    { _sleepFunc = cast(void delegate(Duration) @nogc)dlg; }

    ///
    deprecated("sleep function must be @nogc")
    void sleepFunc(void function(Duration) fnc) @property
    { _sleepFunc = (d){ (cast(void function(Duration) @nogc)fnc)(d); }; }

    /// extended delegate for perform sleep
    void sleepFunc(void delegate(Duration) @nogc dlg) @property
    { _sleepFunc = dlg; }

    /// ditto
    void sleepFunc(void function(Duration) @nogc fnc) @property
    { _sleepFunc = (d){ fnc(d); }; }

    /// ditto
    SleepFuncNoGC sleepFunc() @property { return _sleepFunc; }

    /++ Construct SerialPortFR

        See_Also: SerialPort.this
     +/
    deprecated("sleep function must be @nogc")
    this(F=SleepFunc)(string exmode, F sf)
        if (isSomeFunction!F && !(functionAttributes!F & FunctionAttribute.nogc))
    { sleepFunc = sf; super(exmode); }

    /// ditto
    deprecated("sleep function must be @nogc")
    this(F=SleepFunc)(string port, string mode, F sf)
        if (isSomeFunction!F && !(functionAttributes!F & FunctionAttribute.nogc))
    { sleepFunc = sf; super(port, mode); }

    /// ditto
    deprecated("sleep function must be @nogc")
    this(F=SleepFunc)(string port, uint baudRate, F sf)
        if (isSomeFunction!F && !(functionAttributes!F & FunctionAttribute.nogc))
    { sleepFunc = sf; super(port, baudRate); }

    /// ditto
    deprecated("sleep function must be @nogc")
    this(F=SleepFunc)(string port, uint baudRate, string mode, F sf)
        if (isSomeFunction!F && !(functionAttributes!F & FunctionAttribute.nogc))
    { sleepFunc = sf; super(port, baudRate, mode); }

    /// ditto
    deprecated("sleep function must be @nogc")
    this(F=SleepFunc)(string port, Config conf, F sf)
        if (isSomeFunction!F && !(functionAttributes!F & FunctionAttribute.nogc))
    { sleepFunc = sf; super(port, conf); }

    /// ditto
    this(F=SleepFuncNoGC)(string exmode, F sf=null)
        if (isSomeFunction!F)
    { sleepFunc = sf; super(exmode); }

    /// ditto
    this(F=SleepFuncNoGC)(string port, string mode, F sf=null)
        if (isSomeFunction!F && (functionAttributes!F & FunctionAttribute.nogc))
    { sleepFunc = sf; super(port, mode); }

    /// ditto
    this(F=SleepFuncNoGC)(string port, uint baudRate, F sf=null)
        if (isSomeFunction!F && (functionAttributes!F & FunctionAttribute.nogc))
    { sleepFunc = sf; super(port, baudRate); }

    /// ditto
    this(F=SleepFuncNoGC)(string port, uint baudRate, string mode, F sf=null)
        if (isSomeFunction!F && (functionAttributes!F & FunctionAttribute.nogc))
    { sleepFunc = sf; super(port, baudRate, mode); }

    /// ditto
    this(F=SleepFuncNoGC)(string port, Config conf, F sf=null)
        if (isSomeFunction!F && (functionAttributes!F & FunctionAttribute.nogc))
    { sleepFunc = sf; super(port, conf); }

    override void[] read(void[] buf, CanRead cr=CanRead.allOrNothing)
    {
        if (closed) throwPortClosedException(port);

        size_t res;
        const timeout = buf.length * readTimeoutMult + readTimeout;
        const pause = ioPause();
        const sw = StopWatch(AutoStart.yes);
        while (sw.peek < timeout)
        {
            res += m_read(buf[res..$]).length;
            if (res == buf.length) return buf[];
            this.sleep(pause);
        }

        checkAbility(cr, res, buf.length);

        return buf[0..res];
    }

    override size_t write(string arr)
    {
        if (closed) throwPortClosedException(port);

        size_t written;
        const timeout = arr.length * writeTimeoutMult + writeTimeout;
        const pause = ioPause();
        const sw = StopWatch(AutoStart.yes);
        while (sw.peek < timeout)
        {
            written += m_write(arr[written..$]);
            if (written == arr.length) {
                return written;
            }
            this.sleep(pause);
        }

        throwTimeoutException(port, "write timeout");
        assert(0);
    }

    /++ Read data while available by parts, sleep between checks.

        Sleep time calculates from baud rate and count of bits in one byte.

        -------
        ------|--------|-----|------------|-----|------------> t
            call       |     |            |     |
        readContinues  |     |            |     |
              |        |     |            |     |
              |        |<---------data receive---------->|
              |        |=== =====   ======|     |   |== =| data stream
              |        |     |  |   |     |     |
              |<--timeout--->|  |   |     |     |
              |        |<-1->|  |<2>|     |<-3->|
              |        |                  |     |
              |        |<---readedData--->|     |
              |                               return
              |<-------readAll work time------->|
        
        (1) if readedData.length > 0 then continue reading
            else if expectAnything throw TimeoutException
            else return readedData (empty)
        (2) silent time, if silent < frameGap then continue reading
        (3) else if silent > frameGap then stop reading
            and return readedData
        -------

        Params:
            buf = buffer for reading
            startTimeout = timeout for first byte recive
            frameGap = detect new data frame by silence period
            expectAnything = function throw exception if no data
                              before startTimeout

        Returns: slice of buf with readed data

        Throws:
            PortClosedException
            ReadException
            TimeoutException

        See_Also: SerialPort.read
     +/
    void[] readContinues(void[] buf, Duration startTimeout=1.seconds,
                                     Duration frameGap=50.msecs,
                                     bool expectAnything=true)
    {
        if (closed) throwPortClosedException(port);

        ptrdiff_t readed;

        auto pause = ioPause();

        StopWatch silence, full;

        full.start();
        while (true)
        {
            const res = m_read(buf[readed..$]).length;

            readed += res;

            // buffer filled
            if (readed == buf.length) return buf[];

            if (res == 0)
            {
                if (readed > 0 && silence.peek > frameGap)
                    return buf[0..readed];

                if (!silence.running) silence.start();
            }
            else
            {
                silence.stop();
                silence.reset();
            }

            if (readed == 0 && full.peek > startTimeout)
            {
                if (expectAnything)
                    throwTimeoutException(port, "read timeout");
                else
                    return buf[0..0];
            }

            this.sleep(pause);
        }
    }
}