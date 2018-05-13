///
module serialport.nonblock;

import serialport.base;

///
deprecated("use 'SerialPort' instead")
alias SerialPortNonBlk = SerialPort;

/++ Serial Port Fiber Ready
 +/
class SerialPortFR : SerialPortTm
{
protected:

    /++ Preform pause

        If sleepFunc isn't null call it. Else use `Thread.sleep` or
        `Fiber.yield` if code executes in fiber.

        Params:
            dt = sleep time
     +/
    void sleep(Duration dt)
    {
        if (sleepFunc !is null) sleepFunc(dt);
        else
        {
            import core.thread : Fiber, Thread;
            if (Fiber.getThis is null) Thread.sleep(dt);
            else
            {
                const tm = StopWatch(AutoStart.yes);
                while (tm.peek < dt) Fiber.yield();
            }
        }
    }

    /++ Calc pause for sleep in read and write loops
     +/
    Duration ioPause()
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

public:
    /// extended delegate for perform sleep
    void delegate(Duration dt) sleepFunc;

    /++ Construct SerialPortFR using extend mode string.

        Extend mode string must have port name (e.g. "com1" or "/dev/ttyUSB0")

        Example extend mode string: "/dev/ttyUSB0:9600:8N1"

        Params:
            exmode = extend mode string
            slp = sleep delegate

        See_Also: SerialPort.this(string exmode), Config.parse, Config.set(string mode)
     +/
    this(string exmode, void delegate(Duration) slp=null)
    {
        this.sleepFunc = slp;
        super(exmode);
    }

    /++ Params:
            port = port name
            mode = config mode string
            slp = sleep delegate

        See_Also: Config.parse, Config.set(string mode)
     +/
    this(string port, string mode, void delegate(Duration) slp=null)
    { this(port, Config.parse(mode), slp); }

    /++ Params:
            port = port name
            baudRate = baudrate
            slp = sleep delegate
     +/
    this(string port, uint baudRate, void delegate(Duration) slp=null)
    { this(port, Config(baudRate), slp); }

    /++ Params:
            port = port name
            baudRate = baudrate
            mode = config mode string
            slp = sleep delegate

        See_Also: Config.parse, Config.set(string mode)
     +/
    this(string port, uint baudRate, string mode, void delegate(Duration) slp=null)
    { this(port, Config(baudRate).set(mode), slp); }

    /++ Params:
            port = port name
            conf = config of serialport
            slp = sleep delegate
     +/
    this(string port, Config conf, void delegate(Duration) slp=null)
    {
        this.sleepFunc = slp;
        super(port, conf);
    }

    override void[] read(void[] buf)
    {
        if (closed) throw new PortClosedException(port);

        size_t readed;
        const timeout = buf.length * writeTimeoutMult + writeTimeout;
        const pause = ioPause();
        const sw = StopWatch(AutoStart.yes);
        while (sw.peek < timeout)
        {
            readed += super.read(buf[readed..$]).length;
            if (readed == buf.length) return buf[];
            this.sleep(pause);
        }

        version (readAllOrThrow)
            throw new TimeoutException(port);
        else version (readAvailable)
        {
            if (readed == 0)
                throw new TimeoutException(port);
            else return buf[0..readed];
        }
    }

    override ptrdiff_t write(const(void[]) arr)
    {
        if (closed) throw new PortClosedException(port);

        size_t written;
        const timeout = arr.length * writeTimeoutMult + writeTimeout;
        const pause = ioPause();
        const sw = StopWatch(AutoStart.yes);
        while (sw.peek < timeout)
        {
            written += super.write(arr[written..$]);
            if (written == arr.length) return written;
            this.sleep(pause);
        }

        throw new TimeoutException(port);
    }

    ///
    deprecated("use 'readAll' instead")
    alias readLoop = readAll;

    /++ Read data while available by parts, sleep between checks.

        Call `super.read` (non-blocking).

        Sleep time calculates from baud rate and count of bits in one byte.

        -------
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
           |                            return
           |<------readAll work time----->|
        
        (1) if readedData.length > 0 then continue reading
            else throw TimeoutException
        (2) silent time, if silent < frameGap then continue reading
        (3) else if silent > frameGap then stop reading
            and return readedData
        -------

        Params:
            buf = buffer for reading
            timeout = timeout for first byte recive
            frameGap = detect new data frame by silence period

        Returns: slice of buf with readed data

        Throws:
            PortClosedException
            ReadException
            TimeoutException

        See_Also: SerialPort.read
     +/
    void[] readAll(void[] buf, Duration timeout=1.seconds,
                               Duration frameGap=50.msecs)
    {
        if (closed) throw new PortClosedException(port);

        ptrdiff_t readed;

        auto pause = ioPause();

        StopWatch silence, full;

        full.start();
        while (true)
        {
            const res = super.read(buf[readed..$]).length;

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

            if (readed == 0 && full.peek > timeout)
                throw new TimeoutException(port);

            this.sleep(pause);
        }
    }

    /++ Write all data by parts

        call sleep between parts

        Params:
            arr = data for writing
            timeout = time for writing data

        Throws:
            PortClosedException
            WriteException
            TimeoutException if full write time is out

        See_Also: SerialPort.write
     +/
    deprecated("use 'write' instead")
    void writeLoop(const(void[]) arr, Duration timeout=10.msecs)
    {
        writeTimeout = timeout;
        this.write(arr);
    }
}