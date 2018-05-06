///
module serialport.nonblock;

import serialport.base;

///
class SerialPortNonBlk : SerialPort
{
public:
    /++ Construct SerialPortNonBlk

        See_Also: SerialPort.this
     +/
    this(string exmode) { super(exmode); }

    /// ditto
    this(string port, string mode) { super(port, mode); }

    /// ditto
    this(string port, uint baudRate) { super(port, baudRate); }

    /// ditto
    this(string port, uint baudRate, string mode)
    { super(port, baudRate, mode); }

    /// ditto
    this(string port, Config conf) { super(port, conf); }

    override void[] read(void[] buf)
    {
        if (closed) throw new PortClosedException(port);

        auto ptr = buf.ptr;
        auto len = buf.length;

        size_t res;

        version (Posix)
        {
            auto sres = posixRead(_handle, ptr, len);

            // no bytes for read, it's ok
            if (sres < 0)
            {
                if (errno == EAGAIN) sres = 0;
                else throw new ReadException(port, text("errno ", errno));
            }
            res = sres;
        }
        version (Windows)
        {
            uint sres;
            auto rfr = ReadFile(_handle, ptr, cast(uint)len, &sres, null);
            if (!rfr)
            {
                auto err = GetLastError();
                if (err == ERROR_IO_PENDING) { /+ buffer empty +/ }
                else throw new ReadException(port, text("error ", err));
            }
            res = sres;
        }

        return buf[0..res];
    }

    override ptrdiff_t write(const(void[]) arr)
    {
        if (closed) throw new PortClosedException(port);

        ptrdiff_t res;
        auto ptr = arr.ptr;
        auto len = arr.length;

        version (Posix)
        {
            res = posixWrite(_handle, ptr, len);
            if (res < 0)
            {
                if (errno == EAGAIN) res = 0; // buffer filled
                else throw new WriteException(port, text("errno ", errno));
            }
        }
        version (Windows)
        {
            uint sres;
            auto wfr = WriteFile(_handle, ptr, cast(uint)len, &sres, null);
            if (!wfr)
            {
                auto err = GetLastError();
                if (err == ERROR_IO_PENDING) { /+ buffer filled +/ }
                else throw new WriteException(port, text("error ", err));
            }
            res = sres;
        }

        return res;
    }
}

/++ Serial Port Fiber Ready
 +/
class SerialPortFR : SerialPortNonBlk
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
        // approx theoretical time for receive or send
        // one byte (8 bit + 1 start bit + 1 stop bit)
        return (cast(ulong)(10.0f / config.baudRate * 1e6)).usecs;
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

    ///
    this(string port, Config conf, void delegate(Duration) slp=null)
    {
        this.sleepFunc = slp;
        super(port, conf);
    }

    /++ Read data by parts, sleep between checks.

        Sleep time calculates from baud rate and count of bits in one byte.

        Params:
            arr = buffer for reading
            timeout = timeout for first byte recive
            frameGap = detect new data frame (return current) by silence period

        Returns: slice of arr

        Throws:
            PortClosedException
            ReadException

        See_Also: SerialPort.read
     +/
    void[] readLoop(void[] arr, Duration timeout=1.seconds,
                                Duration frameGap=50.msecs)
    {
        if (closed) throw new PortClosedException(port);

        ptrdiff_t readed;

        auto pause = ioPause();

        StopWatch silence, full;

        full.start();
        while (true)
        {
            // TODO: maybe return readed data?
            enforce(readed <= arr.length,
                    new SerialPortException("read more what can"));

            const res = this.read(arr[readed..$]).length;

            readed += res;

            if (res == 0)
            {
                if (readed > 0 && silence.peek > frameGap)
                    return arr[0..readed];

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

        Params:
            arr = data for writing
            timeout = time for writing data

        Throws:
            PortClosedException
            WriteException
            TimeoutException if full write time is out

        See_Also: SerialPort.write
     +/
    void writeLoop(const(void[]) arr, Duration timeout=10.msecs)
    {
        if (closed) throw new PortClosedException(port);

        size_t written = this.write(arr);
        if (written == arr.length) return;

        auto pause = ioPause();
        const full = StopWatch(AutoStart.yes);

        while (written < arr.length)
        {
            if (full.peek > timeout)
                throw new TimeoutException(port);

            written += this.write(arr[written..$]);
            if (written < arr.length) this.sleep(pause);
        }
    }
}