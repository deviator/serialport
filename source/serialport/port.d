///
module serialport.port;

import std.algorithm;
import std.array;
import std.conv : to, text;
import std.exception;
import std.experimental.logger;
import std.path;
import std.string;
import core.time;
import std.datetime.stopwatch;

import serialport.types;
import serialport.config;
import serialport.exception;

///
class SerialPort
{
protected:
    ///
    string port;

    SPHandle _handle = initHandle;

public:

    ///
    alias Config = SPConfig;

    /++ Construct SerialPort using extend mode string.

        First part of extend mode string must have port name
        (e.g. "com1" or "/dev/ttyUSB0"), second part is equal
        to config mode string. Parts separates by `modeSplitChar` (`:`).

        Example extend mode string: "/dev/ttyUSB0:9600:8N1"

        Params:
            exmode = extend mode string

        See_Also: Config.parse, Config.set(string mode)

        Throws:
            ParseModeException
     +/
    this(string exmode)
    {
        auto s = exmode.split(modeSplitChar);
        if (s.length == 0) throw new ParseModeException("empty mode");
        this(s[0], s.length > 1 ? Config.parse(s[1..$].join(modeSplitChar)) : Config.init);
    }

    /++ Construct SerialPort using port name and mode string.

        Params:
            port = port name
            mode = config mode string

        See_Also: Config.parse, Config.set(string mode)
     +/
    this(string port, string mode) { this(port, Config.parse(mode)); }

    /++ Params:
            port = port name
            baudRate = baudrate
     +/
    this(string port, uint baudRate) { this(port, Config(baudRate)); }

    /++ Params:
            port = port name
            baudRate = baudrate
            mode = config mode string

        See_Also: Config.parse, Config.set(string mode)
     +/
    this(string port, uint baudRate, string mode)
    { this(port, Config(baudRate).set(mode)); }

    ///
    this(string port, Config conf)
    {
        this.port = port;
        setup(conf);
    }

    ~this() { close(); }

    /// close handle
    void close()
    {
        if (closed) return;
        closeHandle(_handle);
        _handle = initHandle;
    }

    ///
    string name() const @property { return port; }

    ///
    inout(SPHandle) handle() inout @property { return _handle; }

    ///
    void reopen(string port, Config cfg)
    {
        if (!closed) close();
        this.port = port;
        setup(cfg);
    }

    ///
    override string toString() { return port ~ ":" ~ config.mode; }

    ///
    SerialPort set(T)(T val)
        if (is(typeof(Config.init.set(val))))
    {
        Config tmp = config;
        tmp.set(val);
        config = tmp;
        return this;
    }

    ///
    bool closed() const @property
    {
        version (Posix) return _handle == initHandle;
        version (Windows) return _handle is initHandle;
    }

    /// Get config
    // const for disallow `com.config.set(value)`
    const(Config) config() @property
    {
        enforce(!closed, new PortClosedException(port));

        Config ret;

        version (Posix)
        {
            termios opt;
            enforce(tcgetattr(_handle, &opt) != -1,
                    new SerialPortException(format("Failed while call tcgetattr: %d", errno)));

            ret.baudRate = getUintBaudRate();

            if (opt.c_cflag.hasFlag(PARODD)) ret.parity = Parity.odd;
            else if (!(opt.c_cflag & PARENB)) ret.parity = Parity.none;
            else ret.parity = Parity.even;

                    if (opt.c_cflag.hasFlag(CS8)) ret.dataBits = DataBits.data8;
            else if (opt.c_cflag.hasFlag(CS7)) ret.dataBits = DataBits.data7;
            else if (opt.c_cflag.hasFlag(CS6)) ret.dataBits = DataBits.data6;
            else ret.dataBits = DataBits.data5;

            ret.stopBits = opt.c_cflag.hasFlag(CSTOPB) ? StopBits.two : StopBits.one;
        }
        version (Windows)
        {
            DCB cfg;
            GetCommState(_handle, &cfg);

            ret.baudRate = cast(uint)cfg.BaudRate;

            enum pAA = [NOPARITY: Parity.none,
                        ODDPARITY: Parity.odd,
                        EVENPARITY: Parity.even];

            ret.parity = pAA[cfg.Parity];

            enum dbAA = [5: DataBits.data5,
                         6: DataBits.data6,
                         7: DataBits.data7,
                         8: DataBits.data8];

            ret.dataBits = dbAA.get(cfg.ByteSize, DataBits.data8);

            ret.stopBits = cfg.StopBits == ONESTOPBIT ? StopBits.one : StopBits.two;
        }

        return ret;
    }

    /// Set config
    void config(Config c) @property
    {
        alias UE = UnsupportedException;
        enforce(!closed, new PortClosedException(port));

        version (Posix)
        {
            setUintBaudRate(c.baudRate);

            termios opt;
            enforce(tcgetattr(_handle, &opt) != -1,
                    new SerialPortException(format("Failed while call tcgetattr: %d", errno)));

            final switch (c.parity)
            {
                case Parity.none:
                    opt.c_cflag &= ~PARENB;
                    break;
                case Parity.odd:
                    opt.c_cflag |= (PARENB | PARODD);
                    break;
                case Parity.even:
                    opt.c_cflag &= ~PARODD;
                    opt.c_cflag |= PARENB;
                    break;
            }

            final switch (c.stopBits)
            {
                case StopBits.one:
                    opt.c_cflag &= ~CSTOPB;
                    break;
                case StopBits.onePointFive:
                case StopBits.two:
                    opt.c_cflag |= CSTOPB;
                    break;
            }

            opt.c_cflag &= ~CSIZE;
            switch (c.dataBits) {
                case DataBits.data5: opt.c_cflag |= CS5; break;
                case DataBits.data6: opt.c_cflag |= CS6; break;
                case DataBits.data7: opt.c_cflag |= CS7; break;
                case DataBits.data8: opt.c_cflag |= CS8; break;
                default:
                    errorf("config dataBits is setted as %d, set default CS8", c.dataBits);
                    opt.c_cflag |= CS8;
                    break;
            }

            enforce(tcsetattr(_handle, TCSANOW, &opt) != -1,
                    new SerialPortException(format("Failed while call tcsetattr: %d", errno)));

            auto test = config;
            enforce(test.baudRate == c.baudRate, new UE(c.baudRate));
            enforce(test.parity   == c.parity,   new UE(c.parity));
            enforce(test.stopBits == c.stopBits, new UE(c.stopBits));
            enforce(test.dataBits == c.dataBits, new UE(c.dataBits));
        }
        version (Windows)
        {
            DCB cfg;
            GetCommState(_handle, &cfg);

            if (cfg.BaudRate != cast(DWORD)c.baudRate)
            {
                cfg.BaudRate = cast(DWORD)c.baudRate;
                enforce(SetCommState(_handle, &cfg),
                        new UE(c.baudRate));
            }

            const tmpParity = [Parity.none: NOPARITY, Parity.odd: ODDPARITY,
                               Parity.even: EVENPARITY][c.parity];

            if (cfg.Parity != tmpParity)
            {
                cfg.Parity = cast(ubyte)tmpParity;
                enforce(SetCommState(_handle, &cfg),
                        new UE(c.parity));
            }

            const tmpStopBits = [StopBits.one: ONESTOPBIT,
                                 StopBits.onePointFive: ONESTOPBIT,
                                 StopBits.two: TWOSTOPBITS][c.stopBits];

            if (cfg.StopBits != tmpStopBits)
            {
                cfg.StopBits = cast(ubyte)tmpStopBits;
                enforce(SetCommState(_handle, &cfg),
                        new UE(c.stopBits));
            }

            if (cfg.ByteSize != cast(typeof(cfg.ByteSize))c.dataBits)
            {
                cfg.ByteSize = cast(typeof(cfg.ByteSize))c.dataBits;
                enforce(SetCommState(_handle, &cfg),
                        new UE(c.dataBits));
            }
        }
    }

    @property
    {
        ///
        Parity parity() { return config.parity; }
        ///
        uint baudRate() { return config.baudRate; }
        ///
        DataBits dataBits() { return config.dataBits; }
        ///
        StopBits stopBits() { return config.stopBits; }

        ///
        Parity parity(Parity v) { set(v); return v; }
        ///
        uint baudRate(uint v) { set(v); return v; }
        ///
        DataBits dataBits(DataBits v) { set(v); return v; }
        ///
        StopBits stopBits(StopBits v) { set(v); return v; }
    }

    /++ List available serial ports in system
        +/
    static string[] listAvailable() @property
    {
        version (Posix)
        {
            import std.file : exists;
            return dirEntries("/sys/class/tty", SpanMode.shallow)
                    .map!(a=>"/dev/"~a.name.baseName)
                    .filter!(a=>a.exists)
                    .array.sort.array
                    ~
                    dirEntries("/dev/pts", SpanMode.shallow)
                    .map!(a=>a.name).array.sort.array;
        }
        version (Windows)
        {
            import std.windows.registry : Registry;
            string[] arr;
            try foreach (v; Registry
                            .localMachine()
                            .getKey("HARDWARE")
                            .getKey("DEVICEMAP")
                            .getKey("SERIALCOMM")
                            .values)
                arr ~= v.value_SZ;
            catch (Throwable e) .error(e.msg);
            return arr;
        }
    }

    /++ Read data from port

        Params:
            buf = buffer for reading

        Returns: slice of buf

        Throws:
            PortClosedException if port closed
            ReadException if read error occurs
     +/
    void[] read(void[] buf)
    {
        if (closed) throw new PortClosedException(port);

        auto ptr = buf.ptr;
        auto len = buf.length;

        size_t res;

        version (Posix)
        {
            auto sres = posixRead(_handle, ptr, len);

            // no bytes for read, it's ok
            if (sres < 0 && errno == EAGAIN) sres = 0;
            else enforce(sres >= 0,
                    new ReadException(port, text("errno ", errno)));
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

    /++ Write data to port

        Params:
            arr = writed data
        
        Returns: count of writen bytes

        Throws:
            PortClosedException if port closed
            WriteException if read error occurs
     +/
    ptrdiff_t write(const(void[]) arr)
    {
        if (closed) throw new PortClosedException(port);

        ptrdiff_t res;
        auto ptr = arr.ptr;
        auto len = arr.length;

        version (Posix)
        {
            res = posixWrite(_handle, ptr, len);
            if (res < 0 && errno == EAGAIN) res = 0; // buffer filled
            else enforce(res >= 0, new WriteException(port, text("errno ", errno)));
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

protected:

    alias SFE = SetupFailException;

    version (Posix)
    {
        void setUintBaudRate(uint br)
        {
            version (usetermios2)
            {
                import std.conv : octal;
                enum CBAUD  = octal!10017;
                enum BOTHER = octal!10000;

                termios2 opt2;
                enforce(ioctl(_handle, TCGETS2, &opt2) != -1,
                        new SFE(port, "can't get termios2 options"));
                opt2.c_cflag &= ~CBAUD; //Remove current BAUD rate
                opt2.c_cflag |= BOTHER; //Allow custom BAUD rate using int input
                opt2.c_ispeed = br;     //Set the input BAUD rate
                opt2.c_ospeed = br;     //Set the output BAUD rate
                ioctl(_handle, TCSETS2, &opt2);
            }
            else
            {
                enforce(br in unixBaudList, new UnsupportedException(br));

                auto baud = unixBaudList[br];

                termios opt;
                enforce(tcgetattr(_handle, &opt) != -1,
                    new SFE(port, "can't get termios options"));

                //cfsetispeed(&opt, B0);
                cfsetospeed(&opt, baud);

                enforce(tcsetattr(_handle, TCSANOW, &opt) != -1,
                        new SFE(port, "Failed while call tcsetattr"));
            }
        }

        uint getUintBaudRate()
        {
            version (usetermios2)
            {
                termios2 opt2;
                enforce(ioctl(_handle, TCGETS2, &opt2) != -1,
                        new SFE(port, "can't get termios2 options"));
                return opt2.c_ospeed;
            }
            else
            {
                termios opt;
                enforce(tcgetattr(_handle, &opt) != -1,
                    new SFE(port, "can't get termios options"));
                version (OSX) alias true_speed_t = uint;
                else alias true_speed_t = typeof(cfgetospeed(&opt));
                auto b = cast(true_speed_t)cfgetospeed(&opt);
                if (b !in unixUintBaudList)
                {
                    warningf("unknown baud speed setted: %s", b);
                    return 0;
                }
                return unixUintBaudList[b];
            }
        }
    }

    /// open handler, set new config
    void setup(Config conf)
    {
        enforce(port.length, new SFE(port, "zero length name"));

        version (Posix)
        {
            _handle = open(port.toStringz(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            enforce(_handle != -1,
                    new SFE(port, format("Can't open port (errno %d)", errno)));

            termios opt;
            enforce(tcgetattr(_handle, &opt) != -1,
                new SFE(port, "can't get termios options"));

            // make raw
            opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                             INLCR | IGNCR | ICRNL | IXON);
            opt.c_oflag &= ~OPOST;
            opt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            opt.c_cflag &= ~(CSIZE | PARENB);

            // hardware flow control
            version (OSX)
            {
                /+
                The CCTS_OFLOW (CRTSCTS) flag is currently unused.
                http://www.manpages.info/macosx/termios.4.html
                 +/
            }
            else
            {
                if (conf.hardwareDisableFlowControl)
                    opt.c_cflag &= ~CRTSCTS;
            }

            opt.c_cflag |= CS8;

            enforce(tcsetattr(_handle, TCSANOW, &opt) != -1,
                    new SFE(port, format("Failed while" ~
                            " call tcsetattr (errno %d)", errno)));
        }
        else version (Windows)
        {
            auto fname = `\\.\` ~ port;
            _handle = CreateFileA(fname.toStringz,
                        GENERIC_READ | GENERIC_WRITE, 0, null,
                        OPEN_EXISTING, 0, null);

            enforce(_handle !is INVALID_HANDLE_VALUE,
                new SFE(port, format("can't CreateFileA '%s' with error: %d",
                        fname, GetLastError())));

            SetupComm(_handle, 4096, 4096);
            PurgeComm(_handle, PURGE_TXABORT | PURGE_TXCLEAR |
                               PURGE_RXABORT | PURGE_RXCLEAR);

            COMMTIMEOUTS tm;
            tm.ReadIntervalTimeout         = DWORD.max;
            tm.ReadTotalTimeoutMultiplier  = 0;
            tm.ReadTotalTimeoutConstant    = 0;
            tm.WriteTotalTimeoutMultiplier = 0;
            tm.WriteTotalTimeoutConstant   = 0;

            enforce(SetCommTimeouts(_handle, &tm) != 0,
                new SFE(port, format("can't SetCommTimeouts with error: %d", GetLastError())));
        }

        config = conf;
    }
}

private bool hasFlag(A,B)(A a, B b) @property { return (a & b) == b; }

/// Serial port with basic loops
class SerialPortBL : SerialPort
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
                while (tm.peek.to!Duration < dt) Fiber.yield();
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

    /++ Construct SerialPortBL using extend mode string.

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
                if (readed > 0 && silence.peek.to!Duration > frameGap)
                    return arr[0..readed];

                if (!silence.running) silence.start();
            }
            else
            {
                silence.stop();
                silence.reset();
            }

            if (readed == 0 && full.peek.to!Duration > timeout)
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
            if (full.peek.to!Duration > timeout)
                throw new TimeoutException(port);
            written += this.write(arr[written..$]);
            if (written < arr.length) this.sleep(pause);
        }
    }
}