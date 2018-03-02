///
module serialport.port;

import std.algorithm;
import std.array;
import std.conv : to, text, octal;
import std.exception;
import std.experimental.logger;
import std.path;
import std.string;
import core.time;
import std.datetime.stopwatch;

import serialport.types;
import serialport.exception;

version (Posix) {} else version (Windows) {}
else static assert(0, "unsupported platform");

///
class SerialPort
{
protected:
    ///
    string port;

    SPHandle _handle = initHandle;

    /// preform pause
    void sleep(Duration dt)
    {
        if (sleepFunc !is null) sleepFunc(dt);
        else
        {
            import core.thread : Fiber, Thread;
            if (Fiber.getThis is null)
                Thread.sleep(dt);
            else
            {
                const tm = StopWatch(AutoStart.yes);
                while (tm.peek.to!Duration < dt) Fiber.yield();
            }
        }
    }

    Duration ioPause()
    {
        // approx theoretical time for receive or send
        // one byte (8 bit + 1 start bit + 1 stop bit)
        return (cast(ulong)(10.0f / baudRate * 1e6)).usecs;
    }

public:

    ///
    static immutable string modeSplitChar=":";

    ///
    static struct Config
    {
        ///
        uint baudRate=9600;
        ///
        DataBits dataBits=DataBits.data8;
        ///
        Parity parity=Parity.none;
        ///
        StopBits stopBits=StopBits.one;

        ///
        bool hardwareDisableFlowControl = true;

        ///
        auto set(Parity v) { parity = v; return this; }
        ///
        auto set(uint v) { baudRate = v; return this; }
        ///
        auto set(DataBits v) { dataBits = v; return this; }
        ///
        auto set(StopBits v) { stopBits = v; return this; }
        /++
            use "B:XYZ"
            where:
                B is baud rate
                X is data bits (5, 6, 7, 8)
                Y is parity ('N' or 'n' -- none, 'E' or 'e' -- even, 'O' or 'o' -- odd)
                Z is stop bits ('1', '1.5', '2')

            example: "9600:8N1" "7o1.5" "2400:6e2"
         +/
        auto set(string mode)
        {
            auto errstr = "usupported mode '%s'".format(mode);
            enforce(mode.length >= 3, new SerialPortException(errstr));

            auto vals = mode.split(modeSplitChar);

            if (vals.length == 0) return this;

            if (vals.length > 2) throw new SerialPortException(errstr);

            if (vals.length == 2)
            {
                if (vals[0].length)
                    baudRate = vals[0].to!uint;
                mode = vals[1];
            }
            else mode = vals[0];

            auto db = cast(int)mode[0] - cast(int)'0';
            if (db >= 5 && db <= 8) dataBits = cast(DataBits)db;
            else throw new SerialPortException(errstr);

            auto p = mode[1..2].toLower;
            if (p == "n" || p == "o" || p == "e")
            {
                parity = ["n": Parity.none,
                          "o": Parity.odd,
                          "e": Parity.even][p];
            }
            else throw new SerialPortException(errstr);

            auto sb = mode[2..$];
            if (sb == "1" || sb == "1.5" || sb == "2")
            {
                stopBits = ["1": StopBits.one,
                            "1.5": StopBits.onePointFive,
                            "2": StopBits.two][sb];
            }
            else throw new SerialPortException(errstr);

            return this;
        }

        /// seealso: set(string mode)
        static Config parse(string mode)
        {
            Config ret;
            ret.set(mode);
            return ret;
        }

        ///
        string mode() const @property
        {
            return "%d:%s%s%s".format(
                baudRate,
                dataBits.to!int,
                [Parity.none: "n",
                 Parity.odd:  "o",
                 Parity.even: "e"][parity],
                [StopBits.one: "1",
                 StopBits.onePointFive: "1.5",
                 StopBits.two: "2"
                ][stopBits]
            );
        }
    }

    /// extended delegate for perform sleep
    void delegate(Duration dt) sleepFunc;

    /// "/dev/ttyUSB0:9600:8N1"
    this(string mode, void delegate(Duration) slp=null)
    {
        auto s = mode.split(modeSplitChar);
        if (s.length == 0)
            throw new SerialPortException(
                "usupported serial port mode '%s'".format(mode));
        this.port = s[0];
        this.sleepFunc = slp;
        Config conf;
        if (s.length > 1)
            conf.set(s[1..$].join(modeSplitChar));
        setup(conf);
    }

    ///
    this(string port, string mode, void delegate(Duration) slp=null)
    {
        this.port = port;
        this.sleepFunc = slp;
        setup(Config.parse(mode));
    }

    ///
    this(string port, uint baudRate, void delegate(Duration) slp=null)
    { this(port, Config(baudRate), slp); }

    ///
    this(string port, uint baudRate, string mode, void delegate(Duration) slp=null)
    { this(port, Config(baudRate).set(mode), slp); }

    ///
    this(string port, Config conf, void delegate(Duration) slp=null)
    {
        this.port = port;
        this.sleepFunc = slp;
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
    SPHandle handle() const @property { return _handle; }

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
    SerialPort set(Parity p) { config = config.set(p); return this; }
    ///
    SerialPort set(uint br) { config = config.set(br); return this; }
    ///
    SerialPort set(DataBits db) { config = config.set(db); return this; }
    ///
    SerialPort set(StopBits sb) { config = config.set(sb); return this; }
    ///
    SerialPort set(string mode) { config = config.set(mode); return this; }

    @property
    {
        ///
        bool closed() const
        {
            version (Posix) return _handle == -1;
            version (Windows) return _handle is null;
        }

        ///
        Config config()
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

                static immutable pAA = [NOPARITY: Parity.none,
                                        ODDPARITY: Parity.odd,
                                        EVENPARITY: Parity.even];

                ret.parity = pAA[cfg.Parity];

                static immutable dbAA = [5: DataBits.data5,
                                         6: DataBits.data6,
                                         7: DataBits.data7,
                                         8: DataBits.data8];

                ret.dataBits = dbAA.get(cfg.ByteSize, DataBits.data8);

                ret.stopBits = cfg.StopBits == ONESTOPBIT ? StopBits.one : StopBits.two;
            }

            return ret;
        }

        ///
        void config(Config c)
        {
            if (closed) throw new PortClosedException(port);

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
                        errorf("config dataBits is setted as %d, set default CS8",
                                c.dataBits);
                        opt.c_cflag |= CS8;
                        break;
                }

                enforce(tcsetattr(_handle, TCSANOW, &opt) != -1,
                        new SerialPortException(format("Failed while call tcsetattr: %d", errno)));

                auto test = config;

                enforce(test.baudRate == c.baudRate,
                            new BaudRateUnsupportedException(c.baudRate));
                enforce(test.parity == c.parity,
                            new ParityUnsupportedException(c.parity));
                enforce(test.stopBits == c.stopBits,
                            new StopBitsUnsupportedException(c.stopBits));
                enforce(test.dataBits == c.dataBits,
                            new DataBitsUnsupportedException(c.dataBits));
            }
            version (Windows)
            {
                DCB cfg;
                GetCommState(_handle, &cfg);

                if (cfg.BaudRate != cast(DWORD)c.baudRate)
                {
                    cfg.BaudRate = cast(DWORD)c.baudRate;
                    enforce(SetCommState(_handle, &cfg),
                            new BaudRateUnsupportedException(c.baudRate));
                }

                const tmpParity = [Parity.none: NOPARITY, Parity.odd: ODDPARITY,
                                   Parity.even: EVENPARITY][c.parity];
                if (cfg.Parity != tmpParity)
                {
                    cfg.Parity = cast(ubyte)tmpParity;
                    enforce(SetCommState(_handle, &cfg),
                            new ParityUnsupportedException(c.parity));
                }

                const tmpStopBits = [StopBits.one: ONESTOPBIT,
                                     StopBits.onePointFive: ONESTOPBIT,
                                     StopBits.two: TWOSTOPBITS][c.stopBits];

                if (cfg.StopBits != tmpStopBits)
                {
                    cfg.StopBits = cast(ubyte)tmpStopBits;
                    enforce(SetCommState(_handle, &cfg),
                            new StopBitsUnsupportedException(c.stopBits));
                }

                if (cfg.ByteSize != cast(typeof(cfg.ByteSize))c.dataBits)
                {
                    cfg.ByteSize = cast(typeof(cfg.ByteSize))c.dataBits;
                    enforce(SetCommState(_handle, &cfg),
                            new DataBitsUnsupportedException(c.dataBits));
                }
            }
        }

        ///
        Parity parity() { return config.parity; }
        ///
        uint baudRate() { return config.baudRate; }
        ///
        DataBits dataBits() { return config.dataBits; }
        ///
        StopBits stopBits() { return config.stopBits; }

        ///
        Parity parity(Parity v) { config = config.set(v); return v; }
        ///
        uint baudRate(uint v) { config = config.set(v); return v; }
        ///
        DataBits dataBits(DataBits v) { config = config.set(v); return v; }
        ///
        StopBits stopBits(StopBits v) { config = config.set(v); return v; }

        deprecated("use listAvailable() @property")
        alias ports = listAvailable;
        ///
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
    }

    ///
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
                if (err == ERROR_IO_PENDING) { /+ asynchronously +/ }
                else throw new ReadException(port, text("error ", err));
            }
            res = sres;
        }

        return buf[0..res];
    }

    ///
    ptrdiff_t write(const(void[]) arr)
    {
        if (closed) throw new PortClosedException(port);

        ptrdiff_t res;
        auto ptr = arr.ptr;
        auto len = arr.length;

        version (Posix)
        {
            res = posixWrite(_handle, ptr, len);
            enforce(res >= 0, new WriteException(port, text("errno ", errno)));
        }
        version (Windows)
        {
            uint sres;
            auto wfr = WriteFile(_handle, ptr, cast(uint)len, &sres, null);
            if (!wfr)
            {
                auto err = GetLastError();
                if (err == ERROR_IO_PENDING) { /+ asynchronously +/ }
                else throw new WriteException(port, text("error ", err));
            }
            res = sres;
        }

        return res;
    }

    ///
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
            enforce(readed <= arr.length,
                    new SerialPortException("read more what can"));

            const res = read(arr[readed..$]).length;

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

            sleep(pause);
        }
    }

    ///
    void writeLoop(const(void[]) arr, Duration timeout=10.msecs)
    {
        if (closed) throw new PortClosedException(port);
        size_t written = write(arr);
        if (written == arr.length) return;
        auto pause = ioPause();
        const full = StopWatch(AutoStart.yes);
        while (written < arr.length)
        {
            if (full.peek.to!Duration > timeout)
                throw new TimeoutException(port);
            written += write(arr[written..$]);
            if (written < arr.length) sleep(pause);
        }
    }

protected:

    version (Posix)
    {
        void setUintBaudRate(uint br)
        {
            version (usetermios2)
            {
                enum CBAUD  = octal!10017;
                enum BOTHER = octal!10000;

                termios2 opt2;
                enforce(ioctl(_handle, TCGETS2, &opt2) != -1,
                        new SetupFailException(port, "can't get termios2 options"));
                opt2.c_cflag &= ~CBAUD; //Remove current BAUD rate
                opt2.c_cflag |= BOTHER; //Allow custom BAUD rate using int input
                opt2.c_ispeed = br;     //Set the input BAUD rate
                opt2.c_ospeed = br;     //Set the output BAUD rate
                ioctl(_handle, TCSETS2, &opt2);
            }
            else
            {
                enforce(br in unixBaudList,
                        new BaudRateUnsupportedException(br));

                auto baud = unixBaudList[br];

                termios opt;
                enforce(tcgetattr(_handle, &opt) != -1,
                    new SerialPortException(port, "can't get termios options"));

                //cfsetispeed(&opt, B0);
                cfsetospeed(&opt, baud);

                enforce(tcsetattr(_handle, TCSANOW, &opt) != -1,
                        new SerialPortException("Failed while call tcsetattr"));
            }
        }

        uint getUintBaudRate()
        {
            version (usetermios2)
            {
                termios2 opt2;
                enforce(ioctl(_handle, TCGETS2, &opt2) != -1,
                        new SetupFailException(port, "can't get termios2 options"));
                return opt2.c_ospeed;
            }
            else
            {
                termios opt;
                enforce(tcgetattr(_handle, &opt) != -1,
                    new SerialPortException(port, "can't get termios options"));
                auto b = cfgetospeed(&opt);
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
    final void setup(Config conf)
    {
        enforce(port.length, new SetupFailException(port, "zero length name"));

        version (Posix)
        {
            _handle = open(port.toStringz(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            enforce(_handle != -1,
                    new SetupFailException(port,
                        format("Can't open port (errno %d)", errno)));

            termios opt;
            enforce(tcgetattr(_handle, &opt) != -1,
                new SetupFailException(port, "can't get termios options"));

            // make raw
            opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                             INLCR | IGNCR | ICRNL | IXON);
            opt.c_oflag &= ~OPOST;
            opt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            opt.c_cflag &= ~(CSIZE | PARENB);
            if (conf.hardwareDisableFlowControl)
                opt.c_cflag &= ~CRTSCTS;
            opt.c_cflag |= CS8;

            enforce(tcsetattr(_handle, TCSANOW, &opt) != -1,
                    new SetupFailException(format("Failed while" ~
                            " call tcsetattr (errno %d)", errno)));
        }
        else version (Windows)
        {
            auto fname = `\\.\` ~ port;
            _handle = CreateFileA(fname.toStringz,
                        GENERIC_READ | GENERIC_WRITE, 0, null,
                        OPEN_EXISTING, 0, null);

            if (_handle is INVALID_HANDLE_VALUE)
            {
                auto err = GetLastError();
                throw new SetupFailException(port,
                        format("can't CreateFileA '%s' with error: %d", fname, err));
            }

            SetupComm(_handle, 4096, 4096);
            PurgeComm(_handle, PURGE_TXABORT | PURGE_TXCLEAR |
                               PURGE_RXABORT | PURGE_RXCLEAR);

            COMMTIMEOUTS tm;
            tm.ReadIntervalTimeout         = DWORD.max;
            tm.ReadTotalTimeoutMultiplier  = 0;
            tm.ReadTotalTimeoutConstant    = 0;
            tm.WriteTotalTimeoutMultiplier = 0;
            tm.WriteTotalTimeoutConstant   = 0;

            if (SetCommTimeouts(_handle, &tm) == 0)
                throw new SetupFailException(port,
                        format("can't SetCommTimeouts with error: %d", GetLastError()));
        }

        config = conf;
    }
}

private bool hasFlag(A,B)(A a, B b) @property { return (a & b) == b; }

unittest
{
    alias Cfg = SerialPort.Config;

    Cfg c;
    c.set("2400:7e1.5");
    assertNotThrown(c.set(c.mode));
    assert(c.baudRate == 2400);
    assert(c.dataBits == DataBits.data7);
    assert(c.parity == Parity.even);
    assert(c.stopBits == StopBits.onePointFive);
    c.set("8N1");
    assertNotThrown(c.set(c.mode));
    assert(c.baudRate == 2400);
    assert(c.dataBits == DataBits.data8);
    assert(c.parity == Parity.none);
    assert(c.stopBits == StopBits.one);
    c.set("320:5o2");
    assertNotThrown(c.set(c.mode));
    assert(c.baudRate == 320);
    assert(c.dataBits == DataBits.data5);
    assert(c.parity == Parity.odd);
    assert(c.stopBits == StopBits.two);

    assertThrown!SerialPortException(c.set("4o2"));
    assertThrown!SerialPortException(c.set("5x2"));
    assertThrown!SerialPortException(c.set("8e3"));
    assertNotThrown!SerialPortException(c.set(":8N1"));
    assertNotThrown(c.set(c.mode));
}