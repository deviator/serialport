///
module serialport.port;

import std.algorithm;
import std.array;
import std.conv : to, octal;
import std.exception;
import std.experimental.logger;
import std.path;
import std.string;
import core.time;
import std.datetime : StopWatch;

import serialport.types;
import serialport.exception;

version (Posix) {}
else version (Windows) {}
else static assert(0, "unsupported platform");

///
class SerialPort
{
protected:
    ///
    string port;

    ///
    version (Posix) int handle = -1;
    ///
    version (Windows) HANDLE handle = null;

    /// preform pause
    void yield()
    {
        if (yieldFunc !is null) yieldFunc();
        else
        {
            import core.thread;
            if (Fiber.getThis is null) Thread.yield();
            else Fiber.yield();
        }
    }

public:

    ///
    static struct Config
    {
        ///
        uint baudRate=9600;
        ///
        Parity parity=Parity.none;
        ///
        DataBits dataBits=DataBits.data8;
        ///
        StopBits stopBits=StopBits.one;

        auto set(Parity v) { parity = v; return this; }
        auto set(uint v) { baudRate = v; return this; }
        auto set(DataBits v) { dataBits = v; return this; }
        auto set(StopBits v) { stopBits = v; return this; }
    }

    /// extended delegate for yielding
    void delegate() yieldFunc;

    ///
    this(string port, Config conf=Config.init, void delegate() yh=null)
    {
        this.port = port;
        this.yieldFunc = yh;
        setup(conf);
    }

    ///
    this(string port, uint baudRate, void delegate() yh=null)
    { this(port, Config(baudRate), yh); }

    ///
    this(string port, uint baudRate, Parity parity, void delegate() yh=null)
    { this(port, Config(baudRate, parity), yh); }

    ///
    this(string port, uint baudRate, Parity parity,
            DataBits dataBits,
            StopBits stopBits, void delegate() yh=null)
    { this(port, Config(baudRate, parity, dataBits, stopBits), yh); }

    ~this() { close(); }

    /// close handle
    void close()
    {
        if (closed) return;
        version(Windows)
        {
            CloseHandle(handle);
            handle = null;
        }
        version(Posix)
        {
            posixClose(handle);
            handle = -1;
        }
    }

    ///
    override string toString() { return port; }

    ///
    SerialPort set(Parity p) { config = config.set(p); return this; }
    ///
    SerialPort set(uint br) { config = config.set(br); return this; }
    ///
    SerialPort set(DataBits db) { config = config.set(db); return this; }
    ///
    SerialPort set(StopBits sb) { config = config.set(sb); return this; }

    @property
    {
        ///
        bool closed() const
        {
            version(Posix) return handle == -1;
            version(Windows) return handle is null;
        }

        ///
        Config config()
        {
            enforce(!closed, new PortClosedException(port));

            Config ret;

            version (Posix)
            {
                termios opt;
                enforce(tcgetattr(handle, &opt) != -1,
                        new SerialPortException("Failed while call tcgetattr"));

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
                GetCommState(handle, &cfg);

                ret.baudRate = cast(uint)cfg.BaudRate;

                ret.parity = [NOPARITY: Parity.none, ODDPARITY: Parity.odd,
                              EVENPARITY: Parity.even][cfg.Parity];

                ret.dataBits = [5: DataBits.data5, 6: DataBits.data6,
                                7: DataBits.data7, 8: DataBits.data8]
                                    .get(cfg.ByteSize, DataBits.data8);

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
                enforce(tcgetattr(handle, &opt) != -1,
                        new SerialPortException("Failed while call tcgetattr"));

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

                enforce(tcsetattr(handle, TCSANOW, &opt) != -1,
                        new SerialPortException("Failed while call tcsetattr"));
            }
            version (Windows)
            {
                DCB cfg;
                GetCommState(handle, &cfg);

                if (cfg.BaudRate != cast(DWORD)c.baudRate)
                {
                    cfg.BaudRate = cast(DWORD)c.baudRate;
                    enforce(SetCommState(handle, &cfg),
                            new BaudRateUnsupportedException(c.baudRate));
                }

                auto tmpParity = [Parity.none: NOPARITY, Parity.odd: ODDPARITY,
                                  Parity.even: EVENPARITY][c.parity];
                if (cfg.Parity != tmpParity)
                {
                    cfg.Parity = cast(ubyte)tmpParity;
                    enforce(SetCommState(handle, &cfg),
                            new ParityUnsupportedException(c.parity));
                }

                auto tmpStopBits = [StopBits.one: ONESTOPBIT,
                                    StopBits.onePointFive: ONESTOPBIT,
                                    StopBits.two: TWOSTOPBITS][c.stopBits];

                if (cfg.StopBits != tmpStopBits)
                {
                    cfg.StopBits = cast(ubyte)tmpStopBits;
                    enforce(SetCommState(handle, &cfg),
                            new StopBitsUnsupportedException(c.stopBits));
                }

                if (cfg.ByteSize != cast(typeof(cfg.ByteSize))c.dataBits)
                {
                    cfg.ByteSize = cast(typeof(cfg.ByteSize))c.dataBits;
                    enforce(SetCommState(handle, &cfg),
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

        ///
        static string[] ports()
        {
            version (Posix)
            {
                bool onlyComPorts(string n)
                {
                    static bool isInRange(T, U)(T v, U a, U b)
                    { return a <= v && v <= b; }

                    version(linux)   return n.startsWith("ttyUSB") ||
                                            n.startsWith("ttyS");
                    version(darwin)  return n.startsWith("cu");
                    version(FreeBSD) return n.startsWith("cuaa") ||
                                            n.startsWith("cuad");
                    version(openbsd) return n.startsWith("tty");
                    version(solaris) return n.startsWith("tty") &&
                                            isInRange(n[$-1],'a','z');
                }

                return dirEntries("/dev/", SpanMode.shallow)
                        .map!(a=>a.name.baseName)
                        .filter!onlyComPorts
                        .map!(a=>"/dev/" ~ a)
                        .array;
            }
            version (Windows)
            {
                string[] ret;
                enum pre = `\\.\COM`;
                foreach (int n; 0 .. 255)
                {
                    auto i = n+1;
                    HANDLE p = CreateFileA(text(pre, i).toStringz,
                            GENERIC_READ | GENERIC_WRITE, 0, null,
                                           OPEN_EXISTING, 0, null);
                    if (p != INVALID_HANDLE_VALUE)
                    {
                        ret ~= text("COM", i);
                        CloseHandle(p);
                    }
                }
                return ret;
            }
        }
    }

    ///
    void write(const(void[]) arr, Duration timeout=500.dur!"usecs")
    {
        if (closed) throw new PortClosedException(port);

        size_t written = 0;

        StopWatch full;

        full.start();
        while (written < arr.length)
        {
            ptrdiff_t res;
            auto ptr = arr.ptr + written;
            auto len = arr.length - written;

            version (Posix)
            {
                res = posixWrite(handle, ptr, len);
                enforce(res >= 0, new WriteException(port));
            }
            version (Windows)
            {
                uint sres;
                enforce(WriteFile(handle, ptr, cast(uint)len, &sres, null),
                        new WriteException(port));
                res = sres;
            }

            written += res;

            if (full.peek().to!Duration > timeout)
                throw new TimeoutException(port);

            yield();
        }
    }

    ///
    void[] read(void[] arr, Duration timeout=1.dur!"seconds",
                            Duration frameGap=4.dur!"msecs")
    {
        if (closed) throw new PortClosedException(port);

        size_t readed = 0;

        StopWatch silence, full;

        full.start();
        while (true)
        {
            enforce(readed < arr.length);
            size_t res;

            auto ptr = arr.ptr + readed;
            auto len = arr.length - readed;

            version (Posix)
            {
                auto sres = posixRead(handle, ptr, len);
                enforce(sres >= 0, new ReadException(port));
                res = sres;
            }
            version (Windows)
            {
                uint sres;
                enforce(ReadFile(handle, ptr, cast(uint)len, &sres, null),
                        new ReadException(port));
                res = sres;
            }

            readed += res;

            if (res == 0)
            {
                if (readed > 0 && silence.peek().to!Duration > frameGap)
                    return arr[0..readed];

                if (!silence.running) silence.start();
            }
            else
            {
                silence.stop();
                silence.reset();
            }

            if (readed == 0 && full.peek().to!Duration > timeout)
                throw new TimeoutException(port);

            yield();
        }
    }

protected:

    version (Posix)
    {
        void setUintBaudRate(uint br)
        {
            version(usetermios2)
            {
                enum CBAUD  = octal!10017;
                enum BOTHER = octal!10000;

                termios2 opt2;
                enforce(ioctl(handle, TCGETS2, &opt2) != -1,
                        new SetupFailException(port, "can't get termios2 options"));
                opt2.c_cflag &= ~CBAUD; //Remove current BAUD rate
                opt2.c_cflag |= BOTHER; //Allow custom BAUD rate using int input
                opt2.c_ispeed = br;     //Set the input BAUD rate
                opt2.c_ospeed = br;     //Set the output BAUD rate
                ioctl(handle, TCSETS2, &opt2);
            }
            else
            {
                enforce(br in unixBaudList, new BaudRateUnsupportedException(br));

                auto baud = unixBaudList[br];

                termios opt;
                enforce(tcgetattr(handle, &opt) != -1,
                    new SerialPortException(port, "can't get termios options"));

                //cfsetispeed(&opt, B0);
                cfsetospeed(&opt, baud);

                enforce(tcsetattr(handle, TCSANOW, &opt) != -1,
                        new SerialPortException("Failed while call tcsetattr"));
            }
        }

        uint getUintBaudRate()
        {
            version (usetermios2)
            {
                termios2 opt2;
                enforce(ioctl(handle, TCGETS2, &opt2) != -1,
                        new SetupFailException(port, "can't get termios2 options"));
                return opt2.c_ospeed;
            }
            else
            {
                termios opt;
                enforce(tcgetattr(handle, &opt) != -1,
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
            handle = open(port.toStringz(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            enforce(handle != -1, new SetupFailException(port, "can't open port"));

            //enforce(fcntl(handle, F_SETFL, 0) != -1,  // disable O_NONBLOCK
            //        new SetupFailException(port, "can't disable O_NONBLOCK"));

            termios opt;
            enforce(tcgetattr(handle, &opt) != -1,
                new SetupFailException(port, "can't get termios options"));

            // make raw
            opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                             INLCR | IGNCR | ICRNL | IXON);
            opt.c_oflag &= ~OPOST;
            opt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            opt.c_cflag &= ~(CSIZE | PARENB);
            opt.c_cflag |= CS8;

            enforce(tcsetattr(handle, TCSANOW, &opt) != -1,
                    new SetupFailException("Failed while call tcsetattr"));
        }
        else version (Windows)
        {
            handle = CreateFileA((`\\.\` ~ port).toStringz,
                        GENERIC_READ | GENERIC_WRITE, 0, null,
                        OPEN_EXISTING, 0, null);

            if(handle is INVALID_HANDLE_VALUE)
                throw new SetupFailException(port, "can't CreateFileA");

            SetupComm(handle, 4096, 4096);
            PurgeComm(handle, PURGE_TXABORT | PURGE_TXCLEAR |
                              PURGE_RXABORT | PURGE_RXCLEAR);

            COMMTIMEOUTS tm;
            tm.ReadIntervalTimeout         = DWORD.max;
            tm.ReadTotalTimeoutMultiplier  = 0;
            tm.ReadTotalTimeoutConstant    = 0;
            tm.WriteTotalTimeoutMultiplier = 0;
            tm.WriteTotalTimeoutConstant   = 0;

            if (SetCommTimeouts(handle, &tm) == 0)
                throw new SetupFailException(port, "can't SetCommTimeouts");
        }

        config = conf;
    }
}

private bool hasFlag(A,B)(A a, B b) @property { return (a & b) == b; }
