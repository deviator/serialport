///
module serialport.base;

package import std.algorithm;
package import std.array;
package import std.conv : to, text;
package import std.exception;
package import std.experimental.logger;
package import std.path;
package import std.string;
package import std.datetime.stopwatch : StopWatch, AutoStart;
package import core.time;

package import serialport.config;
package import serialport.exception;
package import serialport.types;
package import serialport.util;

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
    this(string port, Config conf) { reopen(port, conf); }

    ~this() { close(); }

    /// close handle
    void close() @nogc
    {
        if (closed) return;
        closeHandle(_handle);
        _handle = initHandle;
    }

    ///
    string name() const @nogc @property { return port; }

    ///
    inout(SPHandle) handle() inout @nogc @property { return _handle; }

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
    bool closed() const @property @nogc nothrow
    {
        version (Posix) return _handle == initHandle;
        version (Windows) return _handle is initHandle;
    }

    /++ Get config

        const for disallow `com.config.set(value)`
        use `com.set(value)` instead
     +/
    const(Config) config() @property const
    {
        if (closed) throw new PortClosedException(port);

        Config ret;

        version (Posix)
        {
            termios opt;
            m_tcgetattr(&opt);

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
            GetCommState(cast(SPHandle)_handle, &cfg);

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
        if (closed) throw new PortClosedException(port);

        version (Posix)
        {
            setUintBaudRate(c.baudRate);

            termios opt;
            m_tcgetattr(&opt);

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

            m_tcsetattr(TCSANOW, &opt);

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
                enforce(SetCommState(_handle, &cfg), new UE(c.baudRate));
            }

            const tmpParity = [Parity.none: NOPARITY, Parity.odd: ODDPARITY,
                               Parity.even: EVENPARITY][c.parity];

            if (cfg.Parity != tmpParity)
            {
                cfg.Parity = cast(ubyte)tmpParity;
                enforce(SetCommState(_handle, &cfg), new UE(c.parity));
            }

            const tmpStopBits = [StopBits.one: ONESTOPBIT,
                                 StopBits.onePointFive: ONESTOPBIT,
                                 StopBits.two: TWOSTOPBITS][c.stopBits];

            if (cfg.StopBits != tmpStopBits)
            {
                cfg.StopBits = cast(ubyte)tmpStopBits;
                enforce(SetCommState(_handle, &cfg), new UE(c.stopBits));
            }

            if (cfg.ByteSize != cast(typeof(cfg.ByteSize))c.dataBits)
            {
                cfg.ByteSize = cast(typeof(cfg.ByteSize))c.dataBits;
                enforce(SetCommState(_handle, &cfg), new UE(c.dataBits));
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
        version (linux)
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
        version (OSX)
        {
            return dirEntries("/dev/", "{tty,cu}*", SpanMode.shallow)
                    .map!(a=>a.name).array;
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
        // non-blocking algorithm
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
        // non-blocking algorithm
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

protected:

    /// open handler, set new config
    void setup(Config conf)
    {
        if (port.length == 0)
            throw new SerialPortException("zero length name");

        version (Posix) posixSetup(conf);
        else winSetup();

        config = conf;
    }

    version (Posix)
    {
        void m_tcgetattr(termios* t) const
        {
            if (tcgetattr(_handle, t) == -1)
                throw new SysCallException("tcgetattr", errno);
        }

        void m_tcsetattr(int v, const(termios*) t) inout
        {
            if (tcsetattr(_handle, v, t) == -1)
                throw new SysCallException("tcsetattr", errno);
        }

        version (usetermios2)
        {
            void m_ioctl(int v, termios2* t) inout
            {
                if (ioctl(_handle, v, t) == -1)
                    throw new SysCallException("ioctl", errno);
            }
        }

        void posixSetup(Config conf)
        {
            openPort();
            initialConfig(conf);
        }

        void openPort()
        {
            _handle = open(port.toStringz(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (_handle == -1)
                throw new SysCallException("open", errno);
        }

        void initialConfig(Config conf)
        {
            termios opt;
            m_tcgetattr(&opt);

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

            m_tcsetattr(TCSANOW, &opt);
        }

        void setUintBaudRate(uint br)
        {
            version (usetermios2)
            {
                import std.conv : octal;
                enum CBAUD  = octal!10017;
                enum BOTHER = octal!10000;

                termios2 opt2;
                m_ioctl(TCGETS2, &opt2);
                opt2.c_cflag &= ~CBAUD; //Remove current BAUD rate
                opt2.c_cflag |= BOTHER; //Allow custom BAUD rate using int input
                opt2.c_ispeed = br;     //Set the input BAUD rate
                opt2.c_ospeed = br;     //Set the output BAUD rate
                m_ioctl(TCSETS2, &opt2);
            }
            else
            {
                if (unixBaudList.countA(br) == 0)
                    throw new UnsupportedException(br);

                auto baud = unixBaudList.firstA2B(br, B0);

                termios opt;
                m_tcgetattr(&opt);
                cfsetispeed(&opt, B0);
                cfsetospeed(&opt, baud);
                m_tcsetattr(TCSANOW, &opt);
            }
        }

        uint getUintBaudRate() const
        {
            version (usetermios2)
            {
                termios2 opt2;
                m_ioctl(TCGETS2, &opt2);
                return opt2.c_ospeed;
            }
            else
            {
                termios opt;
                m_tcgetattr(&opt);
                version (OSX) alias true_speed_t = uint;
                else alias true_speed_t = typeof(cfgetospeed(&opt));
                auto b = cast(true_speed_t)cfgetospeed(&opt);
                return unixBaudList.firstB2A(b, 0);
            }
        }
    }

    version (Windows)
    {
        void winSetup()
        {
            auto fname = `\\.\` ~ port;
            _handle = CreateFileA(fname.toStringz,
                        GENERIC_READ | GENERIC_WRITE, 0, null,
                        OPEN_EXISTING, 0, null);

            if (_handle is INVALID_HANDLE_VALUE)
                throw new SysCallException("CreateFileA", GetLastError());

            SetupComm(_handle, 4096, 4096);
            PurgeComm(_handle, PURGE_TXABORT | PURGE_TXCLEAR |
                               PURGE_RXABORT | PURGE_RXCLEAR);
            
            updTimeouts();
        }

        void updTimeouts()
        {
            setTimeouts(DWORD.max, 0, 0, 0, 0);
        }

        void setTimeouts(DWORD rit, DWORD rttm, DWORD rttc, DWORD wttm, DWORD wttc)
        {
            COMMTIMEOUTS tm;
            tm.ReadIntervalTimeout         = rit;
            tm.ReadTotalTimeoutMultiplier  = rttm;
            tm.ReadTotalTimeoutConstant    = rttc;
            tm.WriteTotalTimeoutMultiplier = wttm;
            tm.WriteTotalTimeoutConstant   = wttc;

            if (SetCommTimeouts(_handle, &tm) == 0)
                throw new SysCallException("SetCommTimeouts", GetLastError());
        }
    }
}

///
abstract class SerialPortTm : SerialPort
{
protected:

    Duration _writeTimeout = 1.seconds,
             _writeTimeoutMult = Duration.zero,
             _readTimeout = 1.seconds,
             _readTimeoutMult = Duration.zero;


    void updateTimeouts() {}
    
public:

    /++ Construct SerialPortTm

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

    ///
    void flush()
    {
        void[128] buf = void;
        auto rt = _readTimeout;
        auto rtm = _readTimeoutMult;

        _readTimeout = 10.msecs;
        _readTimeoutMult = Duration.zero;
        updateTimeouts();

        try while(true) read(buf); catch (TimeoutException e) {}

        _readTimeout = rt;
        _readTimeoutMult = rtm;
        updateTimeouts();
    }

    @property
    {
        const
        {
            ///
            Duration readTimeout() { return _readTimeout; }
            ///
            Duration readTimeoutMult() { return _readTimeoutMult; }
            ///
            Duration writeTimeout() { return _writeTimeout; }
            ///
            Duration writeTimeoutMult() { return _writeTimeoutMult; }
        }

        ///
        void readTimeout(Duration tm)
        {
            _readTimeout = tm;
            updateTimeouts();
        }

        ///
        void readTimeoutMult(Duration tm)
        {
            _readTimeoutMult = tm;
            updateTimeouts();
        }

        ///
        void writeTimeout(Duration tm)
        {
            _writeTimeout = tm;
            updateTimeouts();
        }

        ///
        void writeTimeoutMult(Duration tm)
        {
            _writeTimeout = tm;
            updateTimeouts();
        }
    }
}