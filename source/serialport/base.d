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

/++
 +/
abstract class SerialPortBase
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
        if (s.length == 0) throw new ParseModeException("empty config mode");
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

    /++ Params:
            port = port name
            conf = config of serialport
     +/
    this(string port, Config conf) { reopen(port, conf); }

    ~this() { close(); }

    /// Close handle
    void close() @nogc
    {
        if (closed) return;
        closeHandle(_handle);
        _handle = initHandle;
    }

    /// Port name
    string name() const @nogc @property { return port; }

    ///
    inout(SPHandle) handle() inout @nogc @property { return _handle; }

    ///
    void reopen(string np, Config cfg)
    {
        if (!closed) close();
        port = np;
        setup(cfg);
    }

    ///
    void reopen(string np) { reopen(np, config); }
    ///
    void reopen(Config cfg) { reopen(port, cfg); }
    ///
    void reopen() { reopen(port, config); }

    /++ Returns extend mode string (example: "/dev/ttyUSB0:38400:8N1")
     +/
    override string toString() const { return port ~ modeSplitChar ~ config.mode; }

    /++ Set config value
        Params:
            T = typeof of parameter, avalable:
                    int -> baudrate,
                    DataBit -> dataBits,
                    StopBits -> stopBits,
                    Parity -> parity
            val = value
     +/
    typeof(this) set(T)(T val) @nogc if (is(typeof(Config.init.set(val))))
    {
        Config tmp = config;
        tmp.set(val);
        config = tmp;
        return this;
    }

    /// Set config mode string
    typeof(this) set(string val)
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
    const(Config) config() @property const @nogc
    {
        if (closed) throwPortClosedException(port);

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
            else if (opt.c_cflag.hasFlag(CS5)) ret.dataBits = DataBits.data5;
            else throwSerialPortException(port, "unknown flags for databits");

            ret.stopBits = opt.c_cflag.hasFlag(CSTOPB) ? StopBits.two : StopBits.one;
        }
        version (Windows)
        {
            DCB cfg;
            GetCommState(cast(SPHandle)_handle, &cfg);

            ret.baudRate = cast(uint)cfg.BaudRate;

            switch (cfg.Parity)
            {
                case NOPARITY:   ret.parity = Parity.none; break;
                case ODDPARITY:  ret.parity = Parity.odd; break;
                case EVENPARITY: ret.parity = Parity.even; break;
                default: throwSerialPortException(port, "unknown parity"); break;
            }

            if (cfg.ByteSize < 5 || cfg.ByteSize > 8)
                throwSerialPortException(port, "unknown databist count");
            ret.dataBits = cast(DataBits)cfg.ByteSize;

            ret.stopBits = cfg.StopBits == ONESTOPBIT ? StopBits.one : StopBits.two;
        }

        return ret;
    }

    /// Set config
    void config(Config c) @property @nogc
    {
        if (closed) throwPortClosedException(port);

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
            final switch (c.dataBits) with (DataBits)
            {
                case data5: opt.c_cflag |= CS5; break;
                case data6: opt.c_cflag |= CS6; break;
                case data7: opt.c_cflag |= CS7; break;
                case data8: opt.c_cflag |= CS8; break;
            }

            m_tcsetattr(TCSANOW, &opt);

            const test = config;
            if (test.baudRate != c.baudRate) throwUnsupportedException(port, c.baudRate);
            if (test.parity   != c.parity)   throwUnsupportedException(port, c.parity);
            if (test.stopBits != c.stopBits) throwUnsupportedException(port, c.stopBits);
            if (test.dataBits != c.dataBits) throwUnsupportedException(port, c.dataBits);
        }
        version (Windows)
        {
            DCB cfg;
            GetCommState(_handle, &cfg);

            if (cfg.BaudRate != cast(DWORD)c.baudRate)
            {
                cfg.BaudRate = cast(DWORD)c.baudRate;
                if (!SetCommState(_handle, &cfg))
                    throwUnsupportedException(port, c.baudRate);
            }

            auto tmpParity = NOPARITY;
            if (c.parity == Parity.odd) tmpParity = ODDPARITY;
            if (c.parity == Parity.even) tmpParity = EVENPARITY;

            if (cfg.Parity != tmpParity)
            {
                cfg.Parity = cast(ubyte)tmpParity;
                if (!SetCommState(_handle, &cfg))
                    throwUnsupportedException(port, c.parity);
            }

            auto tmpStopBits = ONESTOPBIT;
            if (c.stopBits == StopBits.two) tmpStopBits = TWOSTOPBITS;

            if (cfg.StopBits != tmpStopBits)
            {
                cfg.StopBits = cast(ubyte)tmpStopBits;
                if (!SetCommState(_handle, &cfg))
                    throwUnsupportedException(port, c.stopBits);
            }

            if (cfg.ByteSize != cast(typeof(cfg.ByteSize))c.dataBits)
            {
                cfg.ByteSize = cast(typeof(cfg.ByteSize))c.dataBits;
                if (!SetCommState(_handle, &cfg))
                    throwUnsupportedException(port, c.dataBits);
            }
        }
    }

    @property @nogc
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

    /++ List of available serial ports in system
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

protected:
    void[] m_read(void[] buf) @nogc
    {
        // non-blocking algorithm
        if (closed) throwPortClosedException(port);

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
                else throwReadException(port, "posix read", errno);
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
                else throwReadException(port, "win read", err);
            }
            res = sres;
        }

        return buf[0..res];
    }

    size_t m_write(string arr) @nogc
    {
        // non-blocking algorithm
        if (closed) throwPortClosedException(port);

        auto ptr = arr.ptr;
        auto len = arr.length;

        version (Posix)
        {
            ptrdiff_t res = posixWrite(_handle, ptr, len);
            if (res < 0)
            {
                if (errno == EAGAIN) res = 0; // buffer filled
                else throwWriteException(port, "posix write", errno);
            }
        }
        version (Windows)
        {
            uint res;
            auto wfr = WriteFile(_handle, ptr, cast(uint)len, &res, null);
            if (!wfr)
            {
                auto err = GetLastError();
                if (err == ERROR_IO_PENDING) res = 0;
                else throwWriteException(port, "win write", err);
            }
        }

        return res;
    }

    /// open handler, set new config
    void setup(Config conf)
    {
        if (port.length == 0)
            throwSerialPortException("", "zero length name");

        version (Posix) posixSetup(conf);
        else winSetup();

        config = conf;
    }

    version (Posix)
    {
        void m_tcgetattr(termios* t) const @nogc
        {
            if (tcgetattr(_handle, t) == -1)
                throwSysCallException(port, "tcgetattr", errno);
        }

        void m_tcsetattr(int v, const(termios*) t) inout @nogc
        {
            if (tcsetattr(_handle, v, t) == -1)
                throwSysCallException(port, "tcsetattr", errno);
        }

        version (usetermios2)
        {
            void m_ioctl(int v, termios2* t) inout
            {
                if (ioctl(_handle, v, t) == -1)
                    throwSysCallException(port, "ioctl", errno);
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
                throwSysCallException(port, "open", errno);
        }

        /// Set termios.c_cc[VMIN] and .c_cc[VMAX]
        void setCC(ubyte[2] val) @nogc
        {
            termios opt;
            m_tcgetattr(&opt);
            opt.c_cc[VMIN] = val[0];
            opt.c_cc[VTIME] = val[1];
            m_tcsetattr(TCSADRAIN, &opt);
        }

        /// Get termios.c_cc[VMIN] and .c_cc[VMAX]
        ubyte[2] getCC() @nogc
        {
            ubyte[2] ret;
            termios opt;
            m_tcgetattr(&opt);
            ret[0] = opt.c_cc[VMIN];
            ret[1] = opt.c_cc[VTIME];
            return ret;
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
            opt.c_cc[VMIN] = 0;
            opt.c_cc[VTIME] = 0;

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

        void setUintBaudRate(uint br) @nogc
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
                    throwUnsupportedException(port, br);

                auto baud = unixBaudList.firstA2B(br, B0);

                termios opt;
                m_tcgetattr(&opt);
                cfsetispeed(&opt, B0);
                cfsetospeed(&opt, baud);
                m_tcsetattr(TCSANOW, &opt);
            }
        }

        uint getUintBaudRate() const @nogc
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
                throwSysCallException(port, "CreateFileA", GetLastError());

            SetupComm(_handle, 4096, 4096);
            PurgeComm(_handle, PURGE_TXABORT | PURGE_TXCLEAR |
                               PURGE_RXABORT | PURGE_RXCLEAR);
            
            updTimeouts();
        }

        void updTimeouts() @nogc
        {
            setTimeouts(DWORD.max, 0, 0, 0, 0);
        }

        void setTimeouts(DWORD rit, DWORD rttm, DWORD rttc, DWORD wttm, DWORD wttc) @nogc
        {
            COMMTIMEOUTS tm;
            tm.ReadIntervalTimeout         = rit;
            tm.ReadTotalTimeoutMultiplier  = rttm;
            tm.ReadTotalTimeoutConstant    = rttc;
            tm.WriteTotalTimeoutMultiplier = wttm;
            tm.WriteTotalTimeoutConstant   = wttc;

            if (SetCommTimeouts(_handle, &tm) == 0)
                throwSysCallException(port, "SetCommTimeouts", GetLastError());
        }
    }
}

/++ Timed work with serial port

 +/
abstract class SerialPort : SerialPortBase
{
protected:

    Duration _writeTimeout = 1.seconds,
             _writeTimeoutMult = Duration.zero,
             _readTimeout = 1.seconds,
             _readTimeoutMult = Duration.zero;


    void updateTimeouts() @nogc {}
    
public:

    /++ Construct SerialPort

        See_Also: SerialPortBase.this
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

    /++ Read data from serial port while exists
     +/
    void flush()
    {
        void[128] buf = void;
        const rt = _readTimeout;
        const rtm = _readTimeoutMult;

        _readTimeout = 10.msecs;
        _readTimeoutMult = Duration.zero;
        updateTimeouts();

        version (Posix)
        {
            const last = getCC();
            setCC([0,0]);
        }

        void[] tmp;
        do tmp = read(buf, CanRead.zero);
        while (tmp.length);

        version (Posix)
        {
            setCC(last);
        }

        _readTimeout = rt;
        _readTimeoutMult = rtm;
        updateTimeouts();
    }

    @property @nogc
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

    ///
    enum CanRead
    {
        allOrNothing, ///
        anyNonZero, ///
        zero ///
    }

    /++ Read data from port

        Receive data time schema:

        ------
        ---|-------|--------------|-------|--> t
         call      |              |       |
         read      |              |       |
           |       |              |       |
           |       |<data receive process>|
           |       |=====   ====  | ======|
           |       |              |
           |       |<-readedData->|
           |                      | 
           |<---readTimeoutSum--->|
           |                    return
           |<---read work time--->|
        ------

        where `readTimeoutSum = readTimeout + readTimeoutMult * dataBuffer.length;`

        if canReturn is:
        
        CanRead.allOrNothing

        ---
        if (readedData.length < dataBuffer.length)
            throw TimeoutException(port);
        else return readedData;
        ---

        CanReturn.anyNonZero

        ---
        if (readedData.length == 0)
            throw TimeoutException(port);
        else return readedData;
        ---

        CanReturn.zero

        ---
        return readedData;
        ---

        Params:
            buf = preallocated buffer for reading
            cr = flag what define behavior if readedData.length < buf.length
                 then readTimeoutSum is expires

        Returns: slice of buf with readed data
        Throws:
            PortClosedException if port closed
            ReadException if read error occurs
            TimeoutException if timeout expires
     +/
    abstract void[] read(void[] buf, CanRead cr=CanRead.allOrNothing) @nogc;

    ///
    protected void checkAbility(CanRead cr, size_t readed, size_t buffer) @nogc
    {
        bool err;

        final switch (cr) with(CanRead)
        {
            case allOrNothing: err = readed != buffer; break;
            case anyNonZero:   err = readed == 0; break;
            case zero: /+ no errors +/ break;
        }

        if (err) throwTimeoutException(port, "read timeout");
    }

    /++ Write data to port

        Params:
            arr = data for writing

        Throws:
            PortClosedException if port closed
            WriteException if read error occurs
            TimeoutException if timeout expires
     +/
    abstract size_t write(string buf) @nogc;
}