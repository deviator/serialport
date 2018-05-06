///
module serialport.block;

import serialport.base;

///
class SerialPortBlk : SerialPort
{
public:
    /++ Construct SerialPortBlk

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
            version (Windows) updTimeouts();
        }

        ///
        void readTimeoutMult(Duration tm)
        {
            _readTimeoutMult = tm;
            version (Windows) updTimeouts();
        }

        ///
        void writeTimeout(Duration tm)
        {
            _writeTimeout = tm;
            version (Windows) updTimeouts();
        }

        ///
        void writeTimeoutMult(Duration tm)
        {
            _writeTimeout = tm;
            version (Windows) updTimeouts();
        }
    }

    override void[] read(void[] buf)
    {
        if (closed) throw new PortClosedException(port);

        version (Posix)
        {
            fd_set sset;
            FD_ZERO(&sset);
            FD_SET(_handle, &sset);

            Duration ttm = buf.length * readTimeoutMult + readTimeout;

            timeval ctm;
            ctm.tv_sec = cast(int)(ttm.total!"seconds");
            enum US_PER_MS = 1000;
            ctm.tv_usec = cast(int)(ttm.split().msecs * US_PER_MS);

            const rv = select(_handle + 1, &sset, null, null, &ctm);
            if (rv == -1)
                throw new SysCallException("select", errno);
            else if (rv == 0)
                throw new TimeoutException(port);

            const res = posixRead(handle, buf.ptr, buf.length);
            if (res < 0)
                throw new ReadException(port, text("errno ", errno));
        }
        else
        {
            uint res;

            if (!ReadFile(handle, buf.ptr, cast(uint)buf.length, &res, null))
                throw new ReadException(port, text("error ", GetLastError()));

            if (res == 0)
                throw new TimeoutException(port);
        }

        return buf[0..res];
    }

    override ptrdiff_t write(const(void[]) arr)
    {
        if (closed) throw new PortClosedException(port);

        version (Posix)
        {
            size_t written;
            auto ttm = arr.length * writeTimeoutMult + writeTimeout;
            const full = StopWatch(AutoStart.yes);
            while (written < arr.length)
            {
                if (full.peek > ttm)
                    throw new TimeoutException(port);

                auto res = posixWrite(_handle, arr[written..$].ptr, arr.length - written);

                if (res < 0)
                    throw new WriteException(port, text("errno ", errno));

                written += res;
            }
        }
        else
        {
            uint written;

            if (!WriteFile(_handle, arr.ptr, cast(uint)arr.length, &written, null))
                throw new WriteException(port, text("error ", GetLastError()));

            if (arr.length != written)
                throw new TimeoutException(port);
        }

        return written;
    }

protected:

    Duration _writeTimeout = 1.seconds,
             _writeTimeoutMult = Duration.zero,
             _readTimeout = 1.seconds,
             _readTimeoutMult = Duration.zero
             ;

    version (Windows)
    {
        override void updTimeouts()
        {
            setTimeouts(0, cast(DWORD)readTimeoutMult.total!"msecs",
                           cast(DWORD)readTimeout.total!"msecs",
                           cast(DWORD)writeTimeoutMult.total!"msecs",
                           cast(DWORD)writeTimeout.total!"msecs");
        }
    }

    version (Posix)
    {
        override void posixSetup(Config conf)
        {
            openPort();

            if (fcntl(_handle, F_SETFL, 0) == -1)  // disable O_NONBLOCK
                throw new SysCallException("fcntl", errno);

            initialConfig(conf);
        }
    }
}