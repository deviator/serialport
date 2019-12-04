///
module serialport.block;

import serialport.base;

/// Blocking work serialport
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

    override void[] read(void[] buf, CanRead cr=CanRead.allOrNothing)
    {
        if (closed) throwPortClosedException(port);

        version (Posix)
        {
            if (cr == CanRead.allOrNothing)
                setCC([cast(ubyte)min(buf.length, 255), 0]);
            else setCC([1, 0]);

            ssize_t res = 0;
            auto ttm = buf.length * readTimeoutMult + readTimeout;
            auto sw = StopWatch(AutoStart.yes);
            while (ttm > Duration.zero)
            {
                ttm -= sw.peek;
                sw.reset();

                fd_set sset;
                FD_ZERO(&sset);
                FD_SET(_handle, &sset);

                timeval ctm;
                ctm.tv_sec = cast(int)(ttm.total!"seconds");
                enum US_PER_MS = 1000;
                ctm.tv_usec = cast(int)(ttm.split().msecs * US_PER_MS);

                const rv = select(_handle + 1, &sset, null, null, &ctm);
                if (rv < 0) throwSysCallException(port, "select", errno);

                if (rv == 0) break;

                const r = posixRead(_handle, buf.ptr+res, buf.length-res);
                if (r < 0) throwReadException(port, "posix read", errno);
                res += r;
                if (res == buf.length) return buf;
            }
        }
        else
        {
            uint res;

            if (!ReadFile(handle, buf.ptr, cast(uint)buf.length, &res, null))
                throwReadException(port, "win read", GetLastError());
        }

        checkAbility(cr, res, buf.length);

        return buf[0..res];
    }

    override size_t write(string arr)
    {
        if (closed) throwPortClosedException(port);

        version (Posix)
        {
            size_t written;
            const ttm = arr.length * writeTimeoutMult + writeTimeout;
            const full = StopWatch(AutoStart.yes);
            while (written < arr.length)
            {
                if (full.peek > ttm) {
                    throwTimeoutException(port, "write timeout");
                }

                const res = posixWrite(_handle, arr[written..$].ptr, arr.length - written);

                if (res < 0) {
                    throwWriteException(port, "posix write", errno);
                }

                written += res;
            }
            return written;
        }
        else
        {
            uint written;

            if (!WriteFile(_handle, arr.ptr, cast(uint)arr.length, &written, null)) {
                throwWriteException(port, "win write", GetLastError());
            }

            if (arr.length != written) {
                throwTimeoutException(port, "write timeout");
            }
 
            return written;
        }
    }

protected:

    override void[] m_read(void[]) @nogc
    { assert(0, "disable m_read for blocking"); }
    override size_t m_write(string) @nogc
    { assert(0, "disable m_write for blocking"); }

    override void updateTimeouts() @nogc { version (Windows) updTimeouts(); }

    version (Windows)
    {
        override void updTimeouts() @nogc
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
                throwSysCallException(port, "fcntl", errno);

            setCC([1,0]);

            initialConfig(conf);
        }
    }
}
