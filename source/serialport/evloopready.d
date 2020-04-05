module serialport.evloopready;

import serialport.base;

/++ Serial Port Event Loop Ready
 +/
class SerialPortEL : SerialPort
{
protected:
    EvLoopHook evloop;

public:

    interface EvLoopHook
    {
        void beforeCloseHandle(SPHandle);
        void afterOpenHandle(SPHandle);

        void wakeOnRead(bool, Duration tm=Duration.zero);
        void wakeOnWrite(bool, Duration tm=Duration.zero);

        void wait();
    }

    ///
    this(EvLoopHook e, string exmode)
    {
        evloop = enforce(e, "event loop hook is null");
        super(exmode);
    }

    ///
    this(EvLoopHook e, string port, string mode)
    {
        evloop = enforce(e, "event loop hook is null");
        super(port, Config.parse(mode));
    }

    ///
    this(EvLoopHook e, string port, uint baudRate)
    {
        evloop = enforce(e, "event loop hook is null");
        super(port, Config(baudRate));
    }

    ///
    this(EvLoopHook e, string port, uint baudRate, string mode)
    {
        evloop = enforce(e, "event loop hook is null");
        super(port, Config(baudRate).set(mode));
    }

    ///
    this(EvLoopHook e, string port, Config conf)
    {
        evloop = enforce(e, "event loop hook is null");
        super(port, conf);
    }

    override void close()
    {
        if (closed) return;
        evloop.beforeCloseHandle(_handle);
        closeHandle(_handle);
        _handle = initHandle;
    }

    override void reopen(string np, Config cfg)
    {
        super.reopen(np, cfg);
        evloop.afterOpenHandle(_handle);
    }

    override void[] read(void[] buf, CanRead cr=CanRead.allOrNothing)
    {
        if (closed) throwPortClosedException(port);

        size_t res;
        const timeout = buf.length * readTimeoutMult + readTimeout;
        const sw = StopWatch(AutoStart.yes);

        evloop.wakeOnRead(true, timeout);
        scope (exit) evloop.wakeOnRead(false);

        import std.stdio;
        while (sw.peek < timeout)
        {
            res += m_read(buf[res..$]).length;
            if (res == buf.length) return buf[];

            evloop.wait();
            stderr.writeln("WAKEUP IN SERIALPORT.READ !!!!!!!!!");
        }
        stderr.writeln("TIMEOUT !!!!!!!!!");

        checkAbility(cr, res, buf.length);

        return buf[0..res];
    }

    override void write(const(void[]) arr)
    {
        if (closed) throwPortClosedException(port);

        size_t written;
        const timeout = arr.length * writeTimeoutMult + writeTimeout;
        const sw = StopWatch(AutoStart.yes);

        evloop.wakeOnWrite(true, timeout);
        scope (exit) evloop.wakeOnWrite(false);

        while (sw.peek < timeout)
        {
            written += m_write(arr[written..$]);
            if (written == arr.length) return;

            evloop.wait();
        }

        throwTimeoutException(port, "write timeout");
    }
}