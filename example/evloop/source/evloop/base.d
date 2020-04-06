module evloop.base;

public
{
    import std.datetime : Duration;
    import std.experimental.logger;
    import std.exception : enforce;

    import core.thread : Fiber;

    import core.sys.posix.unistd;
    import core.sys.linux.sys.signalfd;
    import core.stdc.errno;

    import evloop.errproc;
}

alias Callback = void delegate(uint);

enum EvType
{
    ERR = 1,
    READ = 2,
    WRITE = 4
}

struct FDData
{
    bool read;
    bool write;
    Callback* cb;
    int repeat;
}

interface Waker
{
    void beforeCloseHandle(int h);
    void afterOpenHandle(int h);
    void wakeOnRead(bool w, Duration t);
    void wakeOnWrite(bool w, Duration t);
}

interface Timer
{
    void set(Duration d);
    void close();
}

interface EvLoop
{
    void addFD(int fd, FDData data);
    void modFD(int fd, FDData data);
    void delFD(int fd);

    bool step();
    void stop();
    void run();

    Waker makeNewWaker(Worker);

    final Timer makeNewTimer(void delegate() fnc)
    { return makeNewTimer((t){ fnc(); }); }

    Timer makeNewTimer(void delegate(Timer) fnc);
}

package EvLoop evl;

EvLoop defaultLoop() @property
{
    if (evl is null)
        throw new Exception("try get default loop before module ctor call");
    return evl;
}

class WFiber : Fiber
{
    Timer timer;

    this(Timer t, void delegate() dlg)
    {
        timer = t;
        super(dlg);
    }

    static WFiber getWThis()
    {
        auto f = enforce(Fiber.getThis, "not in fiber");
        return enforce(cast(WFiber)f, "not WFiber");
    }

    static void sleep(Duration d) { getWThis().sleepImpl(d); }

    private void sleepImpl(Duration d)
    {
        timer.set(d);
        yield();
    }

    void step()
    {
        if (state == State.TERM)
            reset();
        call();
    }
}

class Worker
{
    Timer timer;
    WFiber fiber;
    Duration period;

    void delegate() execFunc;

    this()
    {
        timer = evl.makeNewTimer(&exec);

        fiber = new WFiber(timer,
        {
            enforce(execFunc !is null, "null exec func");
            execFunc();
            if (period > Duration.zero)
                timer.set(period);
        });
    }

    void setExecFunc(void delegate() dlg)
    {
        enforce(dlg !is null, "can't set null dlg");
        execFunc = dlg;
    }

    void run(void delegate() dlg)
    {
        setExecFunc(dlg);
        exec();
    }

    void exec() { fiber.step(); }
    void finish() { timer.close(); }

    Waker curr_w;

    Waker currentWaker() @property { return curr_w; }
    void setWaker(Waker w) { curr_w = w; }
    void unsetWaker(Waker w) { if (curr_w is w) curr_w = null; }

    Waker makeNewWaker() { return evl.makeNewWaker(this); }
}

class BaseWaker : Waker
{
    int fd = -1;
    FDData data;
    Worker wrkr;
    void delegate(uint) cb;

    this(Worker w)
    {
        wrkr = w;
        cb = &call;
        data.cb = &cb;
        data.read = true;
        data.write = true;
        data.repeat = 1;
    }

    override
    {
        void beforeCloseHandle(int h)
        {
            assert(h == fd, "wrong fd for WrapFD.beforeCloseHandle");
            evl.delFD(fd);
        }

        void afterOpenHandle(int h)
        {
            assert(fd == -1, "WrapFD is inited");
            fd = h;
            data.repeat = 0;
            evl.addFD(fd, data);
        }

        void wakeOnRead(bool wake, Duration timeout)
        {
            data.read = true;
            data.write = false;
            data.repeat = wake ? -1 : 0;
            wakeOnIO(wake, timeout);
        }

        void wakeOnWrite(bool wake, Duration timeout)
        {
            data.read = false;
            data.write = true;
            data.repeat = wake ? -1 : 0;
            wakeOnIO(wake, timeout);
        }
    }

    void wakeOnIO(bool wake, Duration timeout)
    {
        evl.modFD(fd, data);
        if (wake) wrkr.setWaker(this);
        else wrkr.unsetWaker(this);
        wrkr.timer.set(timeout);
    }

    void call(uint evmask)
    {
        if (evmask & EvType.ERR) mlog("SELECT ERR");

        // can read
        if (evmask & EvType.READ)
            if (wrkr.currentWaker == this)
                wrkr.exec();

        // can write
        if (evmask & EvType.WRITE)
            if (wrkr.currentWaker == this)
                wrkr.exec();
    }
}
