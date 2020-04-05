module select_loop;

import core.sys.posix.sys.select;

import base;

enum EvType
{
    ERR = 1,
    READ = 2,
    WRITE = 4
}

alias Callback = void delegate(uint mask);

struct FDData
{
    bool read;
    bool write;
    Callback cb;
    int repeat;
}

class EvLoop
{
    private bool isRun;

    FDData[int] fds;

    this()
    {
        isRun = true;
    }

    bool step()
    {
        fd_set read_set, write_set, err_set;

        FD_ZERO(&read_set);
        FD_ZERO(&write_set);
        FD_ZERO(&err_set);

        int nfds = 0;

        foreach (fd, data; fds)
        {
            //mlogf("SET %d %s", fd, data);
            if (data.repeat == 0) continue;

            if (data.read)
            {
                FD_SET(fd, &read_set);
                if (fd > nfds) nfds = fd;
            }
            if (data.write)
            {
                FD_SET(fd, &write_set);
                if (fd > nfds) nfds = fd;
            }
            FD_SET(fd, &err_set);
        }

        nfds++;

        //mlog("SELECT");
        check!select(nfds, &read_set, &write_set, &err_set, null);
        //mlog("   SELECT FIN");

        foreach (fd, data; fds)
        {
            //mlogf("CHECK FD %d %s", fd, data);
            const setted = FD_ISSET(fd, &read_set) ||
                FD_ISSET(fd, &write_set) ||
                FD_ISSET(fd, &err_set);
            if (setted)
            {
                //mlogf("      setted %d", fd);
                if (data.repeat > 0)
                    data.repeat--;
            }

            if (FD_ISSET(fd, &err_set))
            {
                //mlogf("         as err %d", fd);
                data.cb(EvType.ERR);
                continue;
            }

            if (setted)
            {
                //mlogf("    CALL BACK %d", fd);
                data.cb( (FD_ISSET(fd, &read_set)  ? EvType.READ  : 0 ) |
                        (FD_ISSET(fd, &write_set) ? EvType.WRITE : 0) );
                //mlogf("    CHECK FD %d FIN", fd);
            }
        }

        return isRun;
    }

    void stop() { isRun = false; }

    void run() { while (step()) {} }

    void addFD(int fd, FDData data) { fds[fd] = data; }
    void setFD(int fd, FDData data) { fds[fd] = data; }
    void delFD(int fd) { fds.remove(fd); }
}

private EvLoop evloop;

static this()
{
    evloop = new EvLoop;
}

EvLoop defaultLoop() @property
{
    if (evloop is null)
        throw new Exception("try get default loop before module ctor call");
    return evloop;
}

class Timer
{
    int fd;
    FDData data;

    void delegate(Timer) func;

    this(void delegate() fnc) { this((t){ fnc(); }); }

    this(void delegate(Timer) fnc)
    {
        func = fnc;

        fd = check!timerfd_create(CLOCK_MONOTONIC, 0);
        mlogf("TIMER %d", fd);
        const it = itimerspec(timespec(0,0), timespec(0,0));
        check!timerfd_settime(fd, 0, &it, null);

        data = FDData(true, false, &onTriggered, 1);

        evloop.addFD(fd, data);
    }

    void set(Duration d)
    {
        timespec t;
        d.split!("seconds", "nsecs")(t.tv_sec, t.tv_nsec);
        auto it = itimerspec(timespec(0,0), t);
        check!timerfd_settime(fd, 0, &it, null);

        data.repeat = d > Duration.zero ? 1 : 0;
        evloop.setFD(fd, data);
    }

    private void onTriggered(uint mask)
    {
        ulong v;
        check!read(fd, &v, v.sizeof);
        assert(func !is null, "timer null func");
        func(this);
    }

    void close() { .close(fd); }
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
        timer = new Timer(&exec);

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

    WrapFD makeNewWrapFD() { return new WrapFD(this); }

    void exec() { fiber.step(); }
    void finish() { timer.close(); }

    WrapFD currWaker;
}

class WrapFD
{
    int fd = -1;
    FDData data;
    Worker wrkr;

    this(Worker w)
    {
        wrkr = w;
        data.cb = &call;
        data.read = true;
        data.write = true;
        data.repeat = 1;
    }

    void beforeCloseHandle(int h)
    {
        assert(h == fd, "wrong fd for WrapFD.beforeCloseHandle");
        evloop.delFD(fd);
    }

    void afterOpenHandle(int h)
    {
        assert(fd == -1, "WrapFD is inited");
        fd = h;
        data.repeat = 0;
        evloop.addFD(fd, data);
        mlogf("WRAP %d", fd);
    }

    void wakeOnIO(bool wake, Duration timeout)
    {
        data.repeat = wake ? -1 : 0;
        evloop.setFD(fd, data);
        wrkr.currWaker = wake ? this :
                         wrkr.currWaker == this ? null :
                         wrkr.currWaker;
        wrkr.timer.set(timeout);
    }

    void call(uint evmask)
    {
        if (evmask & EvType.ERR) mlog("SELECT ERR");

        // can read
        if (evmask & EvType.READ)
            if (wrkr.currWaker == this)
                wrkr.exec();

        // can write
        if (evmask & EvType.WRITE)
            if (wrkr.currWaker == this)
                wrkr.exec();
    }
}