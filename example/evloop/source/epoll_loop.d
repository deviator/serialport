module epoll_loop;

import core.sys.linux.epoll;

import base;

alias Callback = void delegate(uint);

class EvLoop
{
    private bool isRun;

    int epfd;
    epoll_event[256] events;

    this()
    {
        epfd = check!epoll_create1(EPOLL_CLOEXEC);
        isRun = true;
    }

    bool step()
    {
        int nfds = check!epoll_wait(epfd,
            events.ptr, cast(int)events.length, -1);

        foreach (i; 0 .. nfds)
        {
            auto e = events[i];
            auto cb = cast(Callback*)(e.data.ptr);
            if (cb is null)
            {
                error("have null ptr in event data");
                continue;
            }
            (*cb)(e.events);
        }
        return isRun;
    }

    void stop() { isRun = false; }

    void run() { while (step()) {} }
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
    Callback cb;
    epoll_event ev;

    void delegate(Timer) func;

    this(void delegate() fnc) { this((t){ fnc(); }); }

    this(void delegate(Timer) fnc)
    {
        func = fnc;

        fd = check!timerfd_create(CLOCK_MONOTONIC, 0);

        cb = &onTriggered;
        ev.data.ptr = cast(void*)&cb;
        ev.events = EPOLLIN | EPOLLONESHOT;

        const it = itimerspec(timespec(0,0), timespec(0,0));
        check!timerfd_settime(fd, 0, &it, null);

        check!epoll_ctl(evloop.epfd, EPOLL_CTL_ADD, fd, &ev);
    }

    void set(Duration d)
    {
        timespec t;
        d.split!("seconds", "nsecs")(t.tv_sec, t.tv_nsec);
        auto it = itimerspec(timespec(0,0), t);
        check!timerfd_settime(fd, 0, &it, null);
        check!epoll_ctl(evloop.epfd, EPOLL_CTL_MOD, fd, &ev);
    }

    private void onTriggered(uint evmask)
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
    epoll_event ev;
    Callback cb;
    Worker wrkr;

    this(Worker w)
    {
        wrkr = w;
        ev.events = EPOLLIN | EPOLLOUT | EPOLLET;
        cb = &call;
        ev.data.ptr = cast(void*)&cb;
    }

    void beforeCloseHandle(int h)
    {
        assert(h == fd, "wrong fd for WrapFD.beforeCloseHandle");
        check!epoll_ctl(evloop.epfd, EPOLL_CTL_DEL, h, null);
    }

    void afterOpenHandle(int h)
    {
        assert(fd == -1, "WrapFD is inited");
        check!epoll_ctl(evloop.epfd, EPOLL_CTL_ADD, h, &ev);
        fd = h;
    }

    void wakeOnIO(bool wake, Duration timeout)
    {
        wrkr.currWaker = wake ? this :
                         wrkr.currWaker == this ? null :
                         wrkr.currWaker;
        wrkr.timer.set(timeout);
    }

    void call(uint evmask)
    {
        // shut down connection
        if (evmask & EPOLLRDHUP) { mlog("EPOLLRDHUP"); }
        // close other side of pipe
        if (evmask & EPOLLERR) { mlog("EPOLLERR"); }
        // internal error
        if (evmask & EPOLLHUP) { mlog("EPOLLHUP"); }
        // priority?
        if (evmask & EPOLLPRI) { mlog("EPOLLPRI"); }

        // EPOLLET can only be setted
        // EPOLLONESHOT can only be setted

        // can read
        if (evmask & EPOLLIN)
            if (wrkr.currWaker == this)
                wrkr.exec();

        // can write
        if (evmask & EPOLLOUT)
            if (wrkr.currWaker == this)
                wrkr.exec();
    }
}