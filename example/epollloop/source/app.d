import std.stdio;
import std.datetime;
import std.experimental.logger;

import core.thread : Fiber;

import core.sys.posix.unistd;
import core.sys.linux.timerfd;
import core.sys.linux.sys.signalfd;
import core.sys.linux.epoll;
import core.sys.linux.errno;

import serialport;
import serialport.types : SPHandle;

import errproc;

std.datetime.stopwatch.StopWatch sw;

static this() { sw.start(); }

void mlog(const(char[]) msg, string fnc=__FUNCTION__)
{
    stderr.writefln("[%09d] %s: %s", sw.peek().total!"usecs", fnc, msg);
}
void mlogf(string fnc=__FUNCTION__, Args...)(string fmt, Args args)
{
    import std : format;
    mlog(format(fmt, args), fnc);
}

// if Callback is interface -- not worked
alias Callback = void delegate(uint);

class Timer
{
    int epfd;
    int fd;
    epoll_event ev;
    Callback cb;

    string name;

    void delegate() dlg;

    this(int epfd, void delegate() dlg)
    {
        this.epfd = epfd;
        this.dlg = dlg;
        fd = check!timerfd_create(CLOCK_MONOTONIC, 0);

        cb = &call;
        ev.data.ptr = cast(void*)&cb;
        ev.events = EPOLLIN | EPOLLONESHOT;

        const it = itimerspec(timespec(0,0), timespec(0,0));
        check!timerfd_settime(fd, 0, &it, null);

        check!epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);
    }

    void set(Duration d)
    {
        timespec t;
        d.split!("seconds", "nsecs")(t.tv_sec, t.tv_nsec);
        auto it = itimerspec(timespec(0,0), t);
        check!timerfd_settime(fd, 0, &it, null);
        check!epoll_ctl(epfd, EPOLL_CTL_MOD, fd, &ev);
    }

    void call(uint evmask)
    {
        ulong v;
        check!read(fd, &v, v.sizeof);
        mlogf("%s: call dlg", name);
        dlg();
    }

    void close() { .close(fd); }
}

class Worker
{
    int epfd;

    Timer timer;

    Fiber fiber;
    Duration period;

    string name;

    this(string name, SPELH spelh, void delegate(Worker w) dlg)
    {
        this.name = name;
        epfd = spelh.epfd;

        timer = new Timer(epfd, &exec);

        timer.name = name;

        fiber = new Fiber(
            {
                mlog(name~": start fiber func");
                dlg(this);
                if (period > Duration.zero)
                    setUpTimer(period);
            }
        );
        spelh.wrkr = this;
        spelh.name = name;
    }

    void setUpTimer(Duration d) { timer.set(d); }
    void wait() { Fiber.yield(); }

    void exec()
    {
        mlog(name);
        if (fiber.state == fiber.State.TERM)
            fiber.reset();
        fiber.call();
    }

    void finish() { timer.close(); }
}

class SPELH : SerialPortEL.EvLoopHook
{
    int epfd;
    epoll_event ev;
    Worker wrkr;
    Callback cb;

    string name;

    bool callFiberOnRead;
    bool callFiberOnWrite;

    this(int epfd)
    {
        this.epfd = epfd;
        ev.events = EPOLLIN | EPOLLOUT | EPOLLET;
        cb = &call;
        ev.data.ptr = cast(void*)&cb;
    }

    override // SerialPortEL.EvLoopHook
    {
        void beforeCloseHandle(SPHandle h)
        {
            check!epoll_ctl(epfd, EPOLL_CTL_DEL, h, null);
        }

        void afterOpenHandle(SPHandle h)
        {
            mlog("1-1-1-1-1-1-1--1-1-1--1-1-1-1--1-1-1-1--1-1");
            check!epoll_ctl(epfd, EPOLL_CTL_ADD, h, &ev);
        }

        void wakeOnRead(bool wake, Duration timeout)
        {
            callFiberOnRead = wake;
            wrkr.setUpTimer(wake ? timeout : Duration.zero);
            mlogf("%s: set wake on read %s (%s)", name, wake, timeout);
        }

        void wakeOnWrite(bool wake, Duration timeout)
        {
            callFiberOnWrite = wake;
            wrkr.setUpTimer(wake ? timeout : Duration.zero);
            mlogf("%s: set wake on write %s (%s)", name, wake, timeout);
        }

        void wait()
        {
            mlog(name);
            Fiber.yield();
            mlogf("%s: return from wait", name);
        }
    }

    void call(uint evmask)
    {
        mlog("*******************************");
        // shut down connection
        if (evmask & EPOLLRDHUP) { info("EPOLLRDHUP"); }
        // close other side of pipe
        if (evmask & EPOLLERR) { info("EPOLLERR"); }
        // internal error
        if (evmask & EPOLLHUP) { info("EPOLLHUP"); }
        // priority?
        if (evmask & EPOLLPRI) { info("EPOLLPRI"); }
        // can read
        if (evmask & EPOLLIN)
        {
            mlog("EPOLLIN");
            if (callFiberOnRead) if (wrkr !is null)
            {
                mlog("----------------EPOLLIN---------------");
                wrkr.exec();
            }
        }
        // can write
        if (evmask & EPOLLOUT)
        {
            mlog("EPOLLOUT");
            if (callFiberOnWrite) if (wrkr !is null) wrkr.exec();
        }
        // can be setted in wait?
        if (evmask & EPOLLET) { info("EPOLLET"); }
        // can be setted in wait?
        if (evmask & EPOLLONESHOT) { info("EPOLLONESHOT"); }

        // undefined in d bind, can be setted in wait?
        //if (evmask & EPOLLWAKEUP) { info("EPOLLWAKEUP"); }

        // can be setted in wait?
        if (evmask & EPOLLEXCLUSIVE) { info("EPOLLEXCLUSIVE"); }
    }
}

class MasterDevice
{
    SerialPort port;
    import std : Appender;

    Appender!(char[]) buffer;

    this(SerialPort sp)
    {
        port = sp;
        port.readTimeout = 500.msecs;
    }

    void step(void delegate(Duration) sleepfnc)
    {
        import std : Clock, formattedWrite;

        buffer.clear();
        buffer.formattedWrite!"hello from master %d"(Clock.currStdTime);
        mlog("============================ master: start write");
        size_t n=1;
        import std : uniform;
        while (n < buffer.data.length)
        {
            port.write(buffer.data[0..min($,n)]);
            n += uniform(1,5);
            sleepfnc(uniform(100,1000).msecs);
        }
        mlog("master: end write ===============================");

        if (sleepfnc !is null) sleepfnc(2.seconds);

        void[256] abuffer = void;
        mlog("master: start read");
        try
        {
            auto answer = cast(char[])port.read(abuffer, SerialPort.CanRead.anyNonZero);
            mlog("master: end read: " ~ answer);
        }
        catch (TimeoutException e) mlog("master: read timeout");
    }
}

class SlaveDevice
{
    SerialPort port;
    this(SerialPort sp)
    {
        port = sp;
        port.readTimeout = 5000.msecs;
    }

    void step()
    {
        void[512] rbuffer = void;

        while (true)
        {
            import std : startsWith;

            mlog("try read");
            auto request = cast(char[])port.read(rbuffer, SerialPort.CanRead.zero);

            if (request.length)
            {
                mlog("read success");
                if (request.startsWith("hello from master"))
                {
                    mlog("get hello from master");
                    port.write("hi!");
                }
            }
            else mlog("no data step");
        }
    }
}

void main(string[] args)
{
    int epfd = check!epoll_create1(0);
    scope (exit) close(epfd);

    infof("epfd: %d", epfd);

    auto spelhM = new SPELH(epfd);
    auto sp1 = new SerialPortEL(spelhM, args[2]);
    auto m = new MasterDevice(sp1);
    auto mw = new Worker("master", spelhM, (w) { m.step((d){ w.setUpTimer(d); w.wait(); }); });
    scope(exit) mw.finish();
    info("master device inited");
    mw.period = 7.seconds;

    auto spelhS = new SPELH(epfd);
    auto sp2 = new SerialPortEL(spelhS, args[1]);
    auto s = new SlaveDevice(sp2);
    auto sw = new Worker("slave", spelhS, (w) { s.step(); });
    scope(exit) sw.finish();
    info("slave device inited");

    mw.exec();
    sw.exec();

    //auto t = new Timer(epfd, null);
    //size_t n;
    //t.dlg = { if (n++<10) t.set(1.seconds); };

    //t.set(1.seconds);

    info("run loop");
    epoll_event[1024] events;
    while (true)
    {
        int nfds = check!epoll_wait(epfd, events.ptr, cast(int)events.length, -1);
        foreach (i; 0 .. nfds)
        {
            auto e = events[i];
            infof("proc #%d %d %s", i, e.events, e.data.ptr);
            auto cb = cast(Callback*)(e.data.ptr);
            if (cb is null)
            {
                warningf("have null ptr in event data");
                continue;
            }
            (*cb)(e.events);
        }
    }
}

//void runLoop(int epfd)
//{
//}

/+
import core.sys.linux.termios;

void main(string[] args)
{
    int epfd = epoll_create1(0);

    void[128] buf = void;

    //int sp = check!open(args[1].toStringz, O_RDWR | O_NOCTTY | O_NONBLOCK);

    //termios opt;
    //check!tcgetattr(sp, &opt);
    //opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
    //                    INLCR | IGNCR | ICRNL | IXON);
    //opt.c_oflag &= ~OPOST;
    //opt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    //opt.c_cflag &= ~(CSIZE | PARENB | CSTOPB);

    //opt.c_cc[VMIN] = 0;
    //opt.c_cc[VTIME] = 0;
    //opt.c_cflag &= ~CRTSCTS;

    //cfsetispeed(&opt, B0);
    //cfsetospeed(&opt, B19200);

    //opt.c_cflag |= CS8;

    //check!tcsetattr(sp, TCSANOW, &opt);

    auto sp = new SerialPortNonBlk(args[1]);

    epoll_event ev;
    ev.events = EPOLLIN | EPOLLET;
    check!epoll_ctl(epfd, EPOLL_CTL_ADD, sp.handle, &ev);

    epoll_event[1024] events;
    while (true)
    {
        int nfds = check!epoll_wait(epfd, events.ptr, cast(int)events.length, -1);
        infof("nfds: %d", nfds);
        foreach (i; 0 .. nfds)
        {
            auto e = events[i];
            // shut down connection
            if (e.events & EPOLLRDHUP) { info("EPOLLRDHUP"); }
            // close other side of pipe
            if (e.events & EPOLLERR) { info("EPOLLERR"); }
            // internal error
            if (e.events & EPOLLHUP) { info("EPOLLHUP"); }
            // priority?
            if (e.events & EPOLLPRI) { info("EPOLLPRI"); }
            // can read
            if (e.events & EPOLLIN)
            {
                stderr.writeln("readed: ", cast(char[])sp.read(buf));
                //auto sres = sp.read(sp, buf.ptr, buf.length);

                //if (sres < 0)
                //{
                //    if (errno == EAGAIN) sres = 0;
                //    else throw new Exception("read error");
                //}

                //stderr.writeln("readed: ", cast(char[])buf[0..sres]);
            }
            // can write
            if (e.events & EPOLLOUT) { info("EPOLLOUT"); }
            // can be setted in wait?
            if (e.events & EPOLLET) { info("EPOLLET"); }
            // can be setted in wait?
            if (e.events & EPOLLONESHOT) { info("EPOLLONESHOT"); }

            // undefined in d bind, can be setted in wait?
            //if (e.events & EPOLLWAKEUP) { info("EPOLLWAKEUP"); }

            // can be setted in wait?
            if (e.events & EPOLLEXCLUSIVE) { info("EPOLLEXCLUSIVE"); }
        }
    }
}
+/