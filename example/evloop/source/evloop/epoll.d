module evloop.epoll;

version (epoll):

import core.sys.linux.epoll;

import evloop.base;
import evloop.linuxtimer;

class EpollLoop : EvLoop
{
    bool isRun;

    int epfd;
    epoll_event[256] events;

    this()
    {
        epfd = check!epoll_create1(EPOLL_CLOEXEC);
        isRun = true;
    }

    epoll_event getEvent(FDData data)
    {
        epoll_event ev;
        ev.events = (data.read ? EPOLLIN : 0) |
                    (data.write ? EPOLLIN : 0) |
                    (data.repeat == 1 ? EPOLLONESHOT : 0) |
                    (data.repeat == -1 ? EPOLLET : 0);
        ev.data.ptr = cast(void*)data.cb;
        return ev;
    }

    override
    {
        void addFD(int fd, FDData data)
        {
            epoll_event ev = getEvent(data);
            check!epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);
        }

        void modFD(int fd, FDData data)
        {
            epoll_event ev = getEvent(data);
            check!epoll_ctl(epfd, EPOLL_CTL_MOD, fd, &ev);
        }

        void delFD(int fd)
        {
            check!epoll_ctl(epfd, EPOLL_CTL_DEL, fd, null);
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

        void run() { while(step()) {} }

        Waker makeNewWaker(Worker w)
        { return new EpollWaker(w); }

        Timer makeNewTimer(void delegate(Timer) fnc)
        { return new LinuxTimer(fnc); }
    }
}

private EpollLoop epollloop;

static this()
{
    epollloop = new EpollLoop; 
    evl = epollloop;
}

class EpollWaker : Waker
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

    override
    {
        void beforeCloseHandle(int h)
        {
            assert(h == fd, "wrong fd for WrapFD.beforeCloseHandle");
            check!epoll_ctl(epollloop.epfd, EPOLL_CTL_DEL, h, null);
        }

        void afterOpenHandle(int h)
        {
            assert(fd == -1, "WrapFD is inited");
            check!epoll_ctl(epollloop.epfd, EPOLL_CTL_ADD, h, &ev);
            fd = h;
        }

        void wakeOnRead(bool w, Duration t) { wakeOnIO(w, t); }
        void wakeOnWrite(bool w, Duration t) { wakeOnIO(w, t); }
    }

    void wakeOnIO(bool wake, Duration timeout)
    {
        if (wake) wrkr.setWaker(this);
        else wrkr.unsetWaker(this);
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
            if (wrkr.currentWaker is this)
                wrkr.exec();

        // can write
        if (evmask & EPOLLOUT)
            if (wrkr.currentWaker is this)
                wrkr.exec();
    }
}