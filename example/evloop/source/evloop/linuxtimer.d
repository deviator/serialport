module evloop.linuxtimer;

version (linux):

import core.sys.linux.timerfd;

import evloop.base;

class LinuxTimer : Timer
{
    int fd;
    FDData data;

    void delegate(uint) cb;

    void delegate(Timer) func;

    this(void delegate(Timer) fnc)
    {
        func = fnc;

        fd = check!timerfd_create(CLOCK_MONOTONIC, 0);

        const it = itimerspec(timespec(0,0), timespec(0,0));
        check!timerfd_settime(fd, 0, &it, null);

        cb = &onTriggered;
        data = FDData(true, false, &cb, 1);

        evl.addFD(fd, data);
    }

    override void set(Duration d)
    {
        timespec t;
        d.split!("seconds", "nsecs")(t.tv_sec, t.tv_nsec);
        const it = itimerspec(timespec(0,0), t);
        check!timerfd_settime(fd, 0, &it, null);
        evl.modFD(fd, data);
    }

    private void onTriggered(uint evmask)
    {
        ulong v;
        check!read(fd, &v, v.sizeof);
        assert(func !is null, "timer null func");
        func(this);
    }

    override void close()
    {
        evl.delFD(fd);
        .close(fd);
    }
}