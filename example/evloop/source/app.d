module app;

import std.stdio;
import std : format;
import std.datetime;

import core.thread : Fiber;

import serialport;

import errproc;

version (epoll) import epoll_loop;
version (select) import select_loop;

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

    void step()
    {
        import std : Clock, formattedWrite;

        buffer.clear();
        buffer.formattedWrite!"hello from master %d"(Clock.currStdTime);
        mlog(">>>> start write");
        size_t n=1;
        import std : uniform;
        while (n < buffer.data.length)
        {
            port.write(buffer.data[0..min($,n)]);
            n += uniform(1,5);
            WFiber.sleep(uniform(100,1000).msecs);
        }
        mlog("end write ]]]]");

        WFiber.sleep(uniform(200, 2000).msecs);

        void[256] abuffer = void;
        mlog("start read <<<<");
        try
        {
            auto answer = cast(char[])port.read(abuffer, SerialPort.CanRead.anyNonZero);
            mlog("[[[[ end read: " ~ answer);
        }
        catch (TimeoutException e) mlog("**** read timeout");
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

            mlog("try read <<<<");
            auto request = cast(char[])port.read(rbuffer, SerialPort.CanRead.zero);

            if (request.length)
            {
                mlog("[[[[ read success");
                if (request.startsWith("hello from master"))
                {
                    mlog("get hello from master");
                    port.write("hi!");
                }
            }
            else mlog("**** no data step");
        }
    }
}

class SPELH : SerialPortEL.EvLoopHook
{
    WrapFD wfd;
    this(WrapFD wfd) { this.wfd = wfd; }
override:
    void beforeCloseHandle(SPHandle h) { wfd.beforeCloseHandle(h); }
    void afterOpenHandle(SPHandle h) { wfd.afterOpenHandle(h); }
    void wakeOnRead(bool w, Duration d) { wfd.wakeOnIO(w, d); }
    void wakeOnWrite(bool w, Duration d) { wfd.wakeOnIO(w, d); }
    void wait() { Fiber.yield(); }
}

void main(string[] args)
{
    auto mw = new Worker;
    scope(exit) mw.finish();
    auto sp1 = new SerialPortEL(new SPELH(mw.makeNewWrapFD()), args[1]);
    auto m = new MasterDevice(sp1);
    mw.setExecFunc({ m.step(); });
    mw.period = 7.seconds;
    mw.exec();

    auto sw = new Worker;
    scope(exit) sw.finish();
    auto sp2 = new SerialPortEL(new SPELH(sw.makeNewWrapFD()), args[2]);
    auto s = new SlaveDevice(sp2);
    sw.run({ s.step(); });

    size_t n = 8;
    new Timer((tm){
        n /= 2;
        if (n) tm.set(n.seconds);
        mlog("timer");
    }).set(n.seconds);

    mlog("run loop");

    defaultLoop.run();
}