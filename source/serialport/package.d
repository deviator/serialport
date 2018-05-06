/++
    Simple non-blocking work with serial port for Posix and Windows.

    See also `example/monitor`
 +/
module serialport;

version (Posix) {} else version (Windows) {}
else static assert(0, "unsupported platform");

public
{
    import serialport.base;
    import serialport.config;
    import serialport.block;
    import serialport.nonblock;
    import serialport.exception;
    import serialport.types;
}

version (unittest): private:

import std.range;
import std.concurrency;
import std.exception;
import std.datetime;
import std.conv;
import std.string;
import std.stdio;
import std.random;
import std.process;
import core.thread;

enum BUFFER_SIZE = 1024;

interface ComPipe
{
    void open();
    string command() const @property;
    string[2] ports() const @property;
}

class SocatPipe : ComPipe
{
    int bufferSize;
    ProcessPipes pipe;
    string[2] _ports;
    string _command;

    this(int bs)
    {
        bufferSize = bs;
        _command = ("socat -d -d -b%d pty,raw,"~
                    "echo=0 pty,raw,echo=0").format(bufferSize);
    }

    static string parsePort(string ln)
    {
        auto ret = ln.split[$-1];
        enforce(ret.startsWith("/dev/"),
        "unexpected last word in output line '%s'".format(ln));
        return ret;
    }

    override void open()
    {
        pipe = pipeShell(_command);
        _ports[0] = parsePort(pipe.stderr.readln.strip);
        _ports[1] = parsePort(pipe.stderr.readln.strip);
    }
    
    override const @property
    {
        string command() { return _command; }
        string[2] ports() { return _ports; }
    }
}

unittest
{
    enum socat_out_ln = "2018/03/08 02:56:58 socat[30331] N PTY is /dev/pts/1";
    assert(SocatPipe.parsePort(socat_out_ln) == "/dev/pts/1");
    assertThrown(SocatPipe.parsePort("some string"));
}

ComPipe getPlatformComPipe(int bufsz)
{
    import std.stdio;
    stderr.writeln("available ports: ", SerialPort.listAvailable);

    version (Posix) return new SocatPipe(bufsz);
    else
    {
        pragma(msg, "platform doesn't support, no real test");
        return null;
    }
}

// real test main
//version (realtest)
unittest
{
    stderr.writeln("=== start real test ===\n");
    scope (exit) stderr.writeln("\n=== finish real test ===");
    auto cp = getPlatformComPipe(BUFFER_SIZE);
    if (cp is null)
    {
        stderr.writeln("platform doesn't support");
        return;
    }
    void reopen()
    {
        stderr.writeln("\n");
        Thread.sleep(150.msecs);
        cp.open();
        stderr.writefln("run command `%s`", cp.command);
        stderr.writefln("pipe ports: %s <=> %s", cp.ports[0], cp.ports[1]);
    }

    reopen();
    utCall!(threadTest!SerialPortNonBlk)("thread test for non-block", cp.ports);

    reopen();
    utCall!(threadTest!SerialPortBlk)("thread test for block", cp.ports);

    reopen();
    utCall!fiberTest("fiber test", cp.ports);
}

auto utCall(alias fnc, Args...)(string fname, Args args)
{
    stderr.writefln("--- run %s ---", fname);
    scope (success) stderr.writefln("--- success %s ---", fname);
    scope (failure) stderr.writefln("!!! failure %s !!!", fname);
    return fnc(args);
}

void threadTest(SPT)(string[2] ports)
{
    assert(SerialPort.listAvailable.length != 0);

    static struct End {}
    static End end() @property { return End.init; }

    static void echoThread(string port)
    {
        auto com = new SPT(port, "2400:8N1");
        scope (exit) com.close();

        com.set(1200);
        assert(com.config.baudRate == 1200);

        com.baudRate = 19200;
        assert(com.config.baudRate == 19200);

        bool work = true;
        void[BUFFER_SIZE] buffer = void;

        while (work)
        {
            auto data = com.read(buffer);

            if (data.length)
                send(ownerTid, cast(string)(data.idup));

            receiveTimeout(100.msecs,
                (SPConfig cfg)
                {
                    com.config = cfg;
                },
                (End e)
                {
                    work = false;
                }
            );
        }

        send(ownerTid, end);
    }

    auto t = spawn(&echoThread, ports[1]);

    auto com = new SPT(ports[0], 19200);

    assert(com.baudRate == 19200);
    assert(com.dataBits == DataBits.data8);
    assert(com.parity == Parity.none);
    assert(com.stopBits == StopBits.one);

    assert(com.config.baudRate == 19200);
    assert(com.config.dataBits == DataBits.data8);
    assert(com.config.parity == Parity.none);
    assert(com.config.stopBits == StopBits.one);

    scope (exit) com.close();

    enum origList = [
        "one",
        "one two",
        "one two three",
        "x".repeat(BUFFER_SIZE-2).join
    ];

    string[] list;

    auto sets = [
        SPConfig.parse("9600:8N1"),
        SPConfig(38400),
        SPConfig(1200),
        SPConfig.parse("19200:8N2"),
    ];

    string msg = sets.front.mode;

    void initList()
    {
        com.config = sets.front;
        send(t, sets.front);
        msg = sets.front.mode;
        sets.popFront;
        list = origList.dup;
    }

    initList();
    com.write(msg);

    bool work = true;
    while (work)
    {
        receive(
            (string rec)
            {
                enforce(rec == msg, "break message: '%s' != '%s'".format(msg, rec));

                if (list.empty)
                {
                    if (sets.empty)
                    {
                        work = false;
                        stderr.writeln("ownerThread send 'end'");
                        send(t, end);
                    }
                    else initList();
                }
                else
                {
                    msg = list.front;
                    list.popFront();
                }

                com.write(msg);
            },
            (End e)
            {
                work = false;
                stderr.writeln("ownerThread receive 'end'");
            },
            (Throwable e)
            {
                work = false;
                throw e;
            }
        );
    }

    /+
    version (Posix) // socat not supported
    {
        assertThrown!SerialPortException(com.set(DataBits.data5));
        assertThrown!SerialPortException(com.set(DataBits.data6));
        assertThrown!SerialPortException(com.set(DataBits.data7));
        assertThrown!SerialPortException(com.set(StopBits.onePointFive));
        // on my system but not in travis
        //assertThrown!SerialPortException(com.set(Parity.even));
    }
    +/
}

void fiberTest(string[2] ports)
{
    static class CF : Fiber
    {
        void[] data;

        SerialPortFR com;

        this(SerialPortFR com, size_t bufsize)
        {
            this.com = com;
            this.data = new void[bufsize];
            super(&run);
        }

        abstract void run();
    }

    static class CFSlave : CF
    {
        void[] result;

        this(SerialPortFR com, size_t bufsize)
        { super(com, bufsize); }

        override void run() { result = com.readLoop(data, 40.msecs, 10.msecs); }
    }

    static class CFMaster : CF
    {
        CFSlave slave;

        this(SerialPortFR com, size_t bufsize)
        {
            super(com, bufsize);
            foreach (ref v; cast(ubyte[])data)
                v = uniform(ubyte(0), ubyte(128));
        }

        override void run() { com.writeLoop(data, 20.msecs); }
    }

    auto slave = new CFSlave(new SerialPortFR(ports[0]), BUFFER_SIZE);
    auto master = new CFMaster(new SerialPortFR(ports[1]), BUFFER_SIZE);

    bool work = true;
    int step;
    while (work)
    {
        if (master.state != Fiber.State.TERM) master.call;
        if (slave.state != Fiber.State.TERM) slave.call;

        step++;
        Thread.sleep(250.nsecs);
        if (slave.result.length == master.data.length)
        {
            import std.algorithm : equal;
            enforce(equal(cast(ubyte[])slave.result, cast(ubyte[])master.data));
            work = false;
            writeln("basic loop steps: ", step);
        }
    }
}