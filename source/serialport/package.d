/++
    Simple non-blocking work with serial port for Posix and Windows.

    See also `example/monitor`
 +/
module serialport;

version (readAvailable) {} else version (readAllOrThrow) {}
else static assert(0, "one of readAvailable or readAllOrThrow must be selected");

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
    void close();
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

    override void close()
    {
        if (pipe.pid is null) return;
        kill(pipe.pid);
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

class DefinedPorts : ComPipe
{
    string[2] env;
    string[2] _ports;

    this(string[2] envNames = ["SERIALPORT_TEST_PORT1", "SERIALPORT_TEST_PORT2"])
    { env = envNames; }

override:

    void open()
    {
        import std.process : environment;
        import std.range : lockstep;
        import std.algorithm : canFind;

        auto lst = SerialPort.listAvailable;

        foreach (ref e, ref p; lockstep(env[], _ports[]))
        {
            p = environment[e];
            enforce(lst.canFind(p), new Exception("unknown port '%s' in env var '%s'".format(p, e)));
        }
    }

    void close() { }

    string command() const @property
    {
        return "env: %s=%s, %s=%s".format(
            env[0], _ports[0],
            env[1], _ports[1]
        );
    }

    string[2] ports() const @property { return _ports; }
}

unittest
{
    enum socat_out_ln = "2018/03/08 02:56:58 socat[30331] N PTY is /dev/pts/1";
    assert(SocatPipe.parsePort(socat_out_ln) == "/dev/pts/1");
    assertThrown(SocatPipe.parsePort("some string"));
}

ComPipe getPlatformComPipe(int bufsz)
{
    stderr.writeln("available ports count: ", SerialPort.listAvailable.length);

    try
    {
        auto ret = new DefinedPorts;
        ret.open();
        return ret;
    }
    catch (Exception e)
    {
        stderr.writeln();
        stderr.writeln("error while open predefined ports: ", e.msg);

        version (linux) return new SocatPipe(bufsz);
        else version (OSX) return new SocatPipe(bufsz);
        else
        {
            pragma(msg, "platform doesn't support, no real test");
            return null;
        }
    }
}

// real test main
//version (realtest)
unittest
{
    stderr.writeln("=== start real test ===\n");
    scope (success) stderr.writeln("\n=== finish real test ===");
    scope (failure) stderr.writeln("\n!!!  fail real test  !!!");
    auto cp = getPlatformComPipe(BUFFER_SIZE);
    if (cp is null)
    {
        stderr.writeln("platform doesn't support");
        return;
    }

    stderr.writefln("port source `%s`", cp.command);
    void reopen()
    {
        cp.close();
        Thread.sleep(50.msecs);
        cp.open();
        stderr.writefln("pipe ports: %s <=> %s", cp.ports[0], cp.ports[1]);
    }

    version (readAvailable)
    {
        // tests designed for readAvailable configuration

        reopen();
        utCall!(threadTest!SerialPortFR)("thread test for non-block", cp.ports);
        reopen();
        utCall!(threadTest!SerialPortBlk)("thread test for block", cp.ports);
    }

    reopen();
    utCall!fiberTest("fiber test", cp.ports);

    reopen();
    utCall!fiberTest2("fiber test 2", cp.ports);

    reopen();
    utCall!readTimeoutTest("read timeout test", cp.ports);

    reopen();
    utCall!(readTimeoutTestConfig!SerialPortFR)("read timeout test for fiber ready and current config", cp.ports);

    reopen();
    utCall!(readTimeoutTestConfig!SerialPortBlk)("read timeout test for block reading and current config", cp.ports);
}

auto utCall(alias fnc, Args...)(string name, Args args)
{
    stderr.writefln("--- run %s ---", name);
    scope (success) stderr.writefln("--- success %s ---", name);
    scope (failure) stderr.writefln("!!! failure %s !!!", name);
    return fnc(args);
}

void threadTest(SPT)(string[2] ports)
{
    assert(SerialPort.listAvailable.length != 0);

    static struct ExcStruct { string msg, type; }

    static void echoThread(string port)
    {
        void[BUFFER_SIZE] buffer = void;
        auto com = new SPT(port, "2400:8N1");
        scope (exit) com.close();
        com.flush();

        com.set(1200);
        assert(com.config.baudRate == 1200);

        com.baudRate = 38400;
        assert(com.config.baudRate == 38400);

        bool work = true;
        com.readTimeout = 1000.msecs;

        bool needRead = false;

        while (work)
        {
            try
            {
                if (needRead)
                {
                    try
                    {
                        version (readAvailable) Thread.sleep(500.msecs);
                        auto data = com.read(buffer);

                        if (data.length)
                        {
                            stderr.writeln("child readed: ", cast(string)(data.idup));
                            send(ownerTid, cast(string)(data.idup));
                        }
                    }
                    catch (TimeoutException e)
                        stderr.writeln("child timeout read");
                }

                receiveTimeout(500.msecs,
                    (SPConfig cfg)
                    {
                        com.config = cfg;
                        stderr.writeln("child get cfg: ", cfg.mode);
                    },
                    (bool nr)
                    {
                        if (nr) needRead = true;
                        else
                        {
                            work = false;
                            needRead = false;
                        }
                        stderr.writeln("get needRead ", nr);
                    },
                    (OwnerTerminated e) { work = false; }
                );
            }
            catch (Throwable e)
            {
                work = false;
                stderr.writeln("exception in child: ", e);
                send(ownerTid, ExcStruct(e.msg, e.classinfo.stringof));
            }
        }
    }

    auto t = spawnLinked(&echoThread, ports[1]);

    auto com = new SPT(ports[0], 19200);
    com.flush();

    assert(com.baudRate == 19200);
    assert(com.dataBits == DataBits.data8);
    assert(com.parity == Parity.none);
    assert(com.stopBits == StopBits.one);

    assert(com.config.baudRate == 19200);
    assert(com.config.dataBits == DataBits.data8);
    assert(com.config.parity == Parity.none);
    assert(com.config.stopBits == StopBits.one);

    scope (exit) com.close();

    enum NN = BUFFER_SIZE / 2;

    enum origList = [
        "one",
        "one two",
        "one two three",
        "x".repeat(NN).join
    ];

    string[] list;

    auto sets = [
        SPConfig(38400),
        SPConfig(2400),
        SPConfig.parse("19200:8N2"),
    ];

    auto cfg = SPConfig(38400);
    com.config = cfg;
    send(t, cfg);

    Thread.sleep(1000.msecs);

    string msg = sets.front.mode;
    com.write(msg);

    bool work = true;
    send(t, true);
    while (work)
    {
        receive(
            (string rec)
            {
                enforce(rec == msg, "break message: '%s' != '%s'".format(msg, rec));

                if (list.empty)
                {
                    work = false;
                    stderr.writeln("owner send data finish");
                    send(t, false);
                }
                else
                {
                    msg = list.front;
                    list.popFront();
                }

                com.write(msg);
                stderr.writeln("owner write msg to com: ", msg);
            },
            (ExcStruct e) { throw new Exception("%s:%s".format(e.type, e.msg)); },
            (LinkTerminated e)
            {
                stderr.writeln("link terminated");
                work = false;
                stderr.writeln(e.tid, " ", t);
                //assert(e.tid == t);
            }
        );
    }
    send(t, false);
}

class CF : Fiber
{
    void[] data;

    SerialPortFR com;

    this(SerialPortFR com, size_t bufsize)
    {
        this.com = com;
        this.com.flush();
        this.data = new void[bufsize];
        super(&run);
    }

    abstract void run();
}

class CFSlave : CF
{
    void[] result;

    Duration readTimeout = 40.msecs;
    Duration readGapTimeout = 10.msecs;

    this(SerialPortFR com, size_t bufsize)
    { super(com, bufsize); }

    override void run()
    {
        stderr.writeln("start read loop");
        result = com.readAll(data, readTimeout, readGapTimeout);
        stderr.writeln("finish read loop");
    }
}

class CFMaster : CF
{
    CFSlave slave;

    Duration writeTimeout = 20.msecs;

    this(SerialPortFR com, size_t bufsize)
    {
        super(com, bufsize);
        foreach (ref v; cast(ubyte[])data)
            v = uniform(ubyte(0), ubyte(128));
    }

    override void run()
    {
        stderr.writeln("start write loop");
        com.writeTimeout = writeTimeout;
        com.write(data);
        stderr.writeln("finish write loop");
    }
}

void fiberTest(string[2] ports)
{
    auto slave = new CFSlave(new SerialPortFR(ports[0]), BUFFER_SIZE);
    auto master = new CFMaster(new SerialPortFR(ports[1]), BUFFER_SIZE);

    bool work = true;
    int step;
    while (work)
    {
        alias TERM = Fiber.State.TERM;
        if (master.state != TERM) master.call;
        if (slave.state != TERM) slave.call;

        step++;
        Thread.sleep(30.msecs);
        if (master.state == TERM && slave.state == TERM)
        {
            if (slave.result.length == master.data.length)
            {
                import std.algorithm : equal;
                enforce(equal(cast(ubyte[])slave.result, cast(ubyte[])master.data));
                work = false;
                writeln("basic loop steps: ", step);
            }
            else throw new Exception(text(slave.result, " != ", master.data));
        }
    }
}

void fiberTest2(string[2] ports)
{
    string mode = "38400:8N1";

    auto scom = new SerialPortFR(ports[0], 9600, "8N1");
    auto mcom = new SerialPortFR(ports[1], "19200:8N1");

    version (Posix)
        assertThrown!UnsupportedException(scom.baudRate = 9200);

    scom.reopen(ports[0], SPConfig.parse(mode));
    mcom.reopen(ports[1], SPConfig.parse(mode));
    scom.flush();
    mcom.flush();

    scom.readTimeout = 1000.msecs;
    mcom.writeTimeout = 100.msecs;

    enum BK = 4;

    auto slave  = new CFSlave(scom,  BUFFER_SIZE * BK);
    auto master = new CFMaster(mcom, BUFFER_SIZE * BK);

    void run()
    {
        bool work = true;
        int step;
        while (work)
        {
            if (master.state != Fiber.State.TERM) master.call;
            Thread.sleep(20.msecs);
            if (slave.state != Fiber.State.TERM) slave.call;

            step++;
            if (slave.result.length == master.data.length)
            {
                import std.algorithm : equal;
                enforce(equal(cast(ubyte[])slave.result, cast(ubyte[])master.data));
                work = false;
                writeln("basic loop steps: ", step);
            }
        }
    }

    run();
}

void readTimeoutTest(string[2] ports)
{
    string mode = "19200:8N1";

    auto comA = new SerialPortFR(ports[0], 19200);
    void[1024] buffer = void;
    try while (true) comA.read(buffer); catch (TimeoutException) {} // flush
    assertThrown!TimeoutException(comA.readAll(buffer[], 1.msecs, 1.msecs));

    auto comB = new SerialPortBlk(ports[1], 19200, "8N1");
    try while (true) comB.read(buffer); catch (TimeoutException) {} // flush
    comB.readTimeout = 1.msecs;
    assertThrown!TimeoutException(comB.read(buffer[]));
}

void readTimeoutTestConfig(SP : SerialPortTm)(string[2] ports)
{
    enum mode = "38400:8N1";

    enum FULL = 100;
    enum SEND = "helloworld";

    static void thfunc(string port)
    {
        auto com = new SP(port, mode);
        com.flush();
        scope (exit) com.close();
        com.write(SEND);
    }

    auto com = new SP(ports[0], mode);
    scope (exit) com.close();
    auto rt = 100.msecs;
    com.readTimeout = rt;
    com.flush();
    assert(com.readTimeout == rt);

    void[FULL] buffer = void;
    void[] data;

    auto t = spawnLinked(&thfunc, ports[1]);

    Thread.sleep(rt);

    version (readAvailable)
    {
        assertNotThrown(data = com.read(buffer));
        assert(cast(string)data == SEND);
    }
    version (readAllOrThrow)
        assertThrown!TimeoutException(data = com.read(buffer));

    receive((LinkTerminated e) { });
}