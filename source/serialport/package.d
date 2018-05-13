/++
    Crossplatform work with serialport.

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
    import serialport.fiberready;
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

unittest
{
    enum socat_out_ln = "2018/03/08 02:56:58 socat[30331] N PTY is /dev/pts/1";
    assert(SocatPipe.parsePort(socat_out_ln) == "/dev/pts/1");
    assertThrown(SocatPipe.parsePort("some string"));
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

        version (Posix) return new SocatPipe(bufsz);
        else return null;
    }
}

// real test main
//version (realtest)
unittest
{
    stderr.writeln("=== start real test ===\n");
    scope (success) stderr.writeln("=== finish real test ===");
    scope (failure) stderr.writeln("!!!  fail real test  !!!");
    auto cp = getPlatformComPipe(BUFFER_SIZE);
    if (cp is null)
    {
        stderr.writeln("platform doesn't support real test");
        return;
    }

    stderr.writefln("port source `%s`\n", cp.command);

    void reopen()
    {
        cp.close();
        Thread.sleep(30.msecs);
        cp.open();
        stderr.writefln("pipe ports: %s <=> %s", cp.ports[0], cp.ports[1]);
    }

    reopen();

    utCall!(threadTest!SerialPortFR)("thread test for fiber ready", cp.ports);
    utCall!(threadTest!SerialPortBlk)("thread test for block", cp.ports);
    utCall!testNonBlock("test non block", cp.ports);
    utCall!fiberTest("fiber test", cp.ports);
    utCall!fiberTest2("fiber test 2", cp.ports);
    utCall!readTimeoutTest("read timeout test", cp.ports);
    utCall!(readTimeoutTestConfig!SerialPortFR)( "read timeout test for FR  cr=zero", cp.ports, SerialPort.CanRead.zero);
    utCall!(readTimeoutTestConfig!SerialPortBlk)("read timeout test for Blk cr=zero", cp.ports, SerialPort.CanRead.zero);
    utCall!(readTimeoutTestConfig!SerialPortFR)( "read timeout test for FR  cr=anyNonZero", cp.ports, SerialPort.CanRead.anyNonZero);
    utCall!(readTimeoutTestConfig!SerialPortBlk)("read timeout test for Blk cr=anyNonZero", cp.ports, SerialPort.CanRead.anyNonZero);
    utCall!(readTimeoutTestConfig!SerialPortFR)( "read timeout test for FR  cr=allOrNothing", cp.ports, SerialPort.CanRead.allOrNothing);
    utCall!(readTimeoutTestConfig!SerialPortBlk)("read timeout test for Blk cr=allOrNothing", cp.ports, SerialPort.CanRead.allOrNothing);
}

void testPrint(Args...)(Args args) { stderr.write("    "); stderr.writeln(args); }
void testPrintf(Args...)(Args args) { stderr.write("    "); stderr.writefln(args); }

auto utCall(alias fnc, Args...)(string name, Args args)
{
    stderr.writefln(">>> run %s", name);
    scope (success) stderr.writefln("<<< success %s\n", name);
    scope (failure) stderr.writefln("!!! failure %s\n", name);
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
                    Thread.sleep(500.msecs);
                    auto data = com.read(buffer, com.CanRead.zero);

                    if (data.length)
                    {
                        testPrint("child readed: ", cast(string)(data.idup));
                        send(ownerTid, cast(string)(data.idup));
                    }
                }

                receiveTimeout(500.msecs,
                    (SPConfig cfg)
                    {
                        com.config = cfg;
                        testPrint("child get cfg: ", cfg.mode);
                    },
                    (bool nr)
                    {
                        if (nr) needRead = true;
                        else
                        {
                            work = false;
                            needRead = false;
                        }
                        testPrint("get needRead ", nr);
                    },
                    (OwnerTerminated e) { work = false; }
                );
            }
            catch (Throwable e)
            {
                work = false;
                testPrint("exception in child: ", e);
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
                    testPrint("owner send data finish");
                    send(t, false);
                }
                else
                {
                    msg = list.front;
                    list.popFront();
                }

                com.write(msg);
                testPrint("owner write msg to com: ", msg);
            },
            (ExcStruct e) { throw new Exception("%s:%s".format(e.type, e.msg)); },
            (LinkTerminated e)
            {
                work = false;
                testPrintf("link terminated for %s, child tid %s", e.tid, t);
                //assert(e.tid == t);
            }
        );
    }
}

void testNonBlock(string[2] ports)
{
    import std.datetime.stopwatch;
    enum mode = "38400:8N1";

    const data = "1234567890987654321qazxswedcvfrtgbnhyujm,ki";

    static void thfunc(string port)
    {
        auto com = new SerialPortNonBlk(port, mode);
        scope (exit) com.close();

        void[1024] buffer = void;
        size_t readed;

        auto sw = StopWatch(AutoStart.yes);

        // flush
        while (sw.peek < 10.msecs)
        {
            com.read(buffer);
            Thread.sleep(1.msecs);
        }

        while (sw.peek < 1.seconds)
            readed += com.read(buffer[readed..$]).length;

        send(ownerTid, buffer[0..readed].idup);

        Thread.sleep(200.msecs);
    }

    auto com = new SerialPortNonBlk(ports[0], 38400, "8N1");
    scope (exit) com.close();

    spawnLinked(&thfunc, ports[1]);

    Thread.sleep(100.msecs);

    size_t written;
    while (written < data.length)
        written += com.write(data[written..$]);

    receive((immutable(void)[] readed)
    {
        testPrint("readed: ", cast(string)readed);
        testPrint("  data: ", data);
        assert(cast(string)readed == data);
    });

    receive((LinkTerminated e) { });
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
        testPrint("start read loop");
        result = com.readContinues(data, readTimeout, readGapTimeout);
        testPrint("finish read loop");
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
        testPrint("start write loop");
        com.writeTimeout = writeTimeout;
        com.write(data);
        testPrint("finish write loop");
    }
}

void fiberTest(string[2] ports)
{
    auto slave = new CFSlave(new SerialPortFR(ports[0]), BUFFER_SIZE);
    scope (exit) slave.com.close();
    auto master = new CFMaster(new SerialPortFR(ports[1]), BUFFER_SIZE);
    scope (exit) master.com.close();

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
                testPrint("basic loop steps: ", step);
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
    scope (exit) scom.close();
    scope (exit) mcom.close();

    version (Posix)
        assertThrown!UnsupportedException(scom.baudRate = 9200);

    scom.reopen(ports[0], SPConfig.parse(mode));
    mcom.reopen(ports[1], SPConfig.parse(mode));
    scom.flush();
    mcom.flush();

    scom.readTimeout = 1000.msecs;
    mcom.writeTimeout = 100.msecs;

    version (OSX)
        enum BS = BUFFER_SIZE / 2;
    else
        enum BS = BUFFER_SIZE * 4;

    auto slave  = new CFSlave(scom,  BS);
    auto master = new CFMaster(mcom, BS);

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
                testPrint("basic loop steps: ", step);
            }
        }
    }

    run();
}

void readTimeoutTest(string[2] ports)
{
    string mode = "19200:8N1";
    void[1024] buffer = void;

    auto comA = new SerialPortFR(ports[0], 19200);
    scope (exit) comA.close();
    comA.flush();
    assertThrown!TimeoutException(comA.readContinues(buffer[], 1.msecs, 1.msecs));
    assertThrown!TimeoutException(comA.read(buffer[]));
    assertThrown!TimeoutException(comA.read(buffer[], comA.CanRead.anyNonZero));

    auto comB = new SerialPortBlk(ports[1], 19200, "8N1");
    scope (exit) comB.close();
    comB.flush();
    comB.readTimeout = 1.msecs;
    assertThrown!TimeoutException(comB.read(buffer[]));
    assertThrown!TimeoutException(comB.read(buffer[], comB.CanRead.anyNonZero));
}

void readTimeoutTestConfig(SP : SerialPort)(string[2] ports, SerialPort.CanRead cr)
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
        Thread.sleep(1.seconds);
    }

    auto com = new SP(ports[0], mode);
    scope (exit) com.close();
    auto rt = 300.msecs;
    com.readTimeout = rt;
    com.flush();
    assert(com.readTimeout == rt);

    void[FULL] buffer = void;
    void[] data;

    auto t = spawnLinked(&thfunc, ports[1]);

    Thread.sleep(rt);

    if (cr == SerialPort.CanRead.anyNonZero)
    {
        assertNotThrown(data = com.read(buffer, cr));
        assert(cast(string)data == SEND);
        assertThrown!TimeoutException(data = com.read(buffer, cr));
    }
    else if (cr == SerialPort.CanRead.allOrNothing)
        assertThrown!TimeoutException(data = com.read(buffer));
    else if (cr == SerialPort.CanRead.zero)
    {
        assertNotThrown(data = com.read(buffer, cr));
        assertNotThrown(data = com.read(buffer, cr));
        assertNotThrown(data = com.read(buffer, cr));
    }
    else assert(0, "not tested variant of CanRead");

    receive((LinkTerminated e) { });
}