module basiclooptest;

import base;

class CF : Fiber
{
    void[] data;

    SerialPortBL com;

    this(SerialPortBL com, size_t bufsize)
    {
        this.com = com;
        this.data = new void[bufsize];
        super(&run);
    }

    abstract void run();
}

class CFMaster : CF
{
    CFSlave slave;

    this(SerialPortBL com, size_t bufsize)
    {
        super(com, bufsize);
        foreach (ref v; cast(ubyte[])data)
            v = uniform(ubyte(0), ubyte(128));
    }

    override void run() { com.writeLoop(data, 20.msecs); }
}

class CFSlave : CF
{
    void[] result;

    this(SerialPortBL com, size_t bufsize)
    { super(com, bufsize); }

    override void run() { result = com.readLoop(data, 10.msecs, 1.msecs); }
}

void basicLoopTest(string[2] ports, size_t bufsize=1024*64)
{
    auto slave = new CFSlave(new SerialPortBL(ports[0]), bufsize);
    auto master = new CFMaster(new SerialPortBL(ports[1]), bufsize);

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
            enforce(equal(cast(ubyte[])slave.result, cast(ubyte[])master.data));
            work = false;
            writeln("basic loop steps: ", step);
        }
    }
}