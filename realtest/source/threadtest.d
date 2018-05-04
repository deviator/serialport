module threadtest;

import base;

enum BUFFER_SIZE = 1024;

void threadTest(string[2] ports)
{
    auto t = spawn(&thread, ports[1]);
    auto com = new SerialPort(ports[0]);
    scope (exit) com.close();

    auto list = [
        "one",
        "one two",
        "one two three",
        "x".repeat(BUFFER_SIZE-2).join
    ];

    auto msg = "hello serialport";

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
                    send(t, end);
                    work = false;
                }
                else
                {
                    msg = list.front;
                    list.popFront();
                }
                com.write(msg);
            },
            (iTResult tr)
            {
                enforce(tr.code == 0, "error in child thread: " ~ tr.msg);
                writeln("child thread: ", tr.msg);
            }
        );
    }
}

struct End {}
End end() @property { return End.init; }

struct TResult { int code; string msg; }
alias iTResult = immutable TResult;
auto failExit(string msg) { return iTResult(1, msg); }
auto successExit() { return iTResult(0, "success exit"); }

void thread(string port)
{
    try
    {
        auto com = new SerialPort(port);
        scope (exit) com.close();

        bool work = true;
        void[BUFFER_SIZE] buffer = void;

        while (work)
        {
            auto data = com.read(buffer);

            if (data.length)
                send(ownerTid, cast(string)(data.idup));

            receiveTimeout(1.msecs,
                (SerialPort.Config cfg) { com.config = cfg; },
                (End e) { work = false; }
            );
        }
    }
    catch (Throwable e) send(ownerTid, failExit(e.msg));

    send(ownerTid, successExit);
}