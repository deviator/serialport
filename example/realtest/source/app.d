/+ dub.sdl:
    name "realtest"
    dependency "serialport" path=".."
 +/

// need socat program for working
module realtest;

import std.stdio;
import std.concurrency;
import std.exception;
import std.datetime;
import std.conv;

import serialport;
import socat;

void main()
{
    auto e = runSocat();
    writefln("socat ports: %s %s", e.port1, e.port2);

    auto t = spawn(&thread, e.port2);

    auto com = new SerialPort(e.port1);

    auto msg = "hello serialport";

    com.write(msg);

    bool work = true;
    while (work)
    {
        receive(
            (string rec)
            {
                enforce(rec == msg, "break message: '%s' != '%s'".format(msg, rec));
                send(t, End.init);
                work = false;
            },
            (iTResult tr)
            {
                writeln(tr.msg);
            }
        );
    }
    writeln("finish");
}

struct End {};
struct TResult { int code; string msg; }
alias iTResult = immutable TResult;

auto failExit(string msg) { return iTResult(1, msg); }
auto successExit() { return iTResult.init; }

void thread(string port)
{
    try
    {
        auto com = new SerialPort(port);

        bool work = true;
        void[1024] buffer = void;

        while (work)
        {
            auto data = com.read(buffer);

            if (data.length)
                send(ownerTid, cast(string)(data.idup));

            receiveTimeout(1.usecs,
                (SerialPort.Config cfg) { com.config = cfg; },
                (End e) { work = false; }
            );
        }
    }
    catch (Throwable e) send(ownerTid, failExit(e.msg));

    send(ownerTid, successExit);
}