import std.stdio;

import socat;
import threadtest;
import basiclooptest;

void main()
{
    writeln("=== start realtest ===");
    auto e = runSocat();
    writefln("socat ports: %s %s", e.port1, e.port2);

    threadTest(e.ports);
    basicLoopTest(e.ports);

    writeln("=== finish realtest ===");
}