import std.stdio;
import core.thread : Thread, msecs;

import serialport;

int main(string[] args)
{
    if (args.length < 2)
    {
        stderr.writeln("no ports");
        return 1;
    }

    SerialPortNonBlk[] ports;

    stderr.writeln("start virtual bus on: ", args[1..$]);

    foreach (p; args[1..$])
        ports ~= new SerialPortNonBlk(p);

    
    void[16*1024] buffer = void;

    while (true)
    {
        foreach (port; ports)
        {
            const readed = port.read(buffer);
            if (readed.length)
            {
                foreach (other; ports)
                {
                    if (other is port) continue;
                    other.write(readed);
                }
            }
            Thread.sleep(1.msecs);
        }
    }
}