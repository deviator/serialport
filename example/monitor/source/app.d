import std.algorithm : max;
import std.stdio;
import std.string;
import std.range;
import core.thread;

import serialport;

int main(string[] args)
{
    string port;
    size_t step = 8;

    void[1024*4] data = void;

    if (args.length < 2)
    {
        stderr.writeln("use: monitor /dev/<PORTNAME>");
        return 1;
    }

    auto msg = "listen " ~ args[1];

    size_t pcount=0;

    void dots(bool reset=false)
    {
        if (pcount > 5) reset = true;
        if (reset)
        {
            auto w = " ".repeat(msg.length+pcount+40).join;
            stdout.write("\r", w, "\r");
            stdout.write(msg);
            pcount = 0;
        }
        else
        {
            pcount++;
            stdout.write(".");
        }
        stdout.flush();
    }

    auto com = new SerialPortNonBlk(args[1], "9600:8N1");

    stdout.writeln("port config: ", com.config);

    dots(true);
    stdout.flush();

    while (true)
    {
        void[] tmp;
        while(tmp.length == 0)
        {
            tmp = com.read(data);
            Thread.sleep(500.msecs);
            dots();
        }

        writeln();
        writeln("-- text\n", cast(string)tmp);
        writeln("-- end\n\n-- data");

        size_t i;
        foreach (c; (cast(ubyte[])tmp).chunks(step))
        {
            auto hex = format("%(0x%02X %)", c);
            auto dec = format("%(% 4d%)", c);
            writefln("%05d..%05d > %s  | %s", i*step, i*step + c.length,
                    hex ~ " ".repeat(max(0,40-hex.length-1)).join, dec);
            i++;
        }
        writefln("-- end\n\nreceive %d bytes\n", tmp.length);

        dots(true);
    }
}
