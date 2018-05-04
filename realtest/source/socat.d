module socat;

version (Posix) {}
else static assert(0, "support only posix platform");

import std.process;
import std.string;
import std.exception;

import core.sys.posix.unistd : getuid;

auto runSocat(uint bufferSize=1024)
{
    struct Result
    {
        ProcessPipes pipe;
        string port1, port2;

        string[2] ports() const @property
        { return [port1, port2]; }

        this(int bufsz)
        {
            enforce(getuid() == 0, "you must be a root");
            auto cmd = ("socat -d -d -b%d pty,raw,"~
                   "echo=0 pty,raw,echo=0").format(bufsz);
            import std.stdio;
            writeln(cmd);
            pipe = pipeShell(cmd);
            
            port1 = parsePort(pipe.stderr.readln.strip);
            port2 = parsePort(pipe.stderr.readln.strip);
        }

        string parsePort(string ln)
        {
            import std.file : exists;
            auto p = parseSocatPort(ln);
            enforce(p.exists, "%s doesn't exists".format(p));
            return p;
        }

        ~this() { kill(pipe.pid); }
    }

    return Result(bufferSize);
}

string parseSocatPort(string ln)
{
    // 
    auto ret = ln.split[$-1];
    enforce(ret.startsWith("/dev/pts/"),
    "unexpected last word in output line '%s'".format(ln));
    return ret;
}

unittest
{
    assert("2018/03/08 02:56:58 socat[30331] N PTY is /dev/pts/1"
                .parseSocatPort == "/dev/pts/1");

    assertThrown("some string".parseSocatPort);
}