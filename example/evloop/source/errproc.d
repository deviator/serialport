module errproc;

import std.array : appender;
import std.format : formattedWrite;
import std.algorithm : canFind;
import core.sys.linux.errno;

version (CRuntime_Glibc) version = GNU_STRERROR;
version (CRuntime_UClibc) version = GNU_STRERROR;

string errnoStr(int errno) nothrow @trusted
{
    import core.stdc.string : strlen;
    version (GNU_STRERROR)
    {
        import core.stdc.string : strerror_r;
        char[1024] buf = void;
        auto s = strerror_r(errno, buf.ptr, buf.length);
    }
    else version (Posix)
    {
        // XSI-compliant
        import core.stdc.string : strerror_r;
        char[1024] buf = void;
        const(char)* s;
        if (strerror_r(errno, buf.ptr, buf.length) == 0)
            s = buf.ptr;
        else
            return "Unknown error";
    }
    else
    {
        import core.stdc.string : strerror;
        auto s = strerror(errno);
    }
    return s[0 .. s.strlen].idup;
}

private 
auto checkBase(alias fn, string file=__FILE__, size_t line=__LINE__, Args...)
    (scope const int[] allowErr, Args args)
{
    enum fname = __traits(identifier, fn);
    auto ret = fn(args);
    if (ret == -1 && !allowErr.canFind(errno))
    {
        auto buf = appender!string;
        buf.formattedWrite!"'%s("(fname);
        foreach (i, arg; args)
        {
            buf.formattedWrite!"%s"(arg);
            if (i+1 != args.length) buf.put(", ");
        }
        buf.formattedWrite!")' failed: %s %s"(errno, errnoStr(errno));
        throw new Exception(buf.data, file, line);
    }
    return ret;
}

auto check(alias fn, string file=__FILE__, size_t line=__LINE__, Args...)(Args args)
{ return checkBase!(fn, file, line)([], args); }

auto checkNB(alias fn, string file=__FILE__, size_t line=__LINE__, Args...)(Args args)
{ return checkBase!(fn, file, line)([EAGAIN], args); }


import std.format : format;
import std.datetime.stopwatch;

StopWatch sw;

static this() { sw.start(); }

void mlog(const(char[]) msg, string fnc=__FUNCTION__)
{
    import std.stdio : stderr;
    stderr.writefln("[%09d] %s: %s", sw.peek().total!"usecs", fnc, msg);
}

void mlogf(string fnc=__FUNCTION__, Args...)(string fmt, Args args)
{ mlog(format(fmt, args), fnc); }