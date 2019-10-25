///
module serialport.exception;

import std.conv : text;

import serialport.types;

///
class ParseModeException : Exception
{
    this(string msg, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow @nogc
    { super(msg, file, line); } 
}

/// General
class SerialPortException : Exception
{
    string port;
    private this() @safe pure nothrow @nogc { super(""); }
}

/// Unsupported config
class UnsupportedException : SerialPortException
{ private this() @safe pure nothrow @nogc { super(); } }

///
class PortClosedException : SerialPortException
{ private this() @safe pure nothrow @nogc { super(); } }

///
class TimeoutException : SerialPortException
{ private this() @safe pure nothrow @nogc { super(); } }

///
class SysCallException : SerialPortException
{
    /// sys call name
    string fnc;
    /// errno or GetLastError
    int err;
    private this() @safe pure nothrow @nogc { super(); }
}

///
class ReadException : SysCallException
{ private this() @safe pure nothrow @nogc { super(); } }

///
class WriteException : SysCallException
{ private this() @safe pure nothrow @nogc { super(); } }

private E setFields(E: SerialPortException)(E e, string port, string msg,
                                              string file, size_t line)
{
    e.port = port;
    e.msg = msg;
    e.file = file;
    e.line = line;
    return e;
}

import std.format;

private enum preallocated;
private enum prealloc_prefix = "prealloc";

private mixin template throwSPEMix(E, string defaultMsg="")
    if (is(E: SerialPortException))
{
    enum string name = E.stringof;
    mixin(`
    @preallocated private %1$s %2$s%1$s;
    void throw%1$s(string port, string msg="%3$s",
                   string file=__FILE__, size_t line=__LINE__) @nogc
    { throw %2$s%1$s.setFields(port, msg, file, line); }
    `.format(name, prealloc_prefix, (defaultMsg.length ? defaultMsg : name))
    );
}

private enum fmtSPSCEMsgFmt = "call '%s' (%s) failed: error %d";

private string fmtSPSCEMsg(string port, string fnc, int err) @nogc
{
    import core.stdc.stdio : sprintf;
    import std.algorithm : min;
    import core.stdc.string : memcpy, memset;

    enum SZ = 256;

    static char[SZ] port_buf;
    static char[SZ] fnc_buf;
    static char[SZ*3] buf;

    memset(port_buf.ptr, 0, SZ);
    memset(fnc_buf.ptr, 0, SZ);
    memset(buf.ptr, 0, SZ*3);
    memcpy(port_buf.ptr, port.ptr, min(port.length, SZ));
    memcpy(fnc_buf.ptr, fnc.ptr, min(fnc.length, SZ));
    auto n = sprintf(buf.ptr, fmtSPSCEMsgFmt, fnc_buf.ptr, port_buf.ptr, err);
    return cast(string)buf[0..n];
}

unittest
{
    import std.format : format;

    static auto fmtSPSCEMsgGC(string port, string fnc, int err)
    { return format!fmtSPSCEMsgFmt(fnc, port, err); }

    void test(string port, string fnc, int err)
    {
        auto trg = fmtSPSCEMsg(port, fnc, err);
        auto tst = fmtSPSCEMsgGC(port, fnc, err);
        if (trg != tst) assert(0, "not equals:\n%s\n%s".format(trg, tst));
    }

    test("/dev/ttyUSB0", "open", 2);
    test("/very/very/very/very/very/very/very/very/very/very/big/path/to/com/port/device/dev",
         "veryVeryVeryVeryLongFunctionName12345678901234567890123456789012345678901234567890",
         int.max);
    test("", "", 0);
}

private mixin template throwSPSCEMix(E)
    if (is(E: SysCallException))
{
    enum name = E.stringof;
    mixin(`
    @preallocated private %1$s %2$s%1$s;
    void throw%1$s(string port, string fnc, int err, string msg="",
                    string file=__FILE__, size_t line=__LINE__) @nogc
    { 
        if (msg.length == 0)
            msg = fmtSPSCEMsg(port, fnc, err);
        auto e = %2$s%1$s.setFields(port, msg, file, line);
        e.fnc = fnc;
        e.err = err;
        throw e;
    }
    `.format(name, prealloc_prefix)
    );
}

static this()
{
    // can't use origin getSymbolsByUDA because
    // https://issues.dlang.org/show_bug.cgi?id=20054
    // paste old impl at end of file
    static if (__VERSION__ != 2088) import std.traits : getSymbolsByUDA;
    static foreach (sym; getSymbolsByUDA!(mixin(__MODULE__), preallocated))
        sym = new typeof(sym);
}

mixin throwSPEMix!SerialPortException;
mixin throwSPEMix!PortClosedException;
mixin throwSPEMix!TimeoutException;

mixin throwSPSCEMix!SysCallException;
mixin throwSPSCEMix!ReadException;
mixin throwSPSCEMix!WriteException;


import serialport.types;
import core.stdc.stdio;

private char[1024] UEMPB;

@preallocated
private UnsupportedException preallocUnsupported;

void throwUnsupportedException(string port, int baudrate,
                               string file=__FILE__, size_t line=__LINE__) @nogc
{
    auto ln = sprintf(UEMPB.ptr, "unsupported baudrate: %d", baudrate);
    throw preallocUnsupported.setFields(port, cast(immutable)UEMPB[0..ln], file, line);
}

void throwUnsupportedException(string port, DataBits dbits,
                          string file=__FILE__, size_t line=__LINE__) @nogc
{
    auto ln = sprintf(UEMPB.ptr, "unsupported data bits: %d", cast(int)dbits);
    throw preallocUnsupported.setFields(port, cast(immutable)UEMPB[0..ln], file, line);
}

void throwUnsupportedException(string port, StopBits sbits,
                           string file=__FILE__, size_t line=__LINE__) @nogc
{
    string str;
    final switch (sbits) with (StopBits)
    {
        case one: str = "1\0"; break;
        case two: str = "2\0"; break;
        case onePointFive: str = "1.5\0"; break;
    }

    auto ln = sprintf(UEMPB.ptr, "unsupported stop bits: %s", str.ptr);
    throw preallocUnsupported.setFields(port, cast(immutable)UEMPB[0..ln], file, line);
}

void throwUnsupportedException(string port, Parity parity,
                           string file=__FILE__, size_t line=__LINE__) @nogc
{
    string str;
    final switch (parity) with (Parity)
    {
        case none: str = "none\0"; break;
        case even: str = "even\0"; break;
        case odd:  str = "odd\0"; break;
    }
    auto ln = sprintf(UEMPB.ptr, "unsupported parity: %s", str.ptr);
    throw preallocUnsupported.setFields(port, cast(immutable)UEMPB[0..ln], file, line);
}

static if (__VERSION__ == 2088)
{
    mixin(q{

    // use old version of getSymbolsByUDA
    private template getSymbolsByUDA(alias symbol, alias attribute)
    {
        import std.traits : hasUDA;
        alias membersWithUDA = getSymbolsByUDAImpl!(symbol, attribute, __traits(allMembers, symbol));

        // if the symbol itself has the UDA, tack it on to the front of the list
        static if (hasUDA!(symbol, attribute))
            alias getSymbolsByUDA = AliasSeq!(symbol, membersWithUDA);
        else
            alias getSymbolsByUDA = membersWithUDA;
    }

    private template getSymbolsByUDAImpl(alias symbol, alias attribute, names...)
    {
        import std.meta : Alias, AliasSeq, Filter;
        static if (names.length == 0)
        {
            alias getSymbolsByUDAImpl = AliasSeq!();
        }
        else
        {
            alias tail = getSymbolsByUDAImpl!(symbol, attribute, names[1 .. $]);

            // Filtering inaccessible members.
            static if (!__traits(compiles, __traits(getMember, symbol, names[0])))
            {
                alias getSymbolsByUDAImpl = tail;
            }
            else
            {
                alias member = __traits(getMember, symbol, names[0]);

                // Filtering not compiled members such as alias of basic types.
                static if (!__traits(compiles, hasUDA!(member, attribute)))
                {
                    alias getSymbolsByUDAImpl = tail;
                }
                // Get overloads for functions, in case different overloads have different sets of UDAs.
                else static if (isFunction!member)
                {
                    enum hasSpecificUDA(alias member) = hasUDA!(member, attribute);
                    alias overloadsWithUDA = Filter!(hasSpecificUDA, __traits(getOverloads, symbol, names[0]));
                    alias getSymbolsByUDAImpl = AliasSeq!(overloadsWithUDA, tail);
                }
                else static if (hasUDA!(member, attribute))
                {
                    alias getSymbolsByUDAImpl = AliasSeq!(member, tail);
                }
                else
                {
                    alias getSymbolsByUDAImpl = tail;
                }
            }
        }
    }
    
    });
}