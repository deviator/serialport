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

private mixin template throwSPEMix(E, string defaultMsg="")
    if (is(E: SerialPortException))
{
    enum name = E.stringof;
    mixin(`
    @preallocated
    private %1$s %3$s%1$s;
    void throw%1$s(string port, string msg="%2$s",
                    string file=__FILE__, size_t line=__LINE__) @nogc
    { throw %3$s%1$s.setFields(port, msg, file, line); }
    `.format(name, defaultMsg, "prealloc")
    );
}

private mixin template throwSPSCEMix(E, string defaultMsg="")
    if (is(E: SysCallException))
{
    enum name = E.stringof;
    mixin(`
    @preallocated
    private %1$s %3$s%1$s;
    void throw%1$s(string port, string fnc, int err, string msg="%2$s",
                    string file=__FILE__, size_t line=__LINE__) @nogc
    { 
        auto e = %3$s%1$s.setFields(port, msg, file, line);
        e.fnc = fnc;
        e.err = err;
        throw e;
    }
    `.format(name, defaultMsg, "prealloc")
    );
}

static this()
{
    import std.traits : getSymbolsByUDA;
    static foreach (sym; getSymbolsByUDA!(mixin(__MODULE__), preallocated))
        sym = new typeof(sym);
}

mixin throwSPEMix!SerialPortException;
mixin throwSPEMix!(PortClosedException, "port closed");
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