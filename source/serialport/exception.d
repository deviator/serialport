///
module serialport.exception;

import std.conv : text;

import serialport.types;

/// General
class SerialPortException : Exception
{
    ///
    this(string msg, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow @nogc
    { super(msg, file, line); }
}

///
class SysCallException : SerialPortException
{
    /// errno or GetLastError
    int err;

    this(string msg, int err, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow @nogc
    {
        this.err = err;
        super(msg, file, line);
    }
}

///
class ParseModeException : SerialPortException
{
    ///
    this(string msg, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow @nogc
    { super(msg, file, line); }
}

/// Unsupported config
class UnsupportedException : SerialPortException
{
    ///
    this(string msg, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow @nogc
    { super(msg, file, line); }

    ///
    this(uint br, string file=__FILE__, size_t line=__LINE__)
    { this(text("Unsupported baudrate ", br), file, line); }

    ///
    this(Parity p, string file=__FILE__, size_t line=__LINE__)
    { this(text("Unsupported parity ", p), file, line); }

    ///
    this(DataBits db, string file=__FILE__, size_t line=__LINE__)
    { this(text("Unsupported data bits ", db), file, line); }

    ///
    this(StopBits sb, string file=__FILE__, size_t line=__LINE__)
    { this(text("Unsupported stop bits ", sb), file, line); }
}

///
class PortClosedException : SerialPortException
{
    ///
    string port;

    ///
    this(string port, string file=__FILE__, size_t line=__LINE__) @safe pure nothrow
    {
        this.port = port;
        super("serial port '" ~ port ~ "' is closed", file, line);
    }
}

/// Base for timeout, read and write exceptions
class IOException : SerialPortException
{
    ///
    string port;

    ///
    this(string port, string msg, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow @nogc
    { super(msg, file, line); }
}

///
class TimeoutException : IOException
{
    ///
    this(string port, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow
    { super(port, "Timeout is expires for '" ~ port ~ "'", file, line); }
}

///
class ReadException : IOException
{
    ///
    this(string port, string msg, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow
    { super(port, "Failed to read from '" ~ port ~ "': " ~ msg, file, line); }
}

///
class WriteException : IOException
{
    ///
    this(string port, string msg, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow
    { super(port, "Failed to write to '" ~ port ~ "': " ~ msg, file, line); }
}
