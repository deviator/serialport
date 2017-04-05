module serialport.exception;

import std.conv : text;

import serialport.types;

///
class SerialPortException : Exception
{
    ///
    this(string msg, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow @nogc
    { super(msg, file, line); }
}

///
class UnsupportedException : SerialPortException
{
    ///
    this(string msg, string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow @nogc
    { super(msg, file, line); }
}

///
class BaudRateUnsupportedException : UnsupportedException
{
    ///
    uint baudRate;

    ///
    this(uint br, string file=__FILE__, size_t line=__LINE__)
    {
        baudRate = br;
        super(text("Unsupported baudrate ", br), file, line);
    }
}

///
class ParityUnsupportedException : UnsupportedException
{
    ///
    Parity parity;

    ///
    this(Parity p, string file=__FILE__, size_t line=__LINE__)
    {
        parity = p;
        super(text("Unsupported parity ", p), file, line);
    }
}

///
class DataBitsUnsupportedException : UnsupportedException
{
    ///
    DataBits dataBits;

    ///
    this(DataBits db, string file=__FILE__, size_t line=__LINE__)
    {
        dataBits = db;
        super(text("Unsupported data bits ", db), file, line);
    }
}

///
class StopBitsUnsupportedException : UnsupportedException
{
    ///
    StopBits stopBits;

    ///
    this(StopBits sb, string file=__FILE__, size_t line=__LINE__)
    {
        stopBits = sb;
        super(text("Unsupported stop bits ", sb), file, line);
    }
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

///
class SetupFailException : SerialPortException
{
    ///
    string port;

    ///
    this(string port, string reason="", string file=__FILE__, size_t line=__LINE__)
        @safe pure nothrow
    {
        this.port = port;
        super("Unable to open serial port '" ~ port ~ "'" ~
                    (reason ? " (" ~ reason ~ ")" : ""), file, line);
    }
}

///
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
