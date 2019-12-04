///
module serialport.nonblock;

import serialport.base;

/// Non-blocking work with serial port
class SerialPortNonBlk : SerialPortBase
{
    /++ Construct SerialPortNonBlk

        See_Also: SerialPortBase.this
     +/
    this(string exmode) { super(exmode); }

    /// ditto
    this(string port, string mode) { super(port, mode); }

    /// ditto
    this(string port, uint baudRate) { super(port, baudRate); }

    /// ditto
    this(string port, uint baudRate, string mode)
    { super(port, baudRate, mode); }

    /// ditto
    this(string port, Config conf) { super(port, conf); }

    /++ Non-block read data from port

        Params:
            buf = preallocated buffer for reading

        Returns: slice of buf with readed data

        Throws:
            PortClosedException if port closed
            ReadException if read error occurs
     +/
    void[] read(void[] buf) @nogc { return m_read(buf); }

    /++ Non-block write data to port

        Params:
            arr = data for writing
        
        Returns: count of writen bytes

        Throws:
            PortClosedException if port closed
            WriteException if read error occurs
     +/
    size_t write(string buf) @nogc { return m_write(buf); }
}