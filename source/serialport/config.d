///
module serialport.config;

import std.range : split;
import std.string : format, toLower;
import std.exception : enforce, assertThrown, assertNotThrown;
import std.conv : to;

import serialport.types;
import serialport.exception;

///
struct SPConfig
{
    ///
    uint baudRate=9600;
    ///
    DataBits dataBits=DataBits.data8;
    ///
    Parity parity=Parity.none;
    ///
    StopBits stopBits=StopBits.one;

    ///
    bool hardwareDisableFlowControl = true;

    /++ Set parity value
        Returns: this
        +/
    ref SPConfig set(Parity v) { parity = v; return this; }

    /++ Set baudrate value
        Returns: this
        +/
    ref SPConfig set(uint v) { baudRate = v; return this; }

    /++ Set data bits value
        Returns: this
        +/
    ref SPConfig set(DataBits v) { dataBits = v; return this; }

    /++ Set stop bits value
        Returns: this
        +/
    ref SPConfig set(StopBits v) { stopBits = v; return this; }

    /++
        Use mode string for setting baudrate, data bits, parity and stop bits.

        Format: "B:DPS"
        where:
            B is baud rate
            D is data bits (5, 6, 7, 8)
            P is parity ('N' or 'n' -- none,
                            'E' or 'e' -- even,
                            'O' or 'o' -- odd)
            S is stop bits ('1', '1.5', '2')

        You can skip baudrate.

        example mode strings: "9600:8N1" ":8n1" "7o1.5" "2400:6e2"

        Throws:
            ParseModeException if mode string is badly formatted or using bad values
        +/
    ref SPConfig set(string mode)
    {
        alias PME = ParseModeException;

        auto errstr = "error mode '%s'".format(mode);
        enforce(mode.length >= 3, new PME(errstr ~ ": too short"));

        auto vals = mode.split(modeSplitChar);

        if (vals.length == 0) return this;

        if (vals.length > 2)
            throw new PME(errstr ~ ": many parts");

        if (vals.length == 2)
        {
            if (vals[0].length)
            {
                try baudRate = vals[0].to!uint;
                catch (Exception e)
                    throw new PME(errstr ~
                            ": baud rate parse error: " ~ e.msg);
            }
            mode = vals[1];
        }
        else mode = vals[0];

        auto db = cast(int)mode[0] - cast(int)'0';
        if (db >= 5 && db <= 8) dataBits = cast(DataBits)db;
        else throw new PME(errstr ~ ": unsupported data bits '" ~ mode[0] ~ "'");

        auto p = mode[1..2].toLower;
        if (p == "n" || p == "o" || p == "e")
        {
            parity = ["n": Parity.none,
                        "o": Parity.odd,
                        "e": Parity.even][p];
        }
        else throw new PME(errstr ~ ": unsupported parity '" ~ p ~ "'");

        auto sb = mode[2..$];
        if (sb == "1" || sb == "1.5" || sb == "2")
        {
            stopBits = ["1": StopBits.one,
                        "1.5": StopBits.onePointFive,
                        "2": StopBits.two][sb];
        }
        else throw new PME(errstr ~ ": unsupported stop bits '" ~ sb ~ "'");

        return this;
    }

    ///
    unittest
    {
        SPConfig c;
        c.set("2400:7e1.5");
        assertNotThrown(c.set(c.mode));
        assert(c.baudRate == 2400);
        assert(c.dataBits == DataBits.data7);
        assert(c.parity == Parity.even);
        assert(c.stopBits == StopBits.onePointFive);
        c.set("8N1");
        assertNotThrown(c.set(c.mode));
        assert(c.baudRate == 2400);
        assert(c.dataBits == DataBits.data8);
        assert(c.parity == Parity.none);
        assert(c.stopBits == StopBits.one);
        c.set("320:5o2");
        assertNotThrown(c.set(c.mode));
        assert(c.baudRate == 320);
        assert(c.dataBits == DataBits.data5);
        assert(c.parity == Parity.odd);
        assert(c.stopBits == StopBits.two);

        alias PME = ParseModeException;
        assertThrown!PME(c.set("4o2"));
        assertThrown!PME(c.set("5x2"));
        assertThrown!PME(c.set("8e3"));
        assertNotThrown!PME(c.set(":8N1"));
        assertNotThrown(c.set(c.mode));
    }

    /++ Construct config, parse mode to it and return.

        Returns: new config

        See_Also: set(string mode)
        +/
    static SPConfig parse(string mode)
    {
        SPConfig ret;
        ret.set(mode);
        return ret;
    }

    /++ Build mode string.

        Can be used for parsing.

        Returns: mode string

        See_Also: parse, set(string mode)
        +/
    string mode() const @property
    {
        return "%s:%s%s%s".format(
            baudRate,
            dataBits.to!int,
            [Parity.none: "n",
                Parity.odd:  "o",
                Parity.even: "e"][parity],
            [StopBits.one: "1",
                StopBits.onePointFive: "1.5",
                StopBits.two: "2"
            ][stopBits]
        );
    }
}

unittest
{
    SPConfig a, b;
    a.set("2400:7e2");
    b.set(a.mode);
    assert(a == b);
    a.set(Parity.none).set(DataBits.data8).set(19200).set(StopBits.one);
    assert(a.parity == Parity.none);
    assert(a.dataBits == DataBits.data8);
    assert(a.baudRate == 19200);
    assert(a.stopBits == StopBits.one);
}

unittest
{
    SPConfig a;
    assertThrown!ParseModeException(a.set("2400:7e2:32"));
    assertThrown!ParseModeException(a.set("24a0:7e2"));
}
