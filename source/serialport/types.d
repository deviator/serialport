/// Definitions of using types
module serialport.types;

///
enum Parity
{
    none, ///
    odd,  ///
    even  ///
}

///
enum DataBits : uint
{
   data8 = 8, ///
   data7 = 7, ///
   data6 = 6, ///
   data5 = 5, ///
}

///
enum StopBits
{
   one, ///
   onePointFive, ///
   two ///
}

package
{
    version(Posix)
    {
        import core.sys.posix.unistd;
        version(linux)
            import core.sys.linux.termios;
        else
            import core.sys.posix.termios;

        import core.sys.posix.fcntl;
        import core.sys.posix.sys.select;
        import core.sys.posix.sys.ioctl;
        import core.stdc.errno;
        import std.algorithm;
        import std.file;

        alias posixRead = core.sys.posix.unistd.read;
        alias posixWrite = core.sys.posix.unistd.write;
        alias posixClose = core.sys.posix.unistd.close;

        alias closeHandle = posixClose;
        /// Posix handle is int
        alias SPHandle = int;
        enum initHandle = -1;
    }
    version(Windows)
    {
        import core.sys.windows.windows;
        import std.bitmanip;

        enum NOPARITY    = 0x0;
        enum EVENPARITY  = 0x2;
        enum MARKPARITY  = 0x3;
        enum ODDPARITY   = 0x1;
        enum SPACEPARITY = 0x4;

        enum ONESTOPBIT   = 0x0;
        enum ONE5STOPBITS = 0x1;
        enum TWOSTOPBITS  = 0x2;

        struct DCB
        {
            DWORD DCBlength;
            DWORD BaudRate;

            mixin(bitfields!(
            DWORD, "fBinary",           1,
            DWORD, "fParity",           1,
            DWORD, "fOutxCtsFlow",      1,
            DWORD, "fOutxDsrFlow",      1,
            DWORD, "fDtrControl",       2,
            DWORD, "fDsrSensitivity",   1,
            DWORD, "fTXContinueOnXoff", 1,
            DWORD, "fOutX",             1,
            DWORD, "fInX",              1,
            DWORD, "fErrorChar",        1,
            DWORD, "fNull",             1,
            DWORD, "fRtsControl",       2,
            DWORD, "fAbortOnError",     1,
            DWORD, "fDummy2",           17));

            WORD  wReserved;
            WORD  XonLim;
            WORD  XoffLim;
            BYTE  ByteSize;
            BYTE  Parity;
            BYTE  StopBits;
            ubyte  XonChar;
            ubyte  XoffChar;
            ubyte  ErrorChar;
            ubyte  EofChar;
            ubyte  EvtChar;
            WORD  wReserved1;
        }

        struct COMMTIMEOUTS
        {
            DWORD ReadIntervalTimeout;
            DWORD ReadTotalTimeoutMultiplier;
            DWORD ReadTotalTimeoutConstant;
            DWORD WriteTotalTimeoutMultiplier;
            DWORD WriteTotalTimeoutConstant;
        }

        extern(Windows) @nogc
        {
            bool GetCommState(HANDLE hFile, DCB* lpDCB);
            bool SetCommState(HANDLE hFile, DCB* lpDCB);
            bool SetCommTimeouts(HANDLE hFile, COMMTIMEOUTS* lpCommTimeouts);
        }

        alias closeHandle = CloseHandle;
        /// Windows is HANDLE
        alias SPHandle = HANDLE;
        enum initHandle = null;
    }
}

///
static immutable string modeSplitChar=":";

version (Posix)
{
    import serialport.util;

    version (OSX)
    {
        /+
        38400 is max for OSX
        http://www.manpages.info/macosx/cfsetospeed.3.html
         +/
        static immutable unixBaudList = pairList(
            pair(    0, B0),
            pair(   50, B50),
            pair(   75, B75),
            pair(  110, B110),
            pair(  134, B134),
            pair(  150, B150),   
            pair(  200, B200),  
            pair(  300, B300),
            pair(  600, B600),
            pair( 1200, B1200), 
            pair( 1800, B1800), 
            pair( 2400, B2400),
            pair( 4800, B4800),
            pair( 9600, B9600),
            pair(19200, B19200),
            pair(38400, B38400)
        );
    }
    else
    {
        static immutable unixBaudList = pairList(
            pair(     0, B0),
            pair(    50, B50), 
            pair(    75, B75),
            pair(   110, B110),
            pair(   134, B134),
            pair(   150, B150),   
            pair(   200, B200),  
            pair(   300, B300),
            pair(   600, B600),
            pair(  1200, B1200), 
            pair(  1800, B1800), 
            pair(  2400, B2400),
            pair(  4800, B4800),
            pair(  9600, B9600),
            pair( 19200, B19200),
            pair( 38400, B38400),
            pair( 57600, B57600),
            pair(115200, B115200),
            pair(230400, B230400) 
        );
    }

    static assert(unixBaudList.isUniqA, "not uniq A vals in unix baud list");
    static assert(unixBaudList.isUniqB, "not uniq B vals in unix baud list");
}