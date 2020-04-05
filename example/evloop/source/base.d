module base;

public
{
    import std.datetime : Duration;
    import std.experimental.logger;
    import std.exception : enforce;

    import core.thread : Fiber;

    import core.sys.posix.unistd;
    import core.sys.linux.timerfd;
    import core.sys.linux.sys.signalfd;
    import core.stdc.errno;

    import errproc;
}