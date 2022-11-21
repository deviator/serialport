module serialport.util;

import std.range;
import std.algorithm;

import std.datetime.stopwatch;

void msleep(Duration dt) @nogc
{
    import core.thread : Fiber, Thread;
    if (Fiber.getThis is null) Thread.sleep(dt);
    else
    {
        const tm = StopWatch(AutoStart.yes);
        do Fiber.yield(); while (tm.peek < dt);
    }
}

package bool hasFlag(A,B)(A a, B b) @property { return (a & b) == b; }

struct Pair(A,B) { A a; B b; }
auto pair(A,B)(A a, B b) { return Pair!(A,B)(a, b); }

struct PairList(A,B)
{
    Pair!(A,B)[] list;
    this(Pair!(A,B)[] list) { this.list = list; }

    @safe pure @nogc nothrow const
    {
        size_t countA(A a) { return list.map!(a=>a.a).count(a); }
        size_t countB(B b) { return list.map!(a=>a.b).count(b); }

        bool isUniqA() @property { return list.all!(v=>countA(v.a) == 1); }
        bool isUniqB() @property { return list.all!(v=>countB(v.b) == 1); }

        B firstA2B(A a, B defalutValue)
        {
            auto f = list.find!(v=>v.a == a);
            if (f.empty) return defalutValue;
            else return f.front.b;
        }

        A firstB2A(B b, A defalutValue)
        {
            auto f = list.find!(v=>v.b == b);
            if (f.empty) return defalutValue;
            else return f.front.a;
        }
    }

    @safe pure nothrow const
    {
        auto allA2B(A a) { return list.filter!(v=>v.a == a).map!(v=>v.b); }
        auto allB2A(B b) { return list.filter!(v=>v.b == b).map!(v=>v.a); }
    }
}

auto pairList(A,B)(Pair!(A,B)[] list...) { return PairList!(A,B)(list.dup); }

unittest
{
    auto pl = pairList(
        pair(1, "hello"),
        pair(1, "ololo"),
        pair(2, "world"),
        pair(3, "okda")
    );

    assert(pl.countA(1) == 2);
    assert(pl.firstA2B(1, "ok") == "hello");
    assert(pl.countB("ok") == 0);
    assert(pl.countB("okda") == 1);
    assert(pl.firstB2A("okda", 0) == 3);
    assert(pl.isUniqA == false);
    assert(pl.isUniqB == true);
}

unittest
{
    static immutable pl = pairList(
        pair(1, "hello"),
        pair(2, "world"),
        pair(3, "okda")
    );

    static assert(pl.firstA2B(2, "ok") == "world");
}

unittest
{
    import std.algorithm : sum;
    import std.string : join;

    immutable pl = pairList(
        pair(1, "hello"),
        pair(2, "okda"),
        pair(1, "world"),
        pair(3, "okda")
    );

    assert(pl.allA2B(1).join(" ") == "hello world");
    assert(pl.allB2A("okda").sum == 5);
}
