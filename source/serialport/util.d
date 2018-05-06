module serialport.util;

import std.range;
import std.algorithm;

package bool hasFlag(A,B)(A a, B b) @property { return (a & b) == b; }

struct Pair(A,B) { A a; B b; }
auto pair(A,B)(A a, B b) { return Pair!(A,B)(a, b); }

struct PairList(A,B)
{
    Pair!(A,B)[] list;
    this(Pair!(A,B)[] list) { this.list = list.dup; }

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

        auto allA2B(A a) { return list.find!(v=>v.a == a); }
        auto allB2A(B b) { return list.find!(v=>v.b == b); }
    }
}

auto pairList(A,B)(Pair!(A,B)[] list...) { return PairList!(A,B)(list); }

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