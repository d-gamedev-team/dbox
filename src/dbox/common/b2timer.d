module dbox.common.b2timer;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.common.b2math;


import std.datetime;

import dbox.common;

/// Workaround for missing default struct ctors in D.
@property B2Timer b2Timer() { return B2Timer(1); }

/// Timer for profiling. This has platform specific code and may
/// not work on every platform.
struct B2Timer
{
    /// Constructor
    this(int)
    {
        sw.start();
    }

    /// Reset the timer.
    void Reset()
    {
        sw.reset();
    }

    /// Get the time since construction or the last reset.
    float32 GetMilliseconds() const
    {
        return sw.peek.to!("msecs", float);
    }

/* private */

    StopWatch sw;
}
