/*
 * Copyright (c) 2011 Erin Catto http://box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
module dbox.common.b2timer;

import dbox.common;

import std.datetime;

/// Workaround for missing default struct ctors in D.
@property B2Timer b2Timer()
{
    return B2Timer(1);
}

/// Timer for profiling. This has platform specific code and may
/// not work on every platform.
struct B2Timer
{
    /// This struct must be properly initialized with an explicit constructor.
    @disable this();

    /// This struct cannot be copied.
    @disable this(this);

    /// Explicit constructor.
    /// Start the timer.
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

private:

    StopWatch sw;
}
