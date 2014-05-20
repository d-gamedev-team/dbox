module dbox.dynamics.b2timestep;

import core.stdc.stdlib;
import core.stdc.float_;
import core.stdc.string;

import dbox.common;
import dbox.collision;
import dbox.dynamics;
import dbox.dynamics.contacts;
import dbox.dynamics.joints;

/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

// #ifndef B2_TIME_STEP_H
// #define B2_TIME_STEP_H

import dbox.common.b2math;

/// Profiling data. Times are in milliseconds.
struct b2Profile
{
    float32 step = 0;
    float32 collide = 0;
    float32 solve = 0;
    float32 solveInit = 0;
    float32 solveVelocity = 0;
    float32 solvePosition = 0;
    float32 broadphase = 0;
    float32 solveTOI = 0;
}

/// This is an internal structure.
struct b2TimeStep
{
    float32 dt = 0;                 // time step
    float32 inv_dt = 0;             // inverse time step (0 if dt == 0).
    float32 dtRatio = 0;            // dt * inv_dt0
    int32 velocityIterations;
    int32 positionIterations;
    bool warmStarting;
}

/// This is an internal structure.
struct b2Position
{
    b2Vec2 c;
    float32 a = 0;
}

/// This is an internal structure.
struct b2Velocity
{
    b2Vec2 v;
    float32 w = 0;
}

/// Solver Data
struct b2SolverData
{
    b2TimeStep step;
    b2Position* positions;
    b2Velocity* velocities;
}

// #endif
