/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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
module tests.test_entries;

import framework.test;

import tests.addpair;
import tests.applyforce;
import tests.basicslidercrank;
import tests.bodytypes;
import tests.breakable;
import tests.bridge;
import tests.bullettest;
import tests.cantilever;
import tests.car;
import tests.continuoustest;
import tests.chain;
import tests.charactercollision;
import tests.collisionfiltering;
import tests.collisionprocessing;
import tests.compoundshapes;
import tests.confined;
import tests.convexhull;
import tests.conveyorbelt;
import tests.distancetest;
import tests.dominos;
import tests.dumpshell;
import tests.dynamictreetest;
import tests.edgeshapes;
import tests.edgetest;
import tests.gears;
import tests.heavyonlight;
import tests.heavyonlighttwo;
//~ import tests.mobile;
//~ import tests.mobilebalanced;
//~ import tests.motorjoint;
//~ import tests.onesidedplatform;
//~ import tests.pinball;
//~ import tests.polycollision;
//~ import tests.polyshapes;
//~ import tests.prismatic;
//~ import tests.pulleys;
//~ import tests.pyramid;
//~ import tests.raycast;
//~ import tests.revolute;
//~ import tests.ropejoint;
//~ import tests.sensortest;
//~ import tests.shapeediting;
//~ import tests.slidercrank;
//~ import tests.spherestack;
//~ import tests.theojansen;
import tests.tiles;

//~ import tests.timeofimpact;
//~ import tests.tumbler;
//~ import tests.varyingfriction;
//~ import tests.varyingrestitution;
import tests.verticalstack;

//~ import tests.web;

TestEntry[] g_testEntries;

shared static this()
{
    g_testEntries =
    [
        TestEntry("Tiles", &Tiles.Create),
        TestEntry("Heavy on Light", &HeavyOnLight.Create),
        TestEntry("Heavy on Light Two", &HeavyOnLightTwo.Create),
        TestEntry("Vertical Stack", &VerticalStack.Create),
        TestEntry("Basic Slider Crank", &BasicSliderCrank.Create),
        //~ TestEntry("Slider Crank", &SliderCrank.Create),
        //~ TestEntry("Sphere Stack", &SphereStack.Create),
        TestEntry("Convex Hull", &ConvexHull.Create),
        //~ TestEntry("Tumbler", &Tumbler.Create),
        //~ TestEntry("Ray-Cast", &RayCast.Create),
        TestEntry("Dump Shell", &DumpShell.Create),
        TestEntry("Apply Force", &ApplyForce.Create),
        TestEntry("Continuous Test", &ContinuousTest.Create),
        //~ TestEntry("Time of Impact", &TimeOfImpact.Create),
        //~ TestEntry("Motor Joint", &MotorJoint.Create),
        //~ TestEntry("One-Sided Platform", &OneSidedPlatform.Create),
        //~ TestEntry("Mobile", &Mobile.Create),
        //~ TestEntry("MobileBalanced", &MobileBalanced.Create),
        TestEntry("Conveyor Belt", &ConveyorBelt.Create),
        TestEntry("Gears", &Gears.Create),
        //~ TestEntry("Varying Restitution", &VaryingRestitution.Create),
        TestEntry("Cantilever", &Cantilever.Create),
        TestEntry("Character Collision", &CharacterCollision.Create),
        TestEntry("Edge Test", &EdgeTest.Create),
        TestEntry("Body Types", &BodyTypes.Create),
        //~ TestEntry("Shape Editing", shapeEditing.Create),
        TestEntry("Car", &Car.Create),  // broken
        //~ TestEntry("Prismatic", &Prismatic.Create),
        //~ TestEntry("Revolute", &Revolute.Create),
        //~ TestEntry("Pulleys", &Pulleys.Create),
        //~ TestEntry("Polygon Shapes", &PolyShapes.Create),
        //~ TestEntry("Web", &Web.Create),
        //~ TestEntry("RopeJoint", &RopeJoint.Create),
        //~ TestEntry("Pinball", &Pinball.Create),
        TestEntry("Bullet Test", &BulletTest.Create),
        TestEntry("Confined", &Confined.Create),  // broken
        //~ TestEntry("Pyramid", &Pyramid.Create),
        //~ TestEntry("Theo Jansen's Walker", &TheoJansen.Create),
        TestEntry("Edge Shapes", &EdgeShapes.Create),
        //~ TestEntry("PolyCollision", &PolyCollision.Create),
        TestEntry("Bridge", &Bridge.Create),
        TestEntry("Breakable", &Breakable.Create),
        TestEntry("Chain", &Chain.Create),
        TestEntry("Collision Filtering", &CollisionFiltering.Create),
        TestEntry("Collision Processing", &CollisionProcessing.Create),
        TestEntry("Compound Shapes", &CompoundShapes.Create),
        TestEntry("Distance Test", &DistanceTest.Create),
        TestEntry("Dominos", &Dominos.Create),
        TestEntry("Dynamic Tree", &DynamicTreeTest.Create),
        //~ TestEntry("Sensor Test", &SensorTest.Create),
        //~ TestEntry("Varying Friction", &VaryingFriction.Create),
        TestEntry("Add Pair Stress Test", &AddPair.Create),
    ];

    import std.algorithm;
    sort!((a, b) => a.name < b.name)(g_testEntries);
}
