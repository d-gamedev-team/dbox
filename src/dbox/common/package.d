module dbox.common;

import std.conv;

template getSizeOf(T) if (is(T == class))
{
    enum getSizeOf = __traits(classInstanceSize, T);
}

template getSizeOf(T) if (!is(T == class))
{
    enum getSizeOf = T.sizeof;
}

T emplace(T)(void* chunk) if (is(T == class))
{
    return std.conv.emplace!T(chunk[0 .. getSizeOf!T]);
}

T* emplace(T)(void* chunk) if (!is(T == class))
{
    return std.conv.emplace!T(chunk[0 .. getSizeOf!T]);
}

T emplace(T, Args...)(void* chunk, Args args) if (is(T == class))
{
    return std.conv.emplace!T(chunk[0 .. getSizeOf!T], args);
}

T* emplace(T, Args...)(void* chunk, Args args) if (!is(T == class))
{
    return std.conv.emplace!T(chunk[0 .. getSizeOf!T], args);
}

/**
    Export all enum members as aliases. This allows enums to be used as types
    and allows its members to be used as if they're defined in module scope.
*/
mixin template _ExportEnumMembers(E) if (is(E == enum))
{
    mixin(_makeEnumAliases!(E)());
}

/// ditto
string _makeEnumAliases(E)() if (is(E == enum))
{
    import std.array;
    import std.string;

    enum enumName = __traits(identifier, E);
    Appender!(string[]) result;

    foreach (string member; __traits(allMembers, E))
        result ~= format("alias %s = %s.%s;", member, enumName, member);

    return result.data.join("\n");
}

public
{
    import dbox.common.b2blockallocator;
    import dbox.common.b2draw;
    import dbox.common.b2growablestack;
    import dbox.common.b2math;
    import dbox.common.b2settings;
    import dbox.common.b2stackallocator;
    import dbox.common.b2timer;
}
