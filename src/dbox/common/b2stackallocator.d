module dbox.common.b2stackallocator;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.common.b2math;

/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

// #ifndef B2_STACK_ALLOCATOR_H
// #define B2_STACK_ALLOCATOR_H

import dbox.common;

const int32 b2_stackSize       = 100 * 1024; // 100k
const int32 b2_maxStackEntries = 32;

struct b2StackEntry
{
    byte* data;
    int32 size;
    bool usedMalloc;
}

// This is a stack allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will assert
// if you try to interleave multiple allocate/free pairs.
struct b2StackAllocator
{
    ~this()
    {
        assert(m_index == 0);
        assert(m_entryCount == 0);
    }

    void* Allocate(int32 size)
    {
        assert(m_entryCount < b2_maxStackEntries);

        b2StackEntry* entry = m_entries.ptr + m_entryCount;
        entry.size = size;

        if (m_index + size > b2_stackSize)
        {
            entry.data       = cast(byte*)b2Alloc(size);
            entry.usedMalloc = true;
        }
        else
        {
            entry.data       = m_data.ptr + m_index;
            entry.usedMalloc = false;
            m_index += size;
        }

        m_allocation   += size;
        m_maxAllocation = b2Max(m_maxAllocation, m_allocation);
        ++m_entryCount;

        return entry.data;
    }

    void Free(void* p)
    {
        assert(m_entryCount > 0);
        b2StackEntry* entry = m_entries.ptr + m_entryCount - 1;
        assert(p == entry.data);

        if (entry.usedMalloc)
        {
            b2Free(p);
        }
        else
        {
            m_index -= entry.size;
        }
        m_allocation -= entry.size;
        --m_entryCount;

        p = null;
    }

    int32 GetMaxAllocation() const
    {
        return m_maxAllocation;
    }

    byte m_data[b2_stackSize];
    int32 m_index;

    int32 m_allocation;
    int32 m_maxAllocation;

    b2StackEntry m_entries[b2_maxStackEntries];
    int32 m_entryCount;
}

import dbox.common.b2stackallocator;
import dbox.common.b2math;
