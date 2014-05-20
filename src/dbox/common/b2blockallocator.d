module dbox.common.b2blockallocator;

import core.stdc.float_;
import core.stdc.stdlib;
import core.stdc.string;

import dbox.common;
import dbox.common.b2math;

import core.stdc.limits;
import core.stdc.string;

import dbox.common;

enum b2_chunkSize           = 16 * 1024;
enum b2_maxBlockSize        = 640;
enum b2_blockSizes          = 14;
enum b2_chunkArrayIncrement = 128;

/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
struct b2BlockAllocator
{
    @disable this();
    @disable this(this);

    this(int)
    {
        assert(b2_blockSizes < UCHAR_MAX);

        m_chunkSpace = b2_chunkArrayIncrement;
        m_chunkCount = 0;
        m_chunks     = cast(b2Chunk*)b2Alloc(m_chunkSpace * getSizeOf!b2Chunk);

        memset(m_chunks, 0, m_chunkSpace * getSizeOf!b2Chunk);
        m_freeLists[] = null;
    }

    b2Chunk* m_chunks;

    ~this()
    {
        for (int32 i = 0; i < m_chunkCount; ++i)
        {
            b2Free(m_chunks[i].blocks);
        }

        b2Free(m_chunks);
    }

    /// Allocate memory. This will use b2Alloc if the size is larger than b2_maxBlockSize.
    void* Allocate(int32 size)
    {
        if (size == 0)
            return null;

        assert(0 < size);

        if (size > b2_maxBlockSize)
        {
            return b2Alloc(size);
        }

        int32 index = s_blockSizeLookup[size];
        assert(0 <= index && index < b2_blockSizes);

        if (m_freeLists[index])
        {
            b2Block* block = m_freeLists[index];
            m_freeLists[index] = block.next;
            return block;
        }
        else
        {
            if (m_chunkCount == m_chunkSpace)
            {
                b2Chunk* oldChunks = m_chunks;
                m_chunkSpace += b2_chunkArrayIncrement;
                m_chunks      = cast(b2Chunk*)b2Alloc(m_chunkSpace * getSizeOf!b2Chunk);
                memcpy(m_chunks, oldChunks, m_chunkCount * getSizeOf!b2Chunk);
                memset(m_chunks + m_chunkCount, 0, b2_chunkArrayIncrement * getSizeOf!b2Chunk);
                b2Free(oldChunks);
            }

            b2Chunk* chunk = m_chunks + m_chunkCount;
            chunk.blocks = cast(b2Block*)b2Alloc(b2_chunkSize);

            int32 blockSize = s_blockSizes[index];
            chunk.blockSize = blockSize;
            int32 blockCount = b2_chunkSize / blockSize;
            assert(blockCount * blockSize <= b2_chunkSize);

            for (int32 i = 0; i < blockCount - 1; ++i)
            {
                b2Block* block = cast(b2Block*)(cast(int8*)chunk.blocks + blockSize * i);
                b2Block* next  = cast(b2Block*)(cast(int8*)chunk.blocks + blockSize * (i + 1));
                block.next = next;
            }

            b2Block* last = cast(b2Block*)(cast(int8*)chunk.blocks + blockSize * (blockCount - 1));
            last.next = null;

            m_freeLists[index] = chunk.blocks.next;
            ++m_chunkCount;

            return chunk.blocks;
        }
    }

    /// Free memory. This will use b2Free if the size is larger than b2_maxBlockSize.
    void Free(void* p, int32 size)
    {
        if (size == 0)
        {
            return;
        }

        assert(0 < size);

        if (size > b2_maxBlockSize)
        {
            b2Free(p);
            return;
        }

        int32 index = s_blockSizeLookup[size];
        assert(0 <= index && index < b2_blockSizes);

        b2Block* block = cast(b2Block*)p;
        block.next        = m_freeLists[index];
        m_freeLists[index] = block;
    }

    void Clear()
    {
        for (int32 i = 0; i < m_chunkCount; ++i)
        {
            b2Free(m_chunks[i].blocks);
        }

        m_chunkCount = 0;
        memset(m_chunks, 0, m_chunkSpace * getSizeOf!b2Chunk);

        m_freeLists[] = null;
    }

/* private */

    int32 m_chunkCount;
    int32 m_chunkSpace = b2_chunkArrayIncrement;

    b2Block*[b2_blockSizes] m_freeLists;

    static immutable int32[b2_blockSizes] s_blockSizes =
    [
        16,         // 0
        32,         // 1
        64,         // 2
        96,         // 3
        128,        // 4
        160,        // 5
        192,        // 6
        224,        // 7
        256,        // 8
        320,        // 9
        384,        // 10
        448,        // 11
        512,        // 12
        640,        // 13
    ];

    static uint8[b2_maxBlockSize + 1] s_blockSizeLookup = initMe();

    private static uint8[b2_maxBlockSize + 1] initMe()
    {
        uint8[b2_maxBlockSize + 1] res;

        int32 j = 0;

        for (int32 i = 1; i <= b2_maxBlockSize; ++i)
        {
            assert(j < b2_blockSizes);

            if (i <= s_blockSizes[j])
            {
                res[i] = cast(uint8)j;
            }
            else
            {
                ++j;
                res[i] = cast(uint8)j;
            }
        }

        return res;
    }
}

struct b2Chunk
{
    int32 blockSize;
    b2Block* blocks;
}

struct b2Block
{
    b2Block* next;
}
