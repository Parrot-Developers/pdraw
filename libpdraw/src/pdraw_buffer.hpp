/**
 * @file pdraw_buffer.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - buffer
 * @date 05/11/2016
 * @author aurelien.barre@akaaba.net
 *
 * Copyright (c) 2016 Aurelien Barre <aurelien.barre@akaaba.net>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 * 
 *   * Neither the name of the copyright holder nor the names of the
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PDRAW_BUFFER_HPP_
#define _PDRAW_BUFFER_HPP_

#include <vector>
#include <queue>
#include <stdint.h>
#include <string.h>
#include <pthread.h>


namespace Pdraw
{


class BufferPool;


class Buffer
{
public:

    Buffer(BufferPool *bufferPool,
           unsigned int id,
           void *userPtr,
           uint8_t *preallocBuf,
           unsigned int capacity,
           bool alloc,
           unsigned int metadataSize,
           int(*bufferCreationCb)(Buffer *buffer),
           int(*bufferDeletionCb)(Buffer *buffer));

    ~Buffer();

    void ref();

    void unref();

    bool isRef() { return (mRefCount == 0) ? false : true; };

    void *getPtr();

    void setPtr(void *ptr);

    void *getResPtr();

    void setResPtr(void *resPtr);

    unsigned int getCapacity();

    void setCapacity(unsigned int capacity);

    unsigned int getSize();

    void setSize(unsigned int size);

    unsigned int getId();

    void *getMetadataPtr();

    unsigned int getMetadataSize();

    void *getUserPtr();

private:

    friend class BufferPool;

    unsigned int getRefCount();

    void setRefCount(unsigned int refCount);

    BufferPool *mBufferPool;
    int(*mBufferDeletionCb)(Buffer *buffer);
    unsigned int mRefCount;
    unsigned int mId;
    void *mUserPtr;
    unsigned int mCapacity;
    unsigned int mSize;
    bool mAlloc;
    void *mPtr;
    void *mResPtr;
    void *mMetadataPtr;
    unsigned int mMetadataSize;
};


class BufferPool
{
public:

    BufferPool(unsigned int bufferCount,
               unsigned int bufferSize,
               unsigned int bufferMetadataSize,
               int(*bufferCreationCb)(Buffer *buffer),
               int(*bufferDeletionCb)(Buffer *buffer));

    ~BufferPool();

    void signal();

    Buffer *getBuffer(bool blocking);

    void putBuffer(Buffer *buffer);

private:

    bool bufferIsValid(Buffer *buffer);

    unsigned int mBufferCount;
    unsigned int mBufferSize;
    std::vector<Buffer*> *mBuffers;
    std::queue<Buffer*> *mPool;
    pthread_mutex_t mMutex;
    pthread_cond_t mCond;
};


class BufferQueue
{
public:

    BufferQueue();

    ~BufferQueue();

    void signal();

    void flush();

    Buffer *peekBuffer(bool blocking);

    Buffer *popBuffer(bool blocking);

    void pushBuffer(Buffer *buffer);

private:

    std::queue<Buffer*> *mQueue;
    pthread_mutex_t mMutex;
    pthread_cond_t mCond;
};

}

#endif /* !_PDRAW_BUFFER_HPP_ */
