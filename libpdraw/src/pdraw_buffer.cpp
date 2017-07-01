/**
 * @file pdraw_buffer.cpp
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

#include "pdraw_buffer.hpp"

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


Buffer::Buffer(BufferPool *bufferPool,
               unsigned int id,
               void *userPtr,
               uint8_t *preallocBuf,
               unsigned int capacity,
               unsigned int metadataSize,
               int(*bufferCreationCb)(Buffer *buffer),
               int(*bufferDeletionCb)(Buffer *buffer))
{
    mBufferPool = bufferPool;
    mBufferDeletionCb = bufferDeletionCb;
    mRefCount = 0;
    mId = id;
    mUserPtr = userPtr;
    mCapacity = capacity;
    mSize = 0;
    mAlloc = ((mCapacity > 0) && (preallocBuf == NULL)) ? true : false;
    mPtr = preallocBuf;
    mResPtr = NULL;
    mMetadataPtr = NULL;
    mMetadataSize = metadataSize;

    if ((mAlloc) && (mCapacity > 0))
    {
        mPtr = (void*)malloc(mCapacity);
        if (mPtr == NULL)
        {
            ULOGE("Buffer: allocation failed (size %d)", mCapacity);
        }
    }

    if (mMetadataSize)
    {
        mMetadataPtr = (void*)malloc(mMetadataSize);
        if (mMetadataPtr == NULL)
        {
            ULOGE("Buffer: metadata allocation failed (size %d)", mMetadataSize);
        }
    }

    if (bufferCreationCb)
    {
        int ret = bufferCreationCb(this);
        if (ret != 0)
        {
            ULOGE("Buffer: buffer creation callback failed (%d)", ret);
        }
    }
}


Buffer::~Buffer()
{
    if (mBufferDeletionCb)
    {
        int ret = mBufferDeletionCb(this);
        if (ret != 0)
        {
            ULOGE("Buffer: buffer deletion callback failed (%d)", ret);
        }
    }

    if (mAlloc)
    {
        free(mPtr);
        mPtr = NULL;
    }

    free(mMetadataPtr);
    mMetadataPtr = NULL;
}


void Buffer::ref()
{
    mRefCount++;
}


void Buffer::unref()
{
    mRefCount--;
    if ((mRefCount == 0) && (mBufferPool))
    {
        mBufferPool->putBuffer(this);
    }
}


void *Buffer::getPtr()
{
    return mPtr;
}


void Buffer::setPtr(void *ptr)
{
    mPtr = ptr;
}


void *Buffer::getResPtr()
{
    return mResPtr;
}


void Buffer::setResPtr(void *resPtr)
{
    mResPtr = resPtr;
}


unsigned int Buffer::getCapacity()
{
    return mCapacity;
}


void Buffer::setCapacity(unsigned int capacity)
{
    mCapacity = capacity;
}


unsigned int Buffer::getSize()
{
    return mSize;
}


void Buffer::setSize(unsigned int size)
{
    mSize = size;
}


unsigned int Buffer::getId()
{
    return mId;
}


void *Buffer::getMetadataPtr()
{
    return mMetadataPtr;
}


unsigned int Buffer::getMetadataSize()
{
    return mMetadataSize;
}


void *Buffer::getUserPtr()
{
    return mUserPtr;
}


unsigned int Buffer::getRefCount()
{
    return mRefCount;
}


void Buffer::setRefCount(unsigned int refCount)
{
    mRefCount = refCount;
}


BufferPool::BufferPool(unsigned int bufferCount,
                       unsigned int bufferSize,
                       unsigned int bufferMetadataSize,
                       int(*bufferCreationCb)(Buffer *buffer),
                       int(*bufferDeletionCb)(Buffer *buffer))
{
    unsigned int i;
    int ret;

    mBufferCount = bufferCount;
    mBufferSize = bufferSize;

    mBuffers = new std::vector<Buffer*>(bufferCount);
    
    /* Allocate all buffers */
    for (i = 0; i < mBuffers->size(); i++)
    {
        (*mBuffers)[i] = new Buffer(this, i, NULL, NULL, bufferSize,
                                    bufferMetadataSize, bufferCreationCb, bufferDeletionCb); //TODO
    }

    mPool = new std::queue<Buffer*>();

    /* Add all buffers in the pool */
    for (i = 0; i < mBuffers->size(); i++)
    {
        mPool->push(((*mBuffers)[i]));
    }

    ret = pthread_mutex_init(&mMutex, NULL);
    if (ret != 0)
    {
        ULOGE("BufferPool: mutex creation failed (%d)", ret);
    }
    ret = pthread_cond_init(&mCond, NULL);
    if (ret != 0)
    {
        ULOGE("BufferPool: cond creation failed (%d)", ret);
    }
}


BufferPool::~BufferPool()
{
    unsigned int i, cnt;

    pthread_mutex_destroy(&mMutex);
    pthread_cond_destroy(&mCond);

    if (mPool->size() != mBufferCount)
    {
        ULOGE("BufferPool: buffer count mismatch - not all buffers have been returned (%zu vs. %d)", mPool->size(), mBufferCount);
    }

    for (i = 0; i < mBuffers->size(); i++)
    {
        if ((cnt = ((*mBuffers)[i])->getRefCount()) != 0)
        {
            ULOGE("BufferPool: buffer ref count is not null (%d)", cnt);
        }
        delete (*mBuffers)[i];
    }

    delete mBuffers;
    delete mPool;
}


void BufferPool::signal()
{
    pthread_cond_signal(&mCond);
}


bool BufferPool::bufferIsValid(Buffer *buffer)
{
    /* Find the buffer */
    unsigned int i;
    for (i = 0; i < mBuffers->size(); i++)
    {
        if (buffer == (*mBuffers)[i])
        {
            return true;
        }
    }
    return false;
}


Buffer *BufferPool::getBuffer(bool blocking)
{
    Buffer *buffer = NULL;

    pthread_mutex_lock(&mMutex);

    if (mPool->size() == 0)
    {
        if (blocking)
        {
            /* The pool is empty, wait for a new buffer */
            pthread_cond_wait(&mCond, &mMutex);
        }
        else
        {
            pthread_mutex_unlock(&mMutex);
            return NULL;
        }
    }

    if (mPool->size() != 0)
    {
        buffer = mPool->front();
        mPool->pop();
    }
    else
    {
        pthread_mutex_unlock(&mMutex);
        return NULL;
    }

    pthread_mutex_unlock(&mMutex);

    buffer->ref();
    return buffer;
}


void BufferPool::putBuffer(Buffer *buffer)
{
    if (buffer == NULL)
        return;

    pthread_mutex_lock(&mMutex);

    /* Check that this is one of our buffers */
    if (!bufferIsValid(buffer))
    {
        pthread_mutex_unlock(&mMutex);
        ULOGE("BufferPool: invalid buffer");
        return;
    }

    /* Put the buffer in the pool */
    buffer->setRefCount(0);
    mPool->push(buffer);
    if (mPool->size() == 1)
    {
        /* The pool was empty, someone might been waiting for a buffer */
        pthread_cond_signal(&mCond);
    }

    pthread_mutex_unlock(&mMutex);
}


BufferQueue::BufferQueue()
{
    int ret;

    mQueue = new std::queue<Buffer*>();

    ret = pthread_mutex_init(&mMutex, NULL);
    if (ret != 0)
    {
        ULOGE("BufferQueue: mutex creation failed (%d)", ret);
    }
    ret = pthread_cond_init(&mCond, NULL);
    if (ret != 0)
    {
        ULOGE("BufferQueue: cond creation failed (%d)", ret);
    }
}


BufferQueue::~BufferQueue()
{
    flush();

    pthread_mutex_destroy(&mMutex);
    pthread_cond_destroy(&mCond);

    delete mQueue;
}


void BufferQueue::signal()
{
    pthread_cond_signal(&mCond);
}


void BufferQueue::flush()
{
    Buffer *buffer = NULL;

    while ((buffer = popBuffer(false)) != NULL)
    {
        buffer->unref();
    }
}


Buffer *BufferQueue::peekBuffer(bool blocking)
{
    Buffer *buffer = NULL;

    pthread_mutex_lock(&mMutex);

    if (mQueue->size() == 0)
    {
        if (blocking)
        {
            /* The queue is empty, wait for a new buffer */
            pthread_cond_wait(&mCond, &mMutex);
        }
        else
        {
            pthread_mutex_unlock(&mMutex);
            return NULL;
        }
    }

    if (mQueue->size() != 0)
    {
        buffer = mQueue->front();
    }
    else
    {
        pthread_mutex_unlock(&mMutex);
        return NULL;
    }

    pthread_mutex_unlock(&mMutex);

    return buffer;
}


Buffer *BufferQueue::popBuffer(bool blocking)
{
    Buffer *buffer = NULL;

    pthread_mutex_lock(&mMutex);

    if (mQueue->size() == 0)
    {
        if (blocking)
        {
            /* The queue is empty, wait for a new buffer */
            pthread_cond_wait(&mCond, &mMutex);
        }
        else
        {
            pthread_mutex_unlock(&mMutex);
            return NULL;
        }
    }

    if (mQueue->size() != 0)
    {
        buffer = mQueue->front();
        mQueue->pop();
    }
    else
    {
        pthread_mutex_unlock(&mMutex);
        return NULL;
    }

    pthread_mutex_unlock(&mMutex);

    return buffer;
}


void BufferQueue::pushBuffer(Buffer *buffer)
{
    if (buffer == NULL)
        return;

    pthread_mutex_lock(&mMutex);

    /* Put the buffer in the queue */
    mQueue->push(buffer);
    if (mQueue->size() == 1)
    {
        /* The queue was empty, someone might been waiting for a buffer */
        pthread_cond_signal(&mCond);
    }

    pthread_mutex_unlock(&mMutex);
}

}
