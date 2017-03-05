/**
 * @file pdraw_bufferqueue.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - buffer queue
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

#ifndef _PDRAW_BUFFERQUEUE_HPP_
#define _PDRAW_BUFFERQUEUE_HPP_

#include <queue>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "pdraw_avcdecoder.hpp"


namespace Pdraw
{


template <class T>
class BufferQueueBuffer
{
public:

    BufferQueueBuffer(void *userPtr, unsigned int size = 0);
    ~BufferQueueBuffer();
    uint8_t* getPtr();
    unsigned int getCapacity();
    void *getUserPtr();
    void getData(T *data);
    void setData(const T *data);

private:

    vector<uint8_t> *mBuffer;
    unsigned int mCapacity;
    void *mUserPtr;
    T mData;
};


typedef enum
{
    BUFFERQUEUE_INVALID_ARGUMENTS = -5,
    BUFFERQUEUE_NO_OUTPUT_BUFFER_AVAILABLE = -4,
    BUFFERQUEUE_NO_INPUT_BUFFER_AVAILABLE = -3,
    BUFFERQUEUE_INVALID_BUFFER = -2,
    BUFFERQUEUE_FAILED = -1,
    BUFFERQUEUE_OK = 0,

} buffer_queue_status_t;


template <class T>
class BufferQueue
{
public:

    BufferQueue(unsigned int nbElement, unsigned int elementInitialSize = 0);
    ~BufferQueue();
    buffer_queue_status_t getInputBuffer(BufferQueueBuffer<T> **buffer, bool blocking);
    buffer_queue_status_t releaseInputBuffer(BufferQueueBuffer<T> *buffer);
    buffer_queue_status_t cancelInputBuffer(BufferQueueBuffer<T> *buffer);
    buffer_queue_status_t getOutputBuffer(BufferQueueBuffer<T> **buffer, bool blocking);
    buffer_queue_status_t releaseOutputBuffer(BufferQueueBuffer<T> *buffer);
    buffer_queue_status_t getBufferFromUserPtr(void *userPtr, BufferQueueBuffer<T> **buffer);

private:

    template <class T2>
    class BufferQueueBufferInternal : public BufferQueueBuffer<T2>
    {
    public:

        typedef enum
        {
            BUFFER_NOT_VALID = -1,
            BUFFER_FREE = 0,
            BUFFER_INPUT_LOCK = 1,
            BUFFER_OUTPUT_LOCK = 2

        } buffer_status_t;

        buffer_status_t mStatus;
        BufferQueueBufferInternal(void *userPtr, unsigned int size = 0);
        ~BufferQueueBufferInternal();
    };

    bool bufferIsValid(BufferQueueBufferInternal<T> *buffer);

    friend void dummyFunction();

    vector<BufferQueueBufferInternal<T>*> *mBuffers;
    queue<BufferQueueBufferInternal<T>*> *mInputQueue;
    pthread_mutex_t mInputMutex;
    pthread_cond_t mInputCond;
    queue<BufferQueueBufferInternal<T>*> *mOutputQueue;
    pthread_mutex_t mOutputMutex;
    pthread_cond_t mOutputCond;
};

}

#endif /* !_PDRAW_BUFFERQUEUE_HPP_ */
