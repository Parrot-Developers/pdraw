/**
 * @file pdraw_bufferqueue.cpp
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

#include "pdraw_bufferqueue.hpp"
#include <math.h>

#define ULOG_TAG libpdraw
#include <ulog.h>


namespace Pdraw
{


template <class T>
BufferQueueBuffer<T>::BufferQueueBuffer(void *userPtr, unsigned int size)
{
    if (size > 0)
        mBuffer = new vector<uint8_t>(size);

    mUserPtr = userPtr;
    mCapacity = size;
}


template <class T>
BufferQueueBuffer<T>::~BufferQueueBuffer()
{
    delete mBuffer;
}


template <class T>
uint8_t* BufferQueueBuffer<T>::getPtr()
{
    if (mBuffer)
        return &((*mBuffer)[0]);
    else
        return NULL;
}


template <class T>
unsigned int BufferQueueBuffer<T>::getCapacity()
{
    return mCapacity;
}


template <class T>
void *BufferQueueBuffer<T>::getUserPtr()
{
    return mUserPtr;
}


template <class T>
void BufferQueueBuffer<T>::getData(T *data)
{
    if (data)
        memcpy((void*)data, (void*)&mData, sizeof(T));
}


template <class T>
void BufferQueueBuffer<T>::setData(const T *data)
{
    if (data)
        memcpy((void*)&mData, (const void*)data, sizeof(T));
}


template <class T>
template <class T2>
BufferQueue<T>::BufferQueueBufferInternal<T2>::BufferQueueBufferInternal(void *userPtr, unsigned int size) :
    BufferQueueBuffer<T2>(userPtr, size)
{
    mStatus = BUFFER_FREE;
}


template <class T>
template <class T2>
BufferQueue<T>::BufferQueueBufferInternal<T2>::~BufferQueueBufferInternal()
{
}


template <class T>
BufferQueue<T>::BufferQueue(unsigned int nbElement, unsigned int elementInitialSize)
{
    int ret;

    mBuffers = new vector<BufferQueueBufferInternal<T>*>(nbElement);

    /* Allocate all buffers */
    unsigned int i;
    uint8_t *usr;
    for (i = 0, usr = NULL; i < mBuffers->size(); i++, usr++)
    {
        (*mBuffers)[i] = new BufferQueueBufferInternal<T>((void*)usr, elementInitialSize);
    }

    mInputQueue = new queue<BufferQueueBufferInternal<T>*>;
    mOutputQueue = new queue<BufferQueueBufferInternal<T>*>;

    /* Add all buffers in the input queue*/
    for (i = 0; i < mBuffers->size(); i++)
    {
        mInputQueue->push(((*mBuffers)[i]));
    }

    ret = pthread_mutex_init(&mInputMutex, NULL);
    if (ret != 0)
    {
        ULOGE("Mutex creation failed (%d)", ret);
    }
    ret = pthread_cond_init(&mInputCond, NULL);
    if (ret != 0)
    {
        ULOGE("Cond creation failed (%d)", ret);
    }
    ret = pthread_mutex_init(&mOutputMutex, NULL);
    if (ret != 0)
    {
        ULOGE("Mutex creation failed (%d)", ret);
    }
    ret = pthread_cond_init(&mOutputCond, NULL);
    if (ret != 0)
    {
        ULOGE("Cond creation failed (%d)", ret);
    }
}


template <class T>
BufferQueue<T>::~BufferQueue()
{
    unsigned int i;

    pthread_mutex_destroy(&mInputMutex);
    pthread_cond_destroy(&mInputCond);
    pthread_mutex_destroy(&mOutputMutex);
    pthread_cond_destroy(&mOutputCond);

    for (i = 0; i < mBuffers->size(); i++)
    {
        delete (*mBuffers)[i];
    }

    delete mBuffers;
    delete mInputQueue;
    delete mOutputQueue;
}


template <class T>
bool BufferQueue<T>::bufferIsValid(BufferQueueBufferInternal<T>* buffer)
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


template <class T>
buffer_queue_status_t BufferQueue<T>::getInputBuffer(BufferQueueBuffer<T> **buffer, bool blocking)
{
    BufferQueueBufferInternal<T> *lockedBuffer = NULL;

    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    pthread_mutex_lock(&mInputMutex);

    if (mInputQueue->size() == 0)
    {
        if (blocking)
        {
            /* Queue empty, wait for a new buffer */
            pthread_cond_wait(&mInputCond, &mInputMutex);
        }
        else
        {
            pthread_mutex_unlock(&mInputMutex);
            return BUFFERQUEUE_NO_INPUT_BUFFER_AVAILABLE;
        }
    }
    if (mInputQueue->size() != 0)
    {
        lockedBuffer = mInputQueue->front();
        mInputQueue->pop();
        lockedBuffer->mStatus = BufferQueueBufferInternal<T>::BUFFER_INPUT_LOCK;
    }
    else
    {
        pthread_mutex_unlock(&mInputMutex);
        return BUFFERQUEUE_NO_INPUT_BUFFER_AVAILABLE;
    }

    pthread_mutex_unlock(&mInputMutex);

    *buffer = (BufferQueueBuffer<T>*)lockedBuffer;
    return BUFFERQUEUE_OK;
}


template <class T>
buffer_queue_status_t BufferQueue<T>::releaseInputBuffer(BufferQueueBuffer<T> *buffer)
{
    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    /* Check this is one of our buffer and status is correct*/
    BufferQueueBufferInternal<T> *bufferInternal = (BufferQueueBufferInternal<T>*)buffer;
    if ((!bufferIsValid(bufferInternal)) || (bufferInternal->mStatus != BufferQueueBufferInternal<T>::BUFFER_INPUT_LOCK))
        return BUFFERQUEUE_INVALID_BUFFER;

    pthread_mutex_lock(&mOutputMutex);

    /* Put input buffer in the ouput queue */
    bufferInternal->mStatus = BufferQueueBufferInternal<T>::BUFFER_FREE;
    mOutputQueue->push(bufferInternal);
    if (mOutputQueue->size() == 1)
    {
        /* The ouput queue was empty, someone might been waiting for a buffer */
        pthread_cond_signal(&mOutputCond);
    }

    pthread_mutex_unlock(&mOutputMutex);

    return BUFFERQUEUE_OK;
}


template <class T>
buffer_queue_status_t BufferQueue<T>::cancelInputBuffer(BufferQueueBuffer<T> *buffer)
{
    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    /* Check this is one of our buffer and status is correct*/
    BufferQueueBufferInternal<T> *bufferInternal = (BufferQueueBufferInternal<T>*)buffer;
    if ((!bufferIsValid(bufferInternal)) || (bufferInternal->mStatus != BufferQueueBufferInternal<T>::BUFFER_INPUT_LOCK))
        return BUFFERQUEUE_INVALID_BUFFER;

    pthread_mutex_lock(&mInputMutex);

    /* Leave the input buffer in the input queue */
    bufferInternal->mStatus = BufferQueueBufferInternal<T>::BUFFER_FREE;
    mInputQueue->push(bufferInternal);
    if (mInputQueue->size() == 1)
    {
        /* The ouput queue was empty, someone might been waiting for a buffer */
        pthread_cond_signal(&mInputCond);
    }

    pthread_mutex_unlock(&mInputMutex);

    return BUFFERQUEUE_OK;
}


template <class T>
buffer_queue_status_t BufferQueue<T>::getOutputBuffer(BufferQueueBuffer<T> **buffer, bool blocking)
{
    BufferQueueBufferInternal<T> *lockedBuffer=NULL;

    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    pthread_mutex_lock(&mOutputMutex);

    if (mOutputQueue->size() == 0)
    {
        if (blocking)
        {
            /* Queue empty, wait for a new buffer */
            pthread_cond_wait(&mOutputCond, &mOutputMutex);
        }
        else
        {
            pthread_mutex_unlock(&mOutputMutex);
            return BUFFERQUEUE_NO_OUTPUT_BUFFER_AVAILABLE;
        }
    }
    if (mOutputQueue->size() != 0)
    {
        lockedBuffer = mOutputQueue->front();
        mOutputQueue->pop();
        lockedBuffer->mStatus = BufferQueueBufferInternal<T>::BUFFER_OUTPUT_LOCK;
    }
    else
    {
        pthread_mutex_unlock(&mOutputMutex);
        return BUFFERQUEUE_NO_OUTPUT_BUFFER_AVAILABLE;
    }

    pthread_mutex_unlock(&mOutputMutex);

    *buffer = (BufferQueueBuffer<T>*)lockedBuffer;
    return BUFFERQUEUE_OK;
}


template <class T>
buffer_queue_status_t BufferQueue<T>::releaseOutputBuffer(BufferQueueBuffer<T> *buffer)
{
    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    /* Check this is one of our buffer and status is correct*/
    BufferQueueBufferInternal<T> *bufferInternal = (BufferQueueBufferInternal<T>*)buffer;
    if ((!bufferIsValid(bufferInternal)) || (bufferInternal->mStatus != BufferQueueBufferInternal<T>::BUFFER_OUTPUT_LOCK))
        return BUFFERQUEUE_INVALID_BUFFER;

    pthread_mutex_lock(&mInputMutex);

    /* Put output buffer in the input queue */
    bufferInternal->mStatus = BufferQueueBufferInternal<T>::BUFFER_FREE;
    mInputQueue->push(bufferInternal);
    if (mInputQueue->size() == 1)
    {
        /* The ouput queue was empty, someone might been waiting for a buffer */
        pthread_cond_signal(&mInputCond);
    }

    pthread_mutex_unlock(&mInputMutex);

    return BUFFERQUEUE_OK;
}


template <class T>
buffer_queue_status_t BufferQueue<T>::getBufferFromUserPtr(void *userPtr, BufferQueueBuffer<T> **buffer)
{
    if (buffer == NULL)
        return BUFFERQUEUE_INVALID_ARGUMENTS;

    /* Find the buffer */
    unsigned int i;
    for (i = 0; i < mBuffers->size(); i++)
    {
        if (userPtr == (*mBuffers)[i]->getUserPtr())
        {
            *buffer = (*mBuffers)[i];
            return BUFFERQUEUE_OK;
        }
    }
    return BUFFERQUEUE_INVALID_ARGUMENTS;
}


/* Explicit declarations to avoid linker issues */
template uint8_t* BufferQueueBuffer<avc_decoder_input_buffer_t>::getPtr();
template unsigned int BufferQueueBuffer<avc_decoder_input_buffer_t>::getCapacity();
template void *BufferQueueBuffer<avc_decoder_input_buffer_t>::getUserPtr();
template void BufferQueueBuffer<avc_decoder_input_buffer_t>::getData(avc_decoder_input_buffer_t*);
template void BufferQueueBuffer<avc_decoder_input_buffer_t>::setData(avc_decoder_input_buffer_t const*);
template uint8_t* BufferQueueBuffer<avc_decoder_output_buffer_t>::getPtr();
template unsigned int BufferQueueBuffer<avc_decoder_output_buffer_t>::getCapacity();
template void *BufferQueueBuffer<avc_decoder_output_buffer_t>::getUserPtr();
template void BufferQueueBuffer<avc_decoder_output_buffer_t>::getData(avc_decoder_output_buffer_t*);
template void BufferQueueBuffer<avc_decoder_output_buffer_t>::setData(avc_decoder_output_buffer_t const*);
template buffer_queue_status_t BufferQueue<avc_decoder_input_buffer_t>::getInputBuffer(BufferQueueBuffer<avc_decoder_input_buffer_t>**, bool);
template buffer_queue_status_t BufferQueue<avc_decoder_input_buffer_t>::cancelInputBuffer(BufferQueueBuffer<avc_decoder_input_buffer_t>*);
template buffer_queue_status_t BufferQueue<avc_decoder_input_buffer_t>::releaseInputBuffer(BufferQueueBuffer<avc_decoder_input_buffer_t>*);
template buffer_queue_status_t BufferQueue<avc_decoder_input_buffer_t>::getOutputBuffer(BufferQueueBuffer<avc_decoder_input_buffer_t>**, bool);
template buffer_queue_status_t BufferQueue<avc_decoder_input_buffer_t>::releaseOutputBuffer(BufferQueueBuffer<avc_decoder_input_buffer_t>*);
template buffer_queue_status_t BufferQueue<avc_decoder_input_buffer_t>::getBufferFromUserPtr(void*, BufferQueueBuffer<avc_decoder_input_buffer_t>**);
template buffer_queue_status_t BufferQueue<avc_decoder_output_buffer_t>::getInputBuffer(BufferQueueBuffer<avc_decoder_output_buffer_t>**, bool);
template buffer_queue_status_t BufferQueue<avc_decoder_output_buffer_t>::cancelInputBuffer(BufferQueueBuffer<avc_decoder_output_buffer_t>*);
template buffer_queue_status_t BufferQueue<avc_decoder_output_buffer_t>::releaseInputBuffer(BufferQueueBuffer<avc_decoder_output_buffer_t>*);
template buffer_queue_status_t BufferQueue<avc_decoder_output_buffer_t>::getOutputBuffer(BufferQueueBuffer<avc_decoder_output_buffer_t>**, bool);
template buffer_queue_status_t BufferQueue<avc_decoder_output_buffer_t>::releaseOutputBuffer(BufferQueueBuffer<avc_decoder_output_buffer_t>*);
template buffer_queue_status_t BufferQueue<avc_decoder_output_buffer_t>::getBufferFromUserPtr(void*, BufferQueueBuffer<avc_decoder_output_buffer_t>**);


void dummyFunction()
{
    BufferQueueBuffer<avc_decoder_input_buffer_t> TempObj(0);
    BufferQueue<avc_decoder_input_buffer_t> TempObj2(0);
    BufferQueue<avc_decoder_input_buffer_t>::BufferQueueBufferInternal<avc_decoder_input_buffer_t> TempObj3(0);

    BufferQueueBuffer<avc_decoder_output_buffer_t> TempObj4(0);
    BufferQueue<avc_decoder_output_buffer_t> TempObj5(0);
    BufferQueue<avc_decoder_output_buffer_t>::BufferQueueBufferInternal<avc_decoder_output_buffer_t> TempObj6(0);
}

}
