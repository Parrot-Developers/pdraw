/**
 * @file pdraw_avcdecoder_videocoreomx.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - Broadcom VideoCore 4 OMX H.264/AVC video decoder
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

#include "pdraw_avcdecoder_videocoreomx.hpp"

#ifdef USE_VIDEOCOREOMX

#include <unistd.h>

#define ULOG_TAG libpdraw
#include <ulog.h>

#include "pdraw_renderer_videocoreegl.hpp"


namespace Pdraw
{


VideoCoreOmxAvcDecoder::VideoCoreOmxAvcDecoder(VideoMedia *media)
{
    mConfigured = false;
    mConfigured2 = false;
    mFirstFrame = true;
    mOutputColorFormat = AVCDECODER_COLORFORMAT_UNKNOWN;
    mMedia = (Media*)media;
    mInputBufferPool = NULL;
    mInputBufferQueue = NULL;
    mOutputBufferPool = NULL;
    mClient = NULL;
    mVideoDecode = NULL;
    mEglRender = NULL;
    int i;
    for (i = 0; i < VIDEOCORE_OMX_AVC_DECODER_OUTPUT_BUFFER_COUNT; i++)
    {
        mEglBuffer[i] = NULL;
        mEglImage[i] = NULL;
    }
    mRenderer = NULL;
    mFrameWidth = 0;
    mFrameHeight = 0;
    mSliceHeight = 0;
    mStride = 0;
    memset(&mTunnel, 0, sizeof(mTunnel));

    if ((mClient = ilclient_init()) == NULL)
    {
        ULOGE("videoCoreOmx: ilclient_init() failed");
        return;
    }

    if (OMX_Init() != OMX_ErrorNone)
    {
        ULOGE("videoCoreOmx: OMX_Init() failed");
        ilclient_destroy(mClient);
        return;
    }

    ilclient_set_fill_buffer_done_callback(mClient, fillBufferDoneCallback, this);

    if (ilclient_create_component(mClient, &mVideoDecode, (char*)"video_decode",
                                  (ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_INPUT_BUFFERS)) != 0)
    {
        ULOGE("videoCoreOmx: ilclient_create_component() failed on video_decode");
        ilclient_destroy(mClient);
        return;
    }

    if (ilclient_create_component(mClient, &mEglRender, (char*)"egl_render",
                                  (ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS | ILCLIENT_ENABLE_OUTPUT_BUFFERS)) != 0)
    {
        ULOGE("videoCoreOmx: ilclient_create_component() failed on egl_render");
        ilclient_destroy(mClient);
        return;
    }

    set_tunnel(mTunnel, mVideoDecode, 131, mEglRender, 220);

    if (ilclient_change_component_state(mVideoDecode, OMX_StateIdle) != 0)
    {
        ULOGE("videoCoreOmx: failed to change OMX component state to 'idle'");
        ilclient_destroy(mClient);
        return;
    }

    OMX_VIDEO_PARAM_PORTFORMATTYPE format;
    memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
    format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
    format.nVersion.nVersion = OMX_VERSION;
    format.nPortIndex = 130;
    format.eCompressionFormat = OMX_VIDEO_CodingAVC;
    if (OMX_SetParameter(ILC_GET_HANDLE(mVideoDecode), OMX_IndexParamVideoPortFormat, &format) != OMX_ErrorNone)
    {
        ULOGE("videoCoreOmx: OMX_SetParameter() failed");
        return;
    }

    OMX_NALSTREAMFORMATTYPE nal;
    memset(&nal, 0, sizeof(OMX_NALSTREAMFORMATTYPE));
    nal.nSize = sizeof(OMX_NALSTREAMFORMATTYPE);
    nal.nVersion.nVersion = OMX_VERSION;
    nal.nPortIndex = 130;
    nal.eNaluFormat = OMX_NaluFormatStartCodes;
    if (OMX_SetParameter(ILC_GET_HANDLE(mVideoDecode), (OMX_INDEXTYPE)OMX_IndexParamNalStreamFormatSelect, &nal) != OMX_ErrorNone)
    {
        ULOGE("videoCoreOmx: failed to set NAL stream format");
        return;
    }

    OMX_PARAM_PORTDEFINITIONTYPE def;
    memset(&def, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
    def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    def.nVersion.nVersion = OMX_VERSION;
    def.nPortIndex = 130;
    if (OMX_GetParameter(ILC_GET_HANDLE(mVideoDecode), OMX_IndexParamPortDefinition, &def) != OMX_ErrorNone)
    {
        ULOGE("videoCoreOmx: failed to get input port definition");
        return;
    }
    else
    {
        ULOGI("videoCoreOmx: nBufferCountActual=%d, nBufferCountMin=%d, nBufferSize=%d",
              def.nBufferCountActual, def.nBufferCountMin, def.nBufferSize);
        ULOGI("videoCoreOmx: width=%d, height=%d, sliceHeight=%d, stride=%d, colorFormat=%d",
              def.format.video.nFrameWidth, def.format.video.nFrameHeight, def.format.video.nSliceHeight,
              def.format.video.nStride, def.format.video.eColorFormat);
        def.format.video.nFrameWidth = 640;
        def.format.video.nFrameHeight = 480;
        def.nBufferSize = 1024 * 1024;
        def.nBufferCountActual = 20;
        if (OMX_SetParameter(ILC_GET_HANDLE(mVideoDecode), OMX_IndexParamPortDefinition, &def) != OMX_ErrorNone)
        {
            ULOGE("videoCoreOmx: failed to set input port definition");
            return;
        }
        else
        {
            if (OMX_GetParameter(ILC_GET_HANDLE(mVideoDecode), OMX_IndexParamPortDefinition, &def) != OMX_ErrorNone)
            {
                ULOGE("videoCoreOmx: failed to get input port definition");
                return;
            }
            else
            {
                ULOGI("videoCoreOmx: nBufferCountActual=%d, nBufferCountMin=%d, nBufferSize=%d",
                      def.nBufferCountActual, def.nBufferCountMin, def.nBufferSize);
                ULOGI("videoCoreOmx: width=%d, height=%d, sliceHeight=%d, stride=%d, colorFormat=%d",
                      def.format.video.nFrameWidth, def.format.video.nFrameHeight, def.format.video.nSliceHeight,
                      def.format.video.nStride, def.format.video.eColorFormat);

                /* Input buffers pool allocation */
                mInputBufferPool = new BufferPool(def.nBufferCountActual, 0,
                                                  sizeof(avc_decoder_input_buffer_t), 0,
                                                  NULL, NULL);
                if (mInputBufferPool == NULL)
                {
                    ULOGE("videoCoreOmx: failed to allocate decoder input buffers pool");
                }

                /* Input buffers queue allocation */
                mInputBufferQueue = new BufferQueue();
                if (mInputBufferQueue == NULL)
                {
                    ULOGE("videoCoreOmx: failed to allocate decoder input buffers queue");
                }
            }
        }
    }

    /* Output buffers pool allocation */
    mOutputBufferPool = new BufferPool(VIDEOCORE_OMX_AVC_DECODER_OUTPUT_BUFFER_COUNT, 0,
                                       sizeof(avc_decoder_output_buffer_t), 0, NULL, NULL);
    if (mOutputBufferPool == NULL)
    {
        ULOGE("videoCoreOmx: failed to allocate decoder output buffers pool");
    }

    ULOGI("videoCoreOmx: initialization complete");
}


VideoCoreOmxAvcDecoder::~VideoCoreOmxAvcDecoder()
{
    COMPONENT_T *list[4] = { mVideoDecode, mEglRender }; //, mClock, mVideoScheduler };
    ilclient_flush_tunnels(mTunnel, 0);
    ilclient_disable_port_buffers(mVideoDecode, 130, NULL, NULL, NULL);
    ilclient_disable_tunnel(mTunnel);
    ilclient_teardown_tunnels(mTunnel);
    ilclient_state_transition(list, OMX_StateIdle);

    ilclient_cleanup_components(list);

    OMX_Deinit();

    ilclient_destroy(mClient);

    if (mInputBufferQueue) delete mInputBufferQueue;
    if (mInputBufferPool) delete mInputBufferPool;
    if (mOutputBufferPool) delete mOutputBufferPool;

    std::vector<BufferQueue*>::iterator q = mOutputBufferQueues.begin();
    while (q != mOutputBufferQueues.end())
    {
        delete *q;
        q++;
    }
}


int VideoCoreOmxAvcDecoder::configure(const uint8_t *pSps, unsigned int spsSize, const uint8_t *pPps, unsigned int ppsSize)
{
    int ret = 0;

    if (mConfigured)
    {
        ULOGE("videoCoreOmx: decoder is already configured");
        return -1;
    }
    if ((!pSps) || (spsSize == 0) || (!pPps) || (ppsSize == 0))
    {
        ULOGE("videoCoreOmx: invalid SPS/PPS");
        return -1;
    }

    if (ret == 0)
    {
        if (ilclient_enable_port_buffers(mVideoDecode, 130, NULL, NULL, NULL) != 0)
        {
            ULOGE("videoCoreOmx: ilclient_enable_port_buffers() failed on input");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        if (ilclient_change_component_state(mVideoDecode, OMX_StateExecuting) != 0)
        {
            ULOGE("videoCoreOmx: failed to change OMX component state to 'executing'");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        OMX_BUFFERHEADERTYPE *buf = ilclient_get_input_buffer(mVideoDecode, 130, 1);
        if (buf != NULL)
        {
            //TODO: demuxer should always output SPS/PPS with start codes
            if (buf->nAllocLen >= spsSize + 4 + ppsSize + 4)
            {
                uint8_t *sps = buf->pBuffer, *pps = buf->pBuffer + 4 + spsSize;
                sps[0] = sps[1] = sps[2] = 0; sps[3] = 1;
                pps[0] = pps[1] = pps[2] = 0; pps[3] = 1;
                memcpy(sps + 4, pSps, spsSize);
                memcpy(pps + 4, pPps, ppsSize);
                buf->nFilledLen = spsSize + 4 + ppsSize + 4;
                buf->nOffset = 0;
                buf->nFlags = OMX_BUFFERFLAG_CODECCONFIG;
                if (OMX_EmptyThisBuffer(ILC_GET_HANDLE(mVideoDecode), buf) != OMX_ErrorNone)
                {
                    ULOGE("videoCoreOmx: failed to release input buffer");
                    ret = -1;
                }
            }
            else
            {
                ULOGE("videoCoreOmx: buffer too small for SPS/PPS");
                ret = -1;
            }
        }
        else
        {
            ULOGE("videoCoreOmx: failed to dequeue an input buffer");
            ret = -1;
        }
    }

    mConfigured = (ret == 0) ? true : false;

    if (mConfigured)
    {
        ULOGI("videoCoreOmx: decoder is configured");
    }

    return ret;
}


void VideoCoreOmxAvcDecoder::setRenderer(Renderer *renderer)
{
    mRenderer = renderer;
}


int VideoCoreOmxAvcDecoder::portSettingsChanged()
{
    int ret = 0;
    ULOGI("videoCoreOmx: port settings changed event");

    OMX_PARAM_PORTDEFINITIONTYPE def;
    memset(&def, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
    def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    def.nVersion.nVersion = OMX_VERSION;
    def.nPortIndex = 131;
    if (OMX_GetParameter(ILC_GET_HANDLE(mVideoDecode), OMX_IndexParamPortDefinition, &def) != OMX_ErrorNone)
    {
        ULOGE("videoCoreOmx: failed to get output port definition");
        return -1;
    }
    else
    {
        ULOGI("videoCoreOmx: width=%d, height=%d, sliceHeight=%d, stride=%d, colorFormat=%d",
              def.format.video.nFrameWidth, def.format.video.nFrameHeight, def.format.video.nSliceHeight,
              def.format.video.nStride, def.format.video.eColorFormat);
        mFrameWidth = def.format.video.nFrameWidth;
        mFrameHeight = def.format.video.nFrameHeight;
        mSliceHeight = def.format.video.nSliceHeight;
        mStride = def.format.video.nStride;
        switch (def.format.video.eColorFormat)
        {
            case OMX_COLOR_FormatYUV420PackedPlanar:
                mOutputColorFormat = AVCDECODER_COLORFORMAT_YUV420PLANAR;
                break;
            default:
                mOutputColorFormat = AVCDECODER_COLORFORMAT_UNKNOWN;
                break;
        }

        if (ret == 0)
        {
            if (mRenderer == NULL)
            {
                ULOGE("videoCoreOmx: invalid renderer");
                ret = -1;
            }
        }


        if (ret == 0)
        {
            if (ilclient_setup_tunnel(mTunnel, 0, 0) != 0)
            {
                ULOGE("videoCoreOmx: ilclient_setup_tunnel() failed");
                ret = -1;
            }
        }

        if (ret == 0)
        {
            if (ilclient_change_component_state(mEglRender, OMX_StateIdle) != 0)
            {
                ULOGE("videoCoreOmx: failed to change egl_render OMX component state to 'idle'");
                ret = -1;
            }
        }

        if (ret == 0)
        {
            OMX_PARAM_PORTDEFINITIONTYPE def;
            memset(&def, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
            def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
            def.nVersion.nVersion = OMX_VERSION;
            def.nPortIndex = 221;
            if (OMX_GetParameter(ILC_GET_HANDLE(mEglRender), OMX_IndexParamPortDefinition, &def) != OMX_ErrorNone)
            {
                ULOGE("videoCoreOmx: failed to get egl_render input port definition");
                ret = -1;
            }
            else
            {
                ULOGI("videoCoreOmx: nBufferCountActual=%d, nBufferCountMin=%d, nBufferSize=%d",
                      def.nBufferCountActual, def.nBufferCountMin, def.nBufferSize);
                ULOGI("videoCoreOmx: width=%d, height=%d, sliceHeight=%d, stride=%d, colorFormat=%d",
                      def.format.video.nFrameWidth, def.format.video.nFrameHeight, def.format.video.nSliceHeight,
                      def.format.video.nStride, def.format.video.eColorFormat);
                def.nBufferCountActual = 3;
                if (OMX_SetParameter(ILC_GET_HANDLE(mEglRender), OMX_IndexParamPortDefinition, &def) != OMX_ErrorNone)
                {
                    ULOGE("videoCoreOmx: failed to set egl_render input port definition");
                    ret = -1;
                }
                else
                {
                    if (OMX_GetParameter(ILC_GET_HANDLE(mEglRender), OMX_IndexParamPortDefinition, &def) != OMX_ErrorNone)
                    {
                        ULOGE("videoCoreOmx: failed to get egl_render input port definition");
                        ret = -1;
                    }
                    else
                    {
                        ULOGI("videoCoreOmx: nBufferCountActual=%d, nBufferCountMin=%d, nBufferSize=%d",
                              def.nBufferCountActual, def.nBufferCountMin, def.nBufferSize);
                        ULOGI("videoCoreOmx: width=%d, height=%d, sliceHeight=%d, stride=%d, colorFormat=%d",
                              def.format.video.nFrameWidth, def.format.video.nFrameHeight, def.format.video.nSliceHeight,
                              def.format.video.nStride, def.format.video.eColorFormat);
                    }
                }
            }
        }

        if (ret == 0)
        {
            //ilclient_enable_port(mEglRender, 221); THIS BLOCKS SO CAN'T BE USED
            if (OMX_SendCommand(ILC_GET_HANDLE(mEglRender), OMX_CommandPortEnable, 221, NULL) != OMX_ErrorNone)
            {
                ULOGE("videoCoreOmx: failed to enable output port on egl_render");
                ret = -1;
            }
        }

        if (ret == 0)
        {
            ret = ((VideoCoreEglRenderer*)mRenderer)->setVideoDimensions(mFrameWidth, mFrameHeight);
            if (ret != 0)
            {
                ULOGE("videoCoreOmx: renderer->setVideoDimensions() failed");
            }
        }

        int i;
        for (i = 0; i < VIDEOCORE_OMX_AVC_DECODER_OUTPUT_BUFFER_COUNT; i++)
        {
            if (ret == 0)
            {
                mEglImage[i] = ((VideoCoreEglRenderer*)mRenderer)->getVideoEglImage(i);
                if (mEglImage[i] == EGL_NO_IMAGE_KHR)
                {
                    ULOGE("videoCoreOmx: failed to get EGL image #%d", i);
                    ret = -1;
                }
            }

            if (ret == 0)
            {
                if (OMX_UseEGLImage(ILC_GET_HANDLE(mEglRender), &mEglBuffer[i], 221, NULL, mEglImage[i]) != OMX_ErrorNone)
                {
                    ULOGE("videoCoreOmx: OMX_UseEGLImage() failed on image #%d", i);
                    ret = -1;
                }
            }
        }

        if (ret == 0)
        {
            if (ilclient_change_component_state(mEglRender, OMX_StateExecuting) != 0)
            {
                ULOGE("videoCoreOmx: failed to change egl_render OMX component state to 'executing'");
                ret = -1;
            }
        }

        if (ret == 0)
        {
            mCurrentEglImageIndex = ((VideoCoreEglRenderer*)mRenderer)->swapDecoderEglImage();
            if (OMX_FillThisBuffer(ILC_GET_HANDLE(mEglRender), mEglBuffer[mCurrentEglImageIndex]) != OMX_ErrorNone)
            {
                ULOGE("videoCoreOmx: OMX_FillThisBuffer() failed");
                ret = -1;
            }
        }

        mConfigured2 = true;
    }

    return ret;
}


int VideoCoreOmxAvcDecoder::getInputBuffer(Buffer **buffer, bool blocking)
{
    if (!buffer)
    {
        ULOGE("videoCoreOmx: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("videoCoreOmx: decoder is not configured");
        return -1;
    }

    if ((!mConfigured2) && ((ilclient_remove_event(mVideoDecode, OMX_EventPortSettingsChanged, 131, 0, 0, 1) == 0)
            || (ilclient_wait_for_event(mVideoDecode, OMX_EventPortSettingsChanged, 131, 0, 0, 1,
                ILCLIENT_EVENT_ERROR | ILCLIENT_PARAMETER_CHANGED, 0) == 0)))
    {
        if (portSettingsChanged() != 0)
        {
            ULOGE("videoCoreOmx: portSettingsChanged() failed");
            return -1;
        }
    }

    if (mInputBufferPool)
    {
        Buffer *buf = mInputBufferPool->getBuffer(blocking);
        if (buf != NULL)
        {
            OMX_BUFFERHEADERTYPE *omxBuf = ilclient_get_input_buffer(mVideoDecode, 130, (blocking) ? 1 : 0);
            if (omxBuf == NULL)
            {
                buf->unref();
                ULOGE("videoCoreOmx: failed to dequeue an input buffer");
                return -2;
            }

            buf->setSize(0);
            buf->setCapacity(omxBuf->nAllocLen);
            buf->setPtr(omxBuf->pBuffer);
            buf->setResPtr((void*)omxBuf);
            *buffer = buf;
        }
        else
        {
            ULOGD("videoCoreOmx: failed to get an input buffer");
            return -2;
        }
    }
    else
    {
        ULOGE("videoCoreOmx: input buffer pool has not been created");
        return -1;
    }

    return 0;
}


int VideoCoreOmxAvcDecoder::queueInputBuffer(Buffer *buffer)
{
    if (!buffer)
    {
        ULOGE("videoCoreOmx: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("videoCoreOmx: decoder is not configured");
        return -1;
    }

    if (mInputBufferQueue)
    {
        OMX_BUFFERHEADERTYPE *omxBuf = (OMX_BUFFERHEADERTYPE*)buffer->getResPtr();
        avc_decoder_input_buffer_t *data = (avc_decoder_input_buffer_t*)buffer->getMetadataPtr();

        omxBuf->nFilledLen = buffer->getSize();
        omxBuf->nOffset = 0;
        omxBuf->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;
        if (mFirstFrame)
        {
            omxBuf->nFlags |= OMX_BUFFERFLAG_STARTTIME;
            mFirstFrame = false;
        }

        if (data)
        {
            omxBuf->nTimeStamp.nHighPart = (uint32_t)(data->auNtpTimestampRaw >> 32);
            omxBuf->nTimeStamp.nLowPart = (uint32_t)(data->auNtpTimestampRaw & 0xFFFFFFFF);
        }
        else
        {
            ULOGW("videoCoreOmx: invalid buffer metadata");
            omxBuf->nTimeStamp.nHighPart = 0;
            omxBuf->nTimeStamp.nLowPart = 0;
        }

        buffer->ref();
        mInputBufferQueue->pushBuffer(buffer);

        if (OMX_EmptyThisBuffer(ILC_GET_HANDLE(mVideoDecode), omxBuf) != OMX_ErrorNone)
        {
            ULOGE("videoCoreOmx: failed to release input buffer");
            return -1;
        }
    }
    else
    {
        ULOGE("videoCoreOmx: input queue has not been created");
        return -1;
    }

    if ((!mConfigured2) && ((ilclient_remove_event(mVideoDecode, OMX_EventPortSettingsChanged, 131, 0, 0, 1) == 0)
            || (ilclient_wait_for_event(mVideoDecode, OMX_EventPortSettingsChanged, 131, 0, 0, 1,
                ILCLIENT_EVENT_ERROR | ILCLIENT_PARAMETER_CHANGED, 0) == 0)))
    {
        if (portSettingsChanged() != 0)
        {
            ULOGE("videoCoreOmx: portSettingsChanged() failed");
            return -1;
        }
    }

    return 0;
}


BufferQueue *VideoCoreOmxAvcDecoder::addOutputQueue()
{
    BufferQueue *q = new BufferQueue();
    if (q == NULL)
    {
        ULOGE("videoCoreOmx: queue allocation failed");
        return NULL;
    }

    mOutputBufferQueues.push_back(q);
    return q;
}


int VideoCoreOmxAvcDecoder::removeOutputQueue(BufferQueue *queue)
{
    if (!queue)
    {
        ULOGE("videoCoreOmx: invalid queue pointer");
        return -1;
    }

    bool found = false;
    std::vector<BufferQueue*>::iterator q = mOutputBufferQueues.begin();

    while (q != mOutputBufferQueues.end())
    {
        if (*q == queue)
        {
            mOutputBufferQueues.erase(q);
            delete *q;
            found = true;
            break;
        }
        q++;
    }

    return (found) ? 0 : -1;
}


bool VideoCoreOmxAvcDecoder::isOutputQueueValid(BufferQueue *queue)
{
    if (!queue)
    {
        ULOGE("videoCoreOmx: invalid queue pointer");
        return false;
    }

    bool found = false;
    std::vector<BufferQueue*>::iterator q = mOutputBufferQueues.begin();

    while (q != mOutputBufferQueues.end())
    {
        if (*q == queue)
        {
            found = true;
            break;
        }
        q++;
    }

    return found;
}


int VideoCoreOmxAvcDecoder::dequeueOutputBuffer(BufferQueue *queue, Buffer **buffer, bool blocking)
{
    if (!queue)
    {
        ULOGE("videoCoreOmx: invalid queue pointer");
        return -1;
    }
    if (!buffer)
    {
        ULOGE("videoCoreOmx: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("videoCoreOmx: decoder is not configured");
        return -1;
    }

    if (!mConfigured2)
    {
        ULOGW("videoCoreOmx: output params not known");
        return -1;
    }

    if (isOutputQueueValid(queue))
    {
        Buffer *buf = queue->popBuffer(blocking);
        if (buf != NULL)
        {
            avc_decoder_output_buffer_t *data = (avc_decoder_output_buffer_t*)buf->getMetadataPtr();
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            data->decoderOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
            *buffer = buf;
        }
        else
        {
            ULOGD("videoCoreOmx: failed to dequeue an output buffer");
            return -2;
        }
    }
    else
    {
        ULOGE("videoCoreOmx: invalid output queue");
        return -1;
    }

    return 0;
}


int VideoCoreOmxAvcDecoder::releaseOutputBuffer(Buffer *buffer)
{
    if (!buffer)
    {
        ULOGE("videoCoreOmx: invalid buffer pointer");
        return -1;
    }

    if (!mConfigured)
    {
        ULOGE("videoCoreOmx: decoder is not configured");
        return -1;
    }

    buffer->unref();

    return 0;
}


int VideoCoreOmxAvcDecoder::stop()
{
    if (!mConfigured)
    {
        ULOGE("videoCoreOmx: decoder is not configured");
        return -1;
    }

    //TODO
    mConfigured = false;

    if (mInputBufferPool) mInputBufferPool->signal();
    if (mOutputBufferPool) mOutputBufferPool->signal();
    if (mInputBufferQueue) mInputBufferQueue->signal();

    return 0;
}


void VideoCoreOmxAvcDecoder::fillBufferDoneCallback(void *data, COMPONENT_T *comp, OMX_BUFFERHEADERTYPE *omxBuf)
{
    VideoCoreOmxAvcDecoder *decoder = (VideoCoreOmxAvcDecoder*)data;
    Buffer *inputBuffer = NULL, *outputBuffer = NULL;
    avc_decoder_input_buffer_t *inputData = NULL;
    avc_decoder_output_buffer_t *outputData = NULL;
    uint64_t ts = ((uint64_t)omxBuf->nTimeStamp.nHighPart << 32) | ((uint64_t)omxBuf->nTimeStamp.nLowPart & 0xFFFFFFFF);

    if (!decoder->mConfigured)
    {
        decoder->mInputBufferQueue->flush();
        return;
    }

    if (ts > 0)
    {
        Buffer *b;
        while ((b = decoder->mInputBufferQueue->peekBuffer(false)) != NULL)
        {
            avc_decoder_input_buffer_t *d = (avc_decoder_input_buffer_t*)b->getMetadataPtr();

            if (ts > d->auNtpTimestampRaw)
            {
                b = decoder->mInputBufferQueue->popBuffer(false);
                b->unref();
            }
            else if (ts == d->auNtpTimestampRaw)
            {
                inputBuffer = decoder->mInputBufferQueue->popBuffer(false);
                inputData = d;
                break;
            }
            else
            {
                break;
            }
        }

        if ((inputBuffer == NULL) || (inputData == NULL))
        {
            ULOGW("videoCoreOmx: failed to find buffer for TS %llu", ts);
        }
    }
    else
    {
        ULOGW("videoCoreOmx: invalid timestamp in buffer callback");
    }

    outputBuffer = decoder->mOutputBufferPool->getBuffer(false);
    if (outputBuffer)
    {
        outputData = (avc_decoder_output_buffer_t*)outputBuffer->getMetadataPtr();
        if (outputData)
        {
            outputBuffer->setMetadataSize(sizeof(avc_decoder_output_buffer_t));
            struct timespec t1;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            outputData->plane[0] = (uint8_t*)decoder->mCurrentEglImageIndex;
            outputData->decoderOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
            outputData->width = decoder->mFrameWidth;
            outputData->height = decoder->mFrameHeight;
            outputData->sarWidth = 1; //TODO
            outputData->sarHeight = 1; //TODO
            outputData->stride[0] = decoder->mStride;
            outputData->stride[1] = decoder->mStride / 2;
            outputData->stride[2] = decoder->mStride / 2;
            outputData->colorFormat = decoder->mOutputColorFormat;
            if (inputData)
            {
                outputData->isComplete = inputData->isComplete;
                outputData->hasErrors = inputData->hasErrors;
                outputData->isRef = inputData->isRef;
                outputData->auNtpTimestamp = inputData->auNtpTimestamp;
                outputData->auNtpTimestampRaw = inputData->auNtpTimestampRaw;
                outputData->auNtpTimestampLocal = inputData->auNtpTimestampLocal;
                outputData->demuxOutputTimestamp = inputData->demuxOutputTimestamp;

                if (inputData->hasMetadata)
                {
                    memcpy(&outputData->metadata, &inputData->metadata, sizeof(struct pdraw_video_frame_metadata));
                    outputData->hasMetadata = true;
                }
                else
                {
                    outputData->hasMetadata = false;
                }
            }

            /* User data */
            unsigned int userDataSize = inputBuffer->getUserDataSize();
            void *userData = inputBuffer->getUserDataPtr();
            if ((userData) && (userDataSize > 0))
            {
                int ret = outputBuffer->setUserDataCapacity(userDataSize);
                if (ret < (signed)userDataSize)
                {
                    ULOGE("ffmpeg: failed to realloc user data buffer");
                }
                else
                {
                    void *dstBuf = outputBuffer->getUserDataPtr();
                    memcpy(dstBuf, userData, userDataSize);
                    outputBuffer->setUserDataSize(userDataSize);
                }
            }
            else
            {
                outputBuffer->setUserDataSize(0);
            }

            std::vector<BufferQueue*>::iterator q = decoder->mOutputBufferQueues.begin();
            while (q != decoder->mOutputBufferQueues.end())
            {
                outputBuffer->ref();
                (*q)->pushBuffer(outputBuffer);
                q++;
            }
        }
        else
        {
            ULOGE("videoCoreOmx: invalid output buffer");
        }
        outputBuffer->unref();
    }
    else
    {
        ULOGE("videoCoreOmx: failed to get an output buffer");
    }

    if (inputBuffer)
    {
        inputBuffer->unref();
    }

    decoder->mCurrentEglImageIndex = ((VideoCoreEglRenderer*)decoder->mRenderer)->swapDecoderEglImage();
    if (OMX_FillThisBuffer(ILC_GET_HANDLE(decoder->mEglRender), decoder->mEglBuffer[decoder->mCurrentEglImageIndex]) != OMX_ErrorNone)
    {
        ULOGE("videoCoreOmx: OMX_FillThisBuffer() failed");
    }
}

}

#endif /* USE_VIDEOCOREOMX */
