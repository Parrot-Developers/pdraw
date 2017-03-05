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


VideoCoreOmxAvcDecoder::VideoCoreOmxAvcDecoder()
{
    mConfigured = false;
    mConfigured2 = false;
    mFirstFrame = true;
    mOutputColorFormat = AVCDECODER_COLORFORMAT_UNKNOWN;
    mBufferMeta = new std::vector<avc_decoder_input_buffer_t>();
    mBufferMetaFreeQueue = new std::queue<avc_decoder_input_buffer_t*>();
    mBufferMetaPushQueue = new std::queue<avc_decoder_input_buffer_t*>();
    mClient = NULL;
    mVideoDecode = NULL;
    //mVideoScheduler = NULL;
    //mClock = NULL;
    mEglRender = NULL;
    mEglBuffer[0] = NULL;
    mEglBuffer[1] = NULL;
    mEglBuffer[2] = NULL;
    mEglImage[0] = NULL;
    mEglImage[1] = NULL;
    mEglImage[2] = NULL;
    mRenderer = NULL;
    mFrameWidth = 0;
    mFrameHeight = 0;
    mSliceHeight = 0;
    mStride = 0;
    mBufferReady = false;
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

    /*if (ilclient_create_component(mClient, &mClock, "clock", (ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS)) != 0)
    {
        ULOGE("videoCoreOmx: ilclient_create_component() failed on clock");
        ilclient_destroy(mClient);
        return;
    }*/

    /*OMX_TIME_CONFIG_CLOCKSTATETYPE cstate;
    memset(&cstate, 0, sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE));
    cstate.nSize = sizeof(OMX_TIME_CONFIG_CLOCKSTATETYPE);
    cstate.nVersion.nVersion = OMX_VERSION;
    cstate.eState = OMX_TIME_ClockStateWaitingForStartTime;
    cstate.nWaitMask = 1;
    if (OMX_SetParameter(ILC_GET_HANDLE(mClock), OMX_IndexConfigTimeClockState, &cstate) != OMX_ErrorNone)
    {
        ULOGE("videoCoreOmx: OMX_SetParameter() failed on clock");
        return;
    }*/

    /*if (ilclient_create_component(mClient, &mVideoScheduler, "video_scheduler", (ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS)) != 0)
    {
        ULOGE("videoCoreOmx: ilclient_create_component() failed on video_scheduler");
        ilclient_destroy(mClient);
        return;
    }*/

    set_tunnel(mTunnel, mVideoDecode, 131, mEglRender, 220);
    /*set_tunnel(mTunnel, mVideoDecode, 131, mVideoScheduler, 10);
    set_tunnel(mTunnel + 1, mVideoScheduler, 11, mEglRender, 220);
    set_tunnel(mTunnel + 2, mClock, 80, mVideoScheduler, 12);*/

    /*if (ilclient_setup_tunnel(mTunnel + 2, 0, 0) != 0)
    {
        ULOGE("videoCoreOmx: ilclient_setup_tunnel() failed");
        return;
    }*/

    /*if (ilclient_change_component_state(mClock, OMX_StateExecuting) != 0)
    {
        ULOGE("videoCoreOmx: failed to change OMX clock component state to 'executing'");
        ilclient_destroy(mClient);
        return;
    }*/

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
                mBufferMeta->resize(def.nBufferCountActual);
                /* add all meta buffers in the free queue */
                unsigned int i;
                for (i = 0; i < def.nBufferCountActual; i++)
                {
                    mBufferMetaFreeQueue->push(&(mBufferMeta->at(i)));
                }
            }
        }
    }

    int ret = pthread_mutex_init(&mMutex, NULL);
    if (ret != 0)
    {
        ULOGE("videoCoreOmx: mutex creation failed (%d)", ret);
    }

    ret = pthread_cond_init(&mCond, NULL);
    if (ret != 0)
    {
        ULOGE("videoCoreOmx: cond creation failed (%d)", ret);
    }

    ULOGI("videoCoreOmx: initialization complete");
}


VideoCoreOmxAvcDecoder::~VideoCoreOmxAvcDecoder()
{
    COMPONENT_T *list[4] = { mVideoDecode, mEglRender }; //, mClock, mVideoScheduler };
    ilclient_flush_tunnels(mTunnel, 0);
    ilclient_disable_port_buffers(mVideoDecode, 130, NULL, NULL, NULL);
    //ilclient_disable_port_buffers(mVideoDecode, 131, NULL, NULL, NULL);
    ilclient_disable_tunnel(mTunnel);
    //ilclient_disable_tunnel(mTunnel + 1);
    //ilclient_disable_tunnel(mTunnel + 2);
    ilclient_teardown_tunnels(mTunnel);
    ilclient_state_transition(list, OMX_StateIdle);
    ilclient_state_transition(list, OMX_StateLoaded);

    ilclient_cleanup_components(list);

    OMX_Deinit();

    ilclient_destroy(mClient);

    pthread_mutex_destroy(&mMutex);
    pthread_cond_destroy(&mCond);

    if (mBufferMeta) delete mBufferMeta;
    if (mBufferMetaFreeQueue) delete mBufferMetaFreeQueue;
    if (mBufferMetaPushQueue) delete mBufferMetaPushQueue;
}


int VideoCoreOmxAvcDecoder::configure(const uint8_t *pSps, unsigned int spsSize, const uint8_t *pPps, unsigned int ppsSize)
{
    int ret = 0;

    if (mConfigured)
    {
        ULOGE("videoCoreOmx: decoder is already configured");
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

        /*if (ret == 0)
        {
            if (ilclient_change_component_state(mVideoScheduler, OMX_StateExecuting) != 0)
            {
                ULOGE("videoCoreOmx: failed to change video_scheduler OMX component state to 'executing'");
                ret = -1;
            }
        }*/

        /*if (ret == 0)
        {
            if (ilclient_setup_tunnel(mTunnel + 1, 0, 0) != 0)
            {
                ULOGE("videoCoreOmx: ilclient_setup_tunnel() failed");
                ret = -1;
            }
        }*/

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

        if (ret == 0)
        {
            mEglImage[0] = ((VideoCoreEglRenderer*)mRenderer)->getVideoEglImage(0);
            if (mEglImage[0] == EGL_NO_IMAGE_KHR)
            {
                ULOGE("videoCoreOmx: failed to get EGL image");
                ret = -1;
            }
        }

        if (ret == 0)
        {
            mEglImage[1] = ((VideoCoreEglRenderer*)mRenderer)->getVideoEglImage(1);
            if (mEglImage[1] == EGL_NO_IMAGE_KHR)
            {
                ULOGE("videoCoreOmx: failed to get EGL image");
                ret = -1;
            }
        }

        if (ret == 0)
        {
            mEglImage[2] = ((VideoCoreEglRenderer*)mRenderer)->getVideoEglImage(2);
            if (mEglImage[2] == EGL_NO_IMAGE_KHR)
            {
                ULOGE("videoCoreOmx: failed to get EGL image");
                ret = -1;
            }
        }

        if (ret == 0)
        {
            if (OMX_UseEGLImage(ILC_GET_HANDLE(mEglRender), &mEglBuffer[0], 221, NULL, mEglImage[0]) != OMX_ErrorNone)
            {
                ULOGE("videoCoreOmx: OMX_UseEGLImage() failed 1");
                ret = -1;
            }
        }

        if (ret == 0)
        {
            if (OMX_UseEGLImage(ILC_GET_HANDLE(mEglRender), &mEglBuffer[1], 221, NULL, mEglImage[1]) != OMX_ErrorNone)
            {
                ULOGE("videoCoreOmx: OMX_UseEGLImage() failed 2");
                ret = -1;
            }
        }

        if (ret == 0)
        {
            if (OMX_UseEGLImage(ILC_GET_HANDLE(mEglRender), &mEglBuffer[2], 221, NULL, mEglImage[2]) != OMX_ErrorNone)
            {
                ULOGE("videoCoreOmx: OMX_UseEGLImage() failed 2");
                ret = -1;
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

        /*if (ilclient_enable_port_buffers(mVideoDecode, 131, NULL, NULL, NULL) != 0)
        {
            ULOGE("videoCoreOmx: ilclient_enable_port_buffers() failed on output");
            return -1;
        }*/

        mConfigured2 = true;
    }

    return ret;
}


int VideoCoreOmxAvcDecoder::getInputBuffer(avc_decoder_input_buffer_t *buffer, bool blocking)
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

    OMX_BUFFERHEADERTYPE *buf = ilclient_get_input_buffer(mVideoDecode, 130, (blocking) ? 1 : 0);
    if (buf != NULL)
    {
        memset(buffer, 0, sizeof(avc_decoder_input_buffer_t));
        buffer->userPtr = (void*)buf;
        buffer->auBuffer = buf->pBuffer;
        buffer->auBufferSize = buf->nAllocLen;
    }
    else
    {
        ULOGE("videoCoreOmx: failed to dequeue an input buffer");
        return -1;
    }

    return 0;
}


int VideoCoreOmxAvcDecoder::queueInputBuffer(avc_decoder_input_buffer_t *buffer)
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

    OMX_BUFFERHEADERTYPE *buf = (OMX_BUFFERHEADERTYPE*)buffer->userPtr;
    buf->nFilledLen = buffer->auSize;
    buf->nOffset = 0;
    buf->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;
    if (mFirstFrame)
    {
        buf->nFlags |= OMX_BUFFERFLAG_STARTTIME;
        mFirstFrame = false;
    }
    buf->nTimeStamp.nHighPart = (uint32_t)(buffer->auNtpTimestampRaw >> 32);
    buf->nTimeStamp.nLowPart = (uint32_t)(buffer->auNtpTimestampRaw & 0xFFFFFFFF);

    pthread_mutex_lock(&mMutex);

    if (mBufferMetaFreeQueue->size() != 0)
    {
        avc_decoder_input_buffer_t *internalMeta = mBufferMetaFreeQueue->front();
        mBufferMetaFreeQueue->pop();
        memcpy(internalMeta, buffer, sizeof(avc_decoder_input_buffer_t));
        mBufferMetaPushQueue->push(internalMeta);
    }
    else
    {
        ULOGW("videoCoreOmx: failed to dequeue a free meta buffer");
    }

    pthread_mutex_unlock(&mMutex);

    if (OMX_EmptyThisBuffer(ILC_GET_HANDLE(mVideoDecode), buf) != OMX_ErrorNone)
    {
        ULOGE("videoCoreOmx: failed to release input buffer");
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


int VideoCoreOmxAvcDecoder::dequeueOutputBuffer(avc_decoder_output_buffer_t *buffer, bool blocking)
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

    if (!mConfigured2)
    {
        ULOGW("videoCoreOmx: output params not known");
        return -1;
    }

    pthread_mutex_lock(&mMutex);
    if (!mBufferReady)
    {
        if (blocking)
        {
            pthread_cond_wait(&mCond, &mMutex);
        }
        else
        {
            pthread_mutex_unlock(&mMutex);
            return -2;
        }
    }
    memcpy(buffer, &mOutputBuffer, sizeof(avc_decoder_output_buffer_t));
    mBufferReady = false;
    pthread_mutex_unlock(&mMutex);

    return 0;
}


int VideoCoreOmxAvcDecoder::releaseOutputBuffer(avc_decoder_output_buffer_t *buffer)
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

    return 0;
}


void VideoCoreOmxAvcDecoder::fillBufferDoneCallback(void *data, COMPONENT_T *comp, OMX_BUFFERHEADERTYPE *buf)
{
    VideoCoreOmxAvcDecoder *decoder = (VideoCoreOmxAvcDecoder*)data;
    avc_decoder_input_buffer_t *internalMeta = NULL;
    uint64_t ts = ((uint64_t)buf->nTimeStamp.nHighPart << 32) | ((uint64_t)buf->nTimeStamp.nLowPart & 0xFFFFFFFF);

    pthread_mutex_lock(&decoder->mMutex);

    if ((ts > 0) && (decoder->mBufferMetaPushQueue->size() != 0))
    {
        while (decoder->mBufferMetaPushQueue->size() != 0)
        {
            avc_decoder_input_buffer_t *b = decoder->mBufferMetaPushQueue->front();

            if (ts > b->auNtpTimestampRaw)
            {
                decoder->mBufferMetaPushQueue->pop();
                decoder->mBufferMetaFreeQueue->push(b);
            }
            else if (ts == b->auNtpTimestampRaw)
            {
                decoder->mBufferMetaPushQueue->pop();
                internalMeta = b;
                break;
            }
            else
            {
                break;
            }
        }
        if (internalMeta == NULL)
        {
            ULOGW("videoCoreOmx: failed to find metadata for TS %llu", ts);
        }
    }
    else
    {
        ULOGW("videoCoreOmx: failed to dequeue a queued meta buffer");
    }

    memset(&decoder->mOutputBuffer, 0, sizeof(avc_decoder_output_buffer_t));
    struct timespec t1;
    clock_gettime(CLOCK_MONOTONIC, &t1);
    decoder->mOutputBuffer.plane[0] = (uint8_t*)decoder->mCurrentEglImageIndex;
    decoder->mOutputBuffer.decoderOutputTimestamp = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
    decoder->mOutputBuffer.width = decoder->mFrameWidth;
    decoder->mOutputBuffer.height = decoder->mFrameHeight;
    decoder->mOutputBuffer.sarWidth = 1; //TODO
    decoder->mOutputBuffer.sarHeight = 1; //TODO
    decoder->mOutputBuffer.stride[0] = decoder->mStride;
    decoder->mOutputBuffer.stride[1] = decoder->mStride / 2;
    decoder->mOutputBuffer.stride[2] = decoder->mStride / 2;
    decoder->mOutputBuffer.colorFormat = decoder->mOutputColorFormat;
    if (internalMeta)
    {
        decoder->mOutputBuffer.isComplete = internalMeta->isComplete;
        decoder->mOutputBuffer.hasErrors = internalMeta->hasErrors;
        decoder->mOutputBuffer.isRef = internalMeta->isRef;
        decoder->mOutputBuffer.auNtpTimestamp = internalMeta->auNtpTimestamp;
        decoder->mOutputBuffer.auNtpTimestampRaw = internalMeta->auNtpTimestampRaw;
        decoder->mOutputBuffer.auNtpTimestampLocal = internalMeta->auNtpTimestampLocal;
        decoder->mOutputBuffer.demuxOutputTimestamp = internalMeta->demuxOutputTimestamp;

        if (internalMeta->hasMetadata)
        {
            memcpy(&decoder->mOutputBuffer.metadata, &internalMeta->metadata, sizeof(frame_metadata_t));
            decoder->mOutputBuffer.hasMetadata = true;
        }
        else
        {
            decoder->mOutputBuffer.hasMetadata = false;
        }
        decoder->mBufferMetaFreeQueue->push(internalMeta);
    }

    decoder->mBufferReady = true;
    pthread_cond_signal(&decoder->mCond);
    pthread_mutex_unlock(&decoder->mMutex);

    decoder->mCurrentEglImageIndex = ((VideoCoreEglRenderer*)decoder->mRenderer)->swapDecoderEglImage();
    if (OMX_FillThisBuffer(ILC_GET_HANDLE(decoder->mEglRender), decoder->mEglBuffer[decoder->mCurrentEglImageIndex]) != OMX_ErrorNone)
    {
        ULOGE("videoCoreOmx: OMX_FillThisBuffer() failed");
    }
}

}

#endif /* USE_VIDEOCOREOMX */
