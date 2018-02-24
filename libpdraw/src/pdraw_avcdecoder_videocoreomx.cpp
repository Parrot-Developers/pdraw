/**
 * Parrot Drones Awesome Video Viewer Library
 * Broadcom VideoCore 4 OMX H.264/AVC video decoder
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "pdraw_avcdecoder_videocoreomx.hpp"

#ifdef USE_VIDEOCOREOMX

#include "pdraw_renderer_videocoreegl.hpp"
#include <unistd.h>
#include <video-buffers/vbuf_ilclient.h>
#define ULOG_TAG libpdraw
#include <ulog.h>
#include <vector>

namespace Pdraw {


VideoCoreOmxAvcDecoder::VideoCoreOmxAvcDecoder(
	VideoMedia *media)
{
	int ret;
	struct vbuf_cbs cbs;
	OMX_ERRORTYPE omxErr;
	OMX_VIDEO_PARAM_PORTFORMATTYPE format;
	OMX_NALSTREAMFORMATTYPE nal;
	OMX_PARAM_PORTDEFINITIONTYPE def;

	mConfigured = false;
	mConfigured2 = false;
	mFirstFrame = true;
	mOutputColorFormat = AVCDECODER_COLOR_FORMAT_UNKNOWN;
	mMedia = (Media*)media;
	mInputBufferPool = NULL;
	mInputBufferQueue = NULL;
	mOutputBufferPool = NULL;
	mClient = NULL;
	mVideoDecode = NULL;
	mEglRender = NULL;
	int i;
	for (i = 0; i < AVCDECODER_VIDEOCOREOMX_OUTPUT_BUFFER_COUNT; i++) {
		mEglBuffer[i] = NULL;
		mEglImage[i] = NULL;
	}
	mRenderer = NULL;
	mFrameWidth = 0;
	mFrameHeight = 0;
	mSliceHeight = 0;
	mStride = 0;
	memset(&mTunnel, 0, sizeof(mTunnel));

	mClient = ilclient_init();
	if (mClient == NULL) {
		ULOGE("VideoCoreOmx: ilclient_init() failed");
		goto err;
	}

	omxErr = OMX_Init();
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: OMX_Init() failed (%d)", omxErr);
		goto err;
	}

	ilclient_set_fill_buffer_done_callback(
		mClient, fillBufferDoneCallback, this);

	ret = ilclient_create_component(mClient,
		&mVideoDecode, (char *)"video_decode",
		(ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS |
			ILCLIENT_ENABLE_INPUT_BUFFERS));
	if (ret != 0) {
		ULOGE("VideoCoreOmx: ilclient_create_component() "
			"failed on video_decode (%d)", ret);
		goto err;
	}

	ret = ilclient_create_component(mClient,
		&mEglRender, (char *)"egl_render",
		(ILCLIENT_CREATE_FLAGS_T)(ILCLIENT_DISABLE_ALL_PORTS |
			ILCLIENT_ENABLE_OUTPUT_BUFFERS));
	if (ret != 0) {
		ULOGE("VideoCoreOmx: ilclient_create_component() "
			"failed on egl_render (%d)", ret);
		goto err;
	}

	set_tunnel(mTunnel, mVideoDecode, 131, mEglRender, 220);

	ret = ilclient_change_component_state(mVideoDecode, OMX_StateIdle);
	if (ret != 0) {
		ULOGE("VideoCoreOmx: failed to change "
			"OMX component state to 'idle' (%d)", ret);
		goto err;
	}

	memset(&format, 0, sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE));
	format.nSize = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);
	format.nVersion.nVersion = OMX_VERSION;
	format.nPortIndex = 130;
	format.eCompressionFormat = OMX_VIDEO_CodingAVC;
	omxErr = OMX_SetParameter(ILC_GET_HANDLE(mVideoDecode),
		OMX_IndexParamVideoPortFormat, &format);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to set input port format (%d)",
			omxErr);
		goto err;
	}

	memset(&nal, 0, sizeof(OMX_NALSTREAMFORMATTYPE));
	nal.nSize = sizeof(OMX_NALSTREAMFORMATTYPE);
	nal.nVersion.nVersion = OMX_VERSION;
	nal.nPortIndex = 130;
	nal.eNaluFormat = OMX_NaluFormatStartCodes;
	omxErr = OMX_SetParameter(ILC_GET_HANDLE(mVideoDecode),
		(OMX_INDEXTYPE)OMX_IndexParamNalStreamFormatSelect, &nal);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to set H.264 bitstream format (%d)",
			omxErr);
		goto err;
	}

	memset(&def, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = 130;
	omxErr = OMX_GetParameter(ILC_GET_HANDLE(mVideoDecode),
		OMX_IndexParamPortDefinition, &def);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to get input port definition (%d)",
			omxErr);
		goto err;
	}

	def.format.video.nFrameWidth = 640;
	def.format.video.nFrameHeight = 480;
	def.nBufferSize = 1024 * 1024;
	def.nBufferCountActual = 20;
	omxErr = OMX_SetParameter(ILC_GET_HANDLE(mVideoDecode),
		OMX_IndexParamPortDefinition, &def);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to set input port definition (%d)",
			omxErr);
		goto err;
	}

	omxErr = OMX_GetParameter(ILC_GET_HANDLE(mVideoDecode),
		OMX_IndexParamPortDefinition, &def);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to get input port definition (%d)",
			omxErr);
		goto err;
	}

	ULOGI("VideoCoreOmx: nBufferCountActual=%d, "
		"nBufferCountMin=%d, nBufferSize=%d",
		def.nBufferCountActual, def.nBufferCountMin, def.nBufferSize);
	ULOGI("VideoCoreOmx: width=%d, height=%d, "
		"sliceHeight=%d, stride=%d, colorFormat=%d",
		def.format.video.nFrameWidth, def.format.video.nFrameHeight,
		def.format.video.nSliceHeight, def.format.video.nStride,
		def.format.video.eColorFormat);

	ret = vbuf_ilclient_get_input_cbs(mVideoDecode, &cbs);
	if (ret != 0) {
		ULOGE("VideoCoreOmx: failed to get "
			"input buffers allocation callbacks");
		goto err;
	}

	/* Input buffers pool allocation */
	mInputBufferPool = vbuf_pool_new(def.nBufferCountActual, 0,
		sizeof(struct avcdecoder_input_buffer), 0, &cbs);
	if (mInputBufferPool == NULL) {
		ULOGE("VideoCoreOmx: failed to allocate "
			"decoder input buffers pool");
		goto err;
	}

	/* Input buffers queue allocation */
	mInputBufferQueue = vbuf_queue_new();
	if (mInputBufferQueue == NULL) {
		ULOGE("VideoCoreOmx: failed to allocate "
			"decoder input buffers queue");
		goto err;
	}

	ret = vbuf_generic_get_cbs(&cbs);
	if (ret != 0) {
		ULOGE("VideoCoreOmx: failed to get "
			"output buffers allocation callbacks");
		goto err;
	}

	/* Output buffers pool allocation */
	mOutputBufferPool = vbuf_pool_new(
		AVCDECODER_VIDEOCOREOMX_OUTPUT_BUFFER_COUNT + 2, 0, /* TODO */
		sizeof(struct avcdecoder_output_buffer), 0, &cbs);
	if (mOutputBufferPool == NULL) {
		ULOGE("VideoCoreOmx: failed to allocate "
			"decoder output buffers pool");
		goto err;
	}

	return;

err:
	COMPONENT_T *list[4] = { mVideoDecode, mEglRender };
	if (mTunnel)
		ilclient_flush_tunnels(mTunnel, 0);
	if (mVideoDecode)
		ilclient_disable_port_buffers(mVideoDecode,
			130, NULL, NULL, NULL);
	if (mTunnel) {
		ilclient_disable_tunnel(mTunnel);
		ilclient_teardown_tunnels(mTunnel);
		memset(mTunnel, 0, sizeof(mTunnel));
	}
	ilclient_state_transition(list, OMX_StateIdle);
	ilclient_cleanup_components(list);
	mVideoDecode = NULL;
	mEglRender = NULL;

	OMX_Deinit();

	if (mClient != NULL) {
		ilclient_destroy(mClient);
		mClient = NULL;
	}
}


VideoCoreOmxAvcDecoder::~VideoCoreOmxAvcDecoder(
	void)
{
	COMPONENT_T *list[4] = { mVideoDecode, mEglRender };
	ilclient_flush_tunnels(mTunnel, 0);
	ilclient_disable_port_buffers(mVideoDecode, 130, NULL, NULL, NULL);
	ilclient_disable_tunnel(mTunnel);
	ilclient_teardown_tunnels(mTunnel);
	ilclient_state_transition(list, OMX_StateIdle);

	ilclient_cleanup_components(list);

	OMX_Deinit();

	ilclient_destroy(mClient);

	if (mInputBufferQueue)
		vbuf_queue_destroy(mInputBufferQueue);
	if (mInputBufferPool)
		vbuf_pool_destroy(mInputBufferPool);
	if (mOutputBufferPool)
		vbuf_pool_destroy(mOutputBufferPool);

	std::vector<struct vbuf_queue *>::iterator q =
		mOutputBufferQueues.begin();
	while (q != mOutputBufferQueues.end()) {
		vbuf_queue_destroy(*q);
		q++;
	}
}


int VideoCoreOmxAvcDecoder::open(
	uint32_t inputBitstreamFormat,
	const uint8_t *pSps,
	unsigned int spsSize,
	const uint8_t *pPps,
	unsigned int ppsSize)
{
	int ret;
	OMX_ERRORTYPE omxErr;

	if (mConfigured) {
		ULOGE("VideoCoreOmx: decoder is already configured");
		return -1;
	}
	if ((pSps == NULL) || (spsSize <= 4) ||
		(pPps == NULL) || (ppsSize <= 4)) {
		ULOGE("VideoCoreOmx: invalid SPS/PPS");
		return -1;
	}
	if (inputBitstreamFormat != AVCDECODER_BITSTREAM_FORMAT_BYTE_STREAM) {
		ULOGE("VideoCoreOmx: unsupported input bitstream format");
		return -1;
	}

	ret = ilclient_enable_port_buffers(mVideoDecode, 130, NULL, NULL, NULL);
	if (ret != 0) {
		ULOGE("VideoCoreOmx: ilclient_enable_port_buffers() "
			"failed on input (%d)", ret);
		return -1;
	}

	ret = ilclient_change_component_state(mVideoDecode, OMX_StateExecuting);
	if (ret != 0) {
		ULOGE("VideoCoreOmx: failed to change "
			"OMX component state to 'executing' (%d)", ret);
		return -1;
	}

	OMX_BUFFERHEADERTYPE *buf =
		ilclient_get_input_buffer(mVideoDecode, 130, 1);
	if (buf == NULL) {
		ULOGE("VideoCoreOmx: failed to dequeue an input buffer");
		return -1;
	}

	if (buf->nAllocLen < spsSize + ppsSize) {
		ULOGE("VideoCoreOmx: buffer too small for SPS/PPS");
		return -1;
	}

	memcpy(buf->pBuffer, pSps, spsSize);
	memcpy(buf->pBuffer + spsSize, pPps, ppsSize);
	buf->nFilledLen = spsSize + ppsSize;
	buf->nOffset = 0;
	buf->nFlags = OMX_BUFFERFLAG_CODECCONFIG;
	omxErr = OMX_EmptyThisBuffer(ILC_GET_HANDLE(mVideoDecode), buf);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to release input buffer");
		return -1;
	}

	mConfigured = true;
	ULOGI("VideoCoreOmx: decoder is configured");

	return 0;
}


int VideoCoreOmxAvcDecoder::close(
	void)
{
	if (!mConfigured) {
		ULOGE("VideoCoreOmx: decoder is not configured");
		return -1;
	}

	/* TODO */
	mConfigured = false;

	if (mInputBufferPool)
		vbuf_pool_abort(mInputBufferPool);
	if (mOutputBufferPool)
		vbuf_pool_abort(mOutputBufferPool);
	if (mInputBufferQueue)
		vbuf_queue_abort(mInputBufferQueue);

	return 0;
}


void VideoCoreOmxAvcDecoder::setRenderer(
	Renderer *renderer)
{
	mRenderer = renderer;
}


int VideoCoreOmxAvcDecoder::portSettingsChanged(
	void)
{
	int ret = 0, i;
	OMX_ERRORTYPE omxErr;
	OMX_PARAM_PORTDEFINITIONTYPE def;

	ULOGI("VideoCoreOmx: port settings changed event");

	memset(&def, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = 131;
	omxErr = OMX_GetParameter(ILC_GET_HANDLE(mVideoDecode),
		OMX_IndexParamPortDefinition, &def);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to get output port definition (%d)",
			omxErr);
		return -1;
	}

	mFrameWidth = def.format.video.nFrameWidth;
	mFrameHeight = def.format.video.nFrameHeight;
	mSliceHeight = def.format.video.nSliceHeight;
	mStride = def.format.video.nStride;
	switch (def.format.video.eColorFormat) {
	case OMX_COLOR_FormatYUV420PackedPlanar:
		mOutputColorFormat = AVCDECODER_COLOR_FORMAT_YUV420PLANAR;
		break;
	default:
		mOutputColorFormat = AVCDECODER_COLOR_FORMAT_UNKNOWN;
		break;
	}

	if (mRenderer == NULL) {
		ULOGE("VideoCoreOmx: invalid renderer");
		return -1;
	}

	ret = ilclient_setup_tunnel(mTunnel, 0, 0);
	if (ret != 0) {
		ULOGE("VideoCoreOmx: ilclient_setup_tunnel() failed (%d)", ret);
		return -1;
	}

	ret = ilclient_change_component_state(mEglRender, OMX_StateIdle);
	if (ret != 0) {
		ULOGE("VideoCoreOmx: failed to change egl_render OMX component "
			"state to 'idle' (%d)", ret);
		return -1;
	}

	memset(&def, 0, sizeof(OMX_PARAM_PORTDEFINITIONTYPE));
	def.nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
	def.nVersion.nVersion = OMX_VERSION;
	def.nPortIndex = 221;
	omxErr = OMX_GetParameter(ILC_GET_HANDLE(mEglRender),
		OMX_IndexParamPortDefinition, &def);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to get egl_render "
			"input port definition (%d)", omxErr);
		return -1;
	}

	def.nBufferCountActual = 3;
	omxErr = OMX_SetParameter(ILC_GET_HANDLE(mEglRender),
		OMX_IndexParamPortDefinition, &def);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to set egl_render input "
			"port definition (%d)", omxErr);
		return -1;
	}

	omxErr = OMX_GetParameter(ILC_GET_HANDLE(mEglRender),
		OMX_IndexParamPortDefinition, &def);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to get egl_render input "
			"port definition (%d)", omxErr);
		return -1;
	}

	ULOGI("VideoCoreOmx: nBufferCountActual=%d, "
		"nBufferCountMin=%d, nBufferSize=%d",
		def.nBufferCountActual, def.nBufferCountMin,
		def.nBufferSize);
	ULOGI("VideoCoreOmx: width=%d, height=%d, "
		"sliceHeight=%d, stride=%d, colorFormat=%d",
		def.format.video.nFrameWidth, def.format.video.nFrameHeight,
		def.format.video.nSliceHeight, def.format.video.nStride,
		def.format.video.eColorFormat);

	/* ilclient_enable_port(mEglRender, 221);
	 * THIS BLOCKS SO CAN'T BE USED */
	omxErr = OMX_SendCommand(ILC_GET_HANDLE(mEglRender),
		OMX_CommandPortEnable, 221, NULL);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: failed to enable "
			"output port on egl_render (%d)", omxErr);
		return -1;
	}

	ret = ((VideoCoreEglRenderer *)mRenderer)->setVideoDimensions(
		mFrameWidth, mFrameHeight);
	if (ret != 0) {
		ULOGE("VideoCoreOmx: renderer->setVideoDimensions() failed");
		return -1;
	}

	for (i = 0, ret = 0; i < AVCDECODER_VIDEOCOREOMX_OUTPUT_BUFFER_COUNT;
		i++) {
		mEglImage[i] = ((VideoCoreEglRenderer *)
			mRenderer)->getVideoEglImage(i);
		if (mEglImage[i] == EGL_NO_IMAGE_KHR) {
			ULOGE("VideoCoreOmx: failed to get EGL image #%d", i);
			ret = -1;
			break;
		}

		omxErr = OMX_UseEGLImage(ILC_GET_HANDLE(mEglRender),
			&mEglBuffer[i], 221, NULL, mEglImage[i]);
		if (omxErr != OMX_ErrorNone) {
			ULOGE("VideoCoreOmx: OMX_UseEGLImage() "
				"failed on image #%d", i);
			ret = -1;
			break;
		}
	}
	if (ret != 0)
		return -1;

	ret = ilclient_change_component_state(mEglRender, OMX_StateExecuting);
	if (ret != 0) {
		ULOGE("VideoCoreOmx: failed to change egl_render OMX component "
			"state to 'executing' (%d)", ret);
		return -1;
	}

	mCurrentEglImageIndex =
		((VideoCoreEglRenderer *)mRenderer)->swapDecoderEglImage();
	omxErr = OMX_FillThisBuffer(ILC_GET_HANDLE(mEglRender),
		mEglBuffer[mCurrentEglImageIndex]);
	if (omxErr != OMX_ErrorNone) {
		ULOGE("VideoCoreOmx: OMX_FillThisBuffer() failed (%d)", omxErr);
		return -1;
	}

	mConfigured2 = true;

	return ret;
}


int VideoCoreOmxAvcDecoder::getInputBuffer(
	struct vbuf_buffer **buffer,
	bool blocking)
{
	if (buffer == NULL) {
		ULOGE("VideoCoreOmx: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("VideoCoreOmx: decoder is not configured");
		return -1;
	}
	if (mInputBufferPool == NULL) {
		ULOGE("VideoCoreOmx: input buffer pool has not been created");
		return -1;
	}

	if (!mConfigured2) {
		int ret;
		ret = ilclient_remove_event(mVideoDecode,
			OMX_EventPortSettingsChanged, 131, 0, 0, 1);
		if (ret != 0) {
			ret = ilclient_wait_for_event(mVideoDecode,
				OMX_EventPortSettingsChanged, 131, 0, 0, 1,
				ILCLIENT_EVENT_ERROR |
				ILCLIENT_PARAMETER_CHANGED, 0);
		}
		if (ret == 0) {
			ret = portSettingsChanged();
			if (ret != 0) {
				ULOGE("VideoCoreOmx: portSettingsChanged() "
					"failed (%d)", ret);
				return -1;
			}
		}
	}

	struct vbuf_buffer *buf = NULL;
	int ret = vbuf_pool_get(mInputBufferPool, (blocking) ? -1 : 0, &buf);
	if ((ret != 0) || (buf == NULL)) {
		ULOGW("VideoCoreOmx: failed to get an input buffer (%d)", ret);
		return -2;
	}

	*buffer = buf;

	return 0;
}


int VideoCoreOmxAvcDecoder::queueInputBuffer(
	struct vbuf_buffer *buffer)
{
	if (buffer == NULL) {
		ULOGE("VideoCoreOmx: invalid buffer pointer");
		return -1;
	}
	if (!mConfigured) {
		ULOGE("VideoCoreOmx: decoder is not configured");
		return -1;
	}
	if (mInputBufferQueue == NULL) {
		ULOGE("VideoCoreOmx: input queue has not been created");
		return -1;
	}

	uint64_t ts = 0;
	uint32_t flags = 0;
	struct avcdecoder_input_buffer *data =
		(struct avcdecoder_input_buffer *)
		vbuf_get_metadata_ptr(buffer);
	if (data)
		ts = data->auNtpTimestampRaw;

	flags = OMX_BUFFERFLAG_ENDOFFRAME;
	if (mFirstFrame) {
		flags |= OMX_BUFFERFLAG_STARTTIME;
		mFirstFrame = false;
	}
	vbuf_ilclient_set_info(buffer, ts, flags);
	vbuf_queue_push(mInputBufferQueue, buffer);

	if (!mConfigured2) {
		int ret;
		ret = ilclient_remove_event(mVideoDecode,
			OMX_EventPortSettingsChanged, 131, 0, 0, 1);
		if (ret != 0) {
			ret = ilclient_wait_for_event(mVideoDecode,
				OMX_EventPortSettingsChanged, 131, 0, 0, 1,
				ILCLIENT_EVENT_ERROR |
				ILCLIENT_PARAMETER_CHANGED, 0);
		}
		if (ret == 0) {
			ret = portSettingsChanged();
			if (ret != 0) {
				ULOGE("VideoCoreOmx: portSettingsChanged() "
					"failed (%d)", ret);
				return -1;
			}
		}
	}

	return 0;
}


struct vbuf_queue *VideoCoreOmxAvcDecoder::addOutputQueue(
	void)
{
	struct vbuf_queue *q = vbuf_queue_new();
	if (q == NULL) {
		ULOGE("VideoCoreOmx: queue allocation failed");
		return NULL;
	}

	mOutputBufferQueues.push_back(q);
	return q;
}


int VideoCoreOmxAvcDecoder::removeOutputQueue(
	struct vbuf_queue *queue)
{
	if (queue == NULL) {
		ULOGE("VideoCoreOmx: invalid queue pointer");
		return -1;
	}

	bool found = false;
	int ret;
	std::vector<struct vbuf_queue *>::iterator q =
		mOutputBufferQueues.begin();

	while (q != mOutputBufferQueues.end()) {
		if (*q == queue) {
			mOutputBufferQueues.erase(q);
			ret = vbuf_queue_destroy(*q);
			if (ret != 0)
				ULOGE("VideoCoreOmx: vbuf_queue_destroy() "
					"failed (%d)", ret);
			found = true;
			break;
		}
		q++;
	}

	return (found) ? 0 : -1;
}


bool VideoCoreOmxAvcDecoder::isOutputQueueValid(
	struct vbuf_queue *queue)
{
	if (queue == NULL) {
		ULOGE("VideoCoreOmx: invalid queue pointer");
		return false;
	}

	bool found = false;
	std::vector<struct vbuf_queue *>::iterator q =
		mOutputBufferQueues.begin();

	while (q != mOutputBufferQueues.end()) {
		if (*q == queue) {
			found = true;
			break;
		}
		q++;
	}

	return found;
}


void VideoCoreOmxAvcDecoder::fillBufferDoneCallback(
	void *data,
	COMPONENT_T *comp,
	OMX_BUFFERHEADERTYPE *omxBuf)
{
	int ret;
	OMX_ERRORTYPE omxErr;
	VideoCoreOmxAvcDecoder *decoder = (VideoCoreOmxAvcDecoder *)data;
	struct vbuf_buffer *inputBuffer = NULL, *outputBuffer = NULL;
	struct avcdecoder_input_buffer *inputMeta = NULL;
	struct avcdecoder_output_buffer *outputMeta = NULL;
	uint64_t ts = ((uint64_t)omxBuf->nTimeStamp.nHighPart << 32) |
		((uint64_t)omxBuf->nTimeStamp.nLowPart & 0xFFFFFFFF);
	struct vbuf_buffer *b;
	uint8_t *userData = NULL;
	unsigned int userDataSize = 0;
	struct timespec t1;

	if (!decoder->mConfigured) {
		vbuf_queue_flush(decoder->mInputBufferQueue);
		goto out;
	}

	while ((ret = vbuf_queue_peek(decoder->mInputBufferQueue,
		-1, &b)) == 0) {
		struct avcdecoder_input_buffer *d =
			(struct avcdecoder_input_buffer *)
			vbuf_get_metadata_ptr(b);

		if (ts == d->auNtpTimestampRaw) {
			vbuf_queue_pop(decoder->mInputBufferQueue,
				0, &inputBuffer);
			inputMeta = d;
			break;
		} else {
			ret = vbuf_queue_pop(decoder->mInputBufferQueue,
				0, &b);
			if ((ret == 0) && (b))
				vbuf_unref(&b);
			ULOGD("VideoCoreOmx: discarded input buffer with "
				"TS %" PRIu64 " (expected %" PRIu64 ")",
				d->auNtpTimestampRaw, ts);
		}
	}

	if ((inputBuffer == NULL) || (inputMeta == NULL)) {
		ULOGW("VideoCoreOmx: failed to find buffer for TS %llu", ts);
		goto out;
	}

	ret = vbuf_pool_get(decoder->mOutputBufferPool, 0, &outputBuffer);
	if ((ret != 0) || (outputBuffer == NULL)) {
		ULOGE("VideoCoreOmx: failed to get an output buffer (%d)", ret);
		goto out;
	}

	outputMeta = (struct avcdecoder_output_buffer *)
		vbuf_get_metadata_ptr(outputBuffer);
	if (outputMeta == NULL) {
		ULOGE("VideoCoreOmx: invalid output buffer");
		goto out;
	}

	clock_gettime(CLOCK_MONOTONIC, &t1);
	memset(outputMeta, 0, sizeof(*outputMeta));

	/* Pixel data */
	outputMeta->plane[0] = (uint8_t *)decoder->mCurrentEglImageIndex;
	outputMeta->width = decoder->mFrameWidth;
	outputMeta->height = decoder->mFrameHeight;
	outputMeta->sarWidth = 1; /* TODO */
	outputMeta->sarHeight = 1; /* TODO */
	outputMeta->stride[0] = decoder->mStride;
	outputMeta->stride[1] = decoder->mStride / 2;
	outputMeta->stride[2] = decoder->mStride / 2;
	outputMeta->colorFormat = decoder->mOutputColorFormat;
	outputMeta->isComplete = inputMeta->isComplete;
	outputMeta->hasErrors = inputMeta->hasErrors;
	outputMeta->isRef = inputMeta->isRef;
	outputMeta->isSilent = inputMeta->isSilent;
	outputMeta->auNtpTimestamp =
		inputMeta->auNtpTimestamp;
	outputMeta->auNtpTimestampRaw =
		inputMeta->auNtpTimestampRaw;
	outputMeta->auNtpTimestampLocal =
		inputMeta->auNtpTimestampLocal;
	outputMeta->demuxOutputTimestamp =
		inputMeta->demuxOutputTimestamp;
	outputMeta->decoderOutputTimestamp =
		(uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;

	/* Frame metadata */
	if (inputMeta->hasMetadata) {
		memcpy(&outputMeta->metadata, &inputMeta->metadata,
			sizeof(struct vmeta_frame_v2));
		outputMeta->hasMetadata = true;
	} else {
		outputMeta->hasMetadata = false;
	}

	/* User data */
	userDataSize = vbuf_get_userdata_size(inputBuffer);
	userData = vbuf_get_userdata_ptr(inputBuffer);
	if ((userData) && (userDataSize > 0)) {
		ret = vbuf_set_userdata_capacity(
			outputBuffer, userDataSize);
		if (ret < (signed)userDataSize) {
			ULOGE("ffmpeg: failed to realloc user data buffer");
		} else {
			uint8_t *dstBuf = vbuf_get_userdata_ptr(outputBuffer);
			memcpy(dstBuf, userData, userDataSize);
			vbuf_set_userdata_size(outputBuffer, userDataSize);
		}
	} else {
		vbuf_set_userdata_size(outputBuffer, 0);
	}

	/* Push the frame */
	if (!outputMeta->isSilent) {
		std::vector<struct vbuf_queue *>::iterator q =
			decoder->mOutputBufferQueues.begin();
		while (q != decoder->mOutputBufferQueues.end()) {
			vbuf_queue_push(*q, outputBuffer);
			q++;
		}
	} else {
		ULOGI("VideoCoreOmx: silent frame (ignored)");
	}

out:
	if (outputBuffer != NULL)
		vbuf_unref(&outputBuffer);
	if (inputBuffer != NULL)
		vbuf_unref(&inputBuffer);

	decoder->mCurrentEglImageIndex = ((VideoCoreEglRenderer *)
		decoder->mRenderer)->swapDecoderEglImage();
	omxErr = OMX_FillThisBuffer(ILC_GET_HANDLE(decoder->mEglRender),
		decoder->mEglBuffer[decoder->mCurrentEglImageIndex]);
	if (omxErr != OMX_ErrorNone)
		ULOGE("VideoCoreOmx: OMX_FillThisBuffer() failed");
}

} /* namespace Pdraw */

#endif /* USE_VIDEOCOREOMX */
