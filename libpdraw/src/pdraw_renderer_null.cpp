/**
 * Parrot Drones Awesome Video Viewer Library
 * Null renderer
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

#include "pdraw_renderer_null.hpp"
#include <unistd.h>
#include <time.h>
#define ULOG_TAG libpdraw
#include <ulog.h>

namespace Pdraw {


NullRenderer::NullRenderer(
	Session *session)
{
	int ret;
	mSession = session;
	mMedia = NULL;
	mDecoder = NULL;
	mDecoderOutputBufferQueue = NULL;
	mRendererThreadLaunched = false;
	mThreadShouldStop = false;

	ret = pthread_create(&mRendererThread, NULL,
		runRendererThread, (void*)this);
	if (ret != 0) {
		ULOGE("NullRenderer: renderer thread creation failed (%d)",
			ret);
		return;
	}

	mRendererThreadLaunched = true;
}


NullRenderer::~NullRenderer(
	void)
{
	int ret;

	mThreadShouldStop = true;

	if (mRendererThreadLaunched) {
		ret = pthread_join(mRendererThread, NULL);
		if (ret != 0)
			ULOGE("NullRenderer: pthread_join() "
				"failed (%d)", ret);
	}

	if (mDecoder) {
		ret = removeAvcDecoder(mDecoder);
		if (ret != 0)
			ULOGE("NullRenderer: removeAvcDecoder() "
				"failed (%d)", ret);
	}
}


int NullRenderer::addAvcDecoder(
	AvcDecoder *decoder)
{
	if (decoder == NULL) {
		ULOGE("NullRenderer: invalid decoder pointer");
		return -1;
	}
	if (mDecoder != NULL) {
		ULOGE("NullRenderer: multiple decoders are not supported");
		return -1;
	}

	mDecoderOutputBufferQueue = decoder->addOutputQueue();
	if (mDecoderOutputBufferQueue == NULL) {
		ULOGE("NullRenderer: failed to add output queue to decoder");
		return -1;
	}

	mDecoder = decoder;
	mMedia = mDecoder->getMedia();

	return 0;
}


int NullRenderer::removeAvcDecoder(
	AvcDecoder *decoder)
{
	if (decoder == NULL) {
		ULOGE("NullRenderer: invalid decoder pointer");
		return -1;
	}

	if (decoder != mDecoder) {
		ULOGE("NullRenderer: invalid decoder");
		return -1;
	}

	if (mDecoderOutputBufferQueue) {
		int ret = decoder->removeOutputQueue(mDecoderOutputBufferQueue);
		if (ret != 0)
			ULOGE("NullRenderer: failed to remove "
				"output queue from decoder");
	}

	mDecoder = NULL;
	mDecoderOutputBufferQueue = NULL;

	return 0;
}


int NullRenderer::setRendererParams(
	int windowWidth,
	int windowHeight,
	int renderX,
	int renderY,
	int renderWidth,
	int renderHeight,
	bool hmdDistorsionCorrection,
	bool headtracking,
	void *uiHandler)
{
	return 0;
}


int NullRenderer::render(
	uint64_t lastRenderTime)
{
	return 0;
}


void* NullRenderer::runRendererThread(
	void *ptr)
{
	NullRenderer *renderer = (NullRenderer*)ptr;
	int ret;
	struct vbuf_buffer *buffer;

	while (!renderer->mThreadShouldStop) {
		if ((renderer->mDecoder == NULL) ||
			(!renderer->mDecoder->isConfigured())) {
			usleep(5000); /* TODO */
			continue;
		}

		ret = renderer->mDecoder->dequeueOutputBuffer(
			renderer->mDecoderOutputBufferQueue, &buffer, true);
		if ((ret != 0) || (buffer == NULL)) {
			ULOGE("NullRenderer: failed to get buffer "
				"from queue (%d)", ret);
			usleep(5000); /* TODO */
		}

		struct avcdecoder_output_buffer *data =
			(struct avcdecoder_output_buffer *)
			vbuf_get_metadata_ptr(buffer);
		struct timespec t1;
		clock_gettime(CLOCK_MONOTONIC, &t1);
		uint64_t renderTimestamp =
			(uint64_t)t1.tv_sec * 1000000 +
			(uint64_t)t1.tv_nsec / 1000;

		ULOGI("NullRenderer: frame (decoding: %.2fms, "
			"rendering: %.2fms, est. latency: %.2fms)",
			  (float)(data->decoderOutputTimestamp -
			  	data->demuxOutputTimestamp) / 1000.,
			  (float)(renderTimestamp -
			  	data->decoderOutputTimestamp) / 1000.,
			  (data->auNtpTimestampLocal != 0) ?
			  	(float)(renderTimestamp -
			  	data->auNtpTimestampLocal) / 1000. : 0.);

		ret = renderer->mDecoder->releaseOutputBuffer(&buffer);
		if (ret != 0)
			ULOGE("NullRenderer: failed to release buffer (%d)",
				ret);
	}

	return NULL;
}

} /* namespace Pdraw */
