/**
 * Parrot Drones Awesome Video Viewer Library
 * Video frame filter
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

#ifndef _PDRAW_FILTER_VIDEOFRAME_HPP_
#define _PDRAW_FILTER_VIDEOFRAME_HPP_

#include <pthread.h>
#include <pdraw/pdraw_defs.h>
#include "pdraw_avcdecoder.hpp"

namespace Pdraw {


class Media;
class VideoMedia;


class VideoFrameFilter {
public:
	VideoFrameFilter(
		VideoMedia *media,
		AvcDecoder *decoder,
		bool frameByFrame = false);

	VideoFrameFilter(
		VideoMedia *media,
		AvcDecoder *decoder,
		pdraw_video_frame_filter_callback_t cb,
		void *userPtr,
		bool frameByFrame = false);

	~VideoFrameFilter(
		void);

	/**
	 * timeout : time in microseconds to wait for a frame
	 *  0: don't wait
	 * -1: wait forever
	 * >0: wait time
	 */
	int getLastFrame(
		struct pdraw_video_frame *frame,
		int timeout = 0);

	Media *getMedia(
		void) {
		return mMedia;
	}

	VideoMedia *getVideoMedia(
		void) {
		return (VideoMedia *)mMedia;
	}

private:
	static void *runThread(
		void *ptr);

	Media *mMedia;
	AvcDecoder *mDecoder;
	struct vbuf_queue *mDecoderOutputBufferQueue;
	pthread_mutex_t mMutex;
	pthread_t mThread;
	pthread_cond_t mCondition;
	bool mThreadLaunched;
	bool mThreadShouldStop;
	bool mFrameByFrame;
	pdraw_video_frame_filter_callback_t mCb;
	void *mUserPtr;
	uint8_t *mBuffer[2];
	uint8_t *mUserData[2];
	unsigned int mUserDataBuferSize[2];
	struct pdraw_video_frame mBufferData[2];
	unsigned int mBufferIndex;
	enum pdraw_color_format mColorFormat;
	unsigned int mWidth;
	unsigned int mHeight;
	bool mFrameAvailable;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_FILTER_VIDEOFRAME_HPP_ */
