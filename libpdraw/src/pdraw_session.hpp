/**
 * Parrot Drones Awesome Video Viewer Library
 * Session
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

#ifndef _PDRAW_SESSION_HPP_
#define _PDRAW_SESSION_HPP_

#include <inttypes.h>
#include <libpomp.h>
#include <string>
#include <vector>
#include "pdraw_metadata_session.hpp"
#include "pdraw_media.hpp"
#include "pdraw_demuxer.hpp"
#include "pdraw_renderer.hpp"

namespace Pdraw {


class Settings;


class Session {
public:
	Session(
		Settings *settings);

	~Session(
		void);

	int open(
		const std::string &url);

	int open(
		const std::string &url,
		const std::string &ifaceAddr);

	int open(
		const std::string &localAddr,
		int localStreamPort,
		int localControlPort,
		const std::string &remoteAddr,
		int remoteStreamPort,
		int remoteControlPort,
		const std::string &ifaceAddr);

	int openSdp(
		const std::string &sdp,
		const std::string &ifaceAddr);

	Media *addMedia(
		enum elementary_stream_type esType);

	Media *addMedia(
		enum elementary_stream_type esType,
		Demuxer *demuxer,
		int demuxEsIndex);

	int removeMedia(
		Media *media);

	int removeMedia(
		unsigned int index);

	unsigned int getMediaCount(
		void);

	Media *getMedia(
		unsigned int index);

	Media *getMediaById(
		unsigned int id);

	int enableRenderer(
		void);

	int disableRenderer(
		void);

	uint64_t getDuration(
		void);

	uint64_t getCurrentTime(
		void);

	Demuxer *getDemuxer(
		void) {
		return mDemuxer;
	}

	Renderer *getRenderer(
		void) {
		return mRenderer;
	}

	Settings *getSettings(
		void) {
		return mSettings;
	}

	enum pdraw_session_type getSessionType(
		void) {
		return mSessionType;
	}

	SessionSelfMetadata *getSelfMetadata(
		void) {
		return &mSelfMetadata;
	}

	SessionPeerMetadata *getPeerMetadata(
		void) {
		return &mPeerMetadata;
	}

	void getCameraOrientationForHeadtracking(
		float *pan,
		float *tilt);

	struct pomp_loop *getLoop() {
		return mLoop;
	}

	void *getJniEnv(
		void) {
		return mJniEnv;
	}

	void setJniEnv(
		void *jniEnv) {
		mJniEnv = jniEnv;
	}

private:
	int addMediaFromDemuxer(
		void);

	static void* runLoopThread(
		void *ptr);

	enum pdraw_session_type mSessionType;
	Settings *mSettings;
	SessionSelfMetadata mSelfMetadata;
	SessionPeerMetadata mPeerMetadata;
	struct pomp_loop *mLoop;
	pthread_t mLoopThread;
	bool mLoopThreadLaunched;
	bool mThreadShouldStop;
	std::vector<Media *> mMedias;
	Demuxer *mDemuxer;
	Renderer *mRenderer;
	unsigned int mMediaIdCounter;
	void *mJniEnv;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SESSION_HPP_ */
