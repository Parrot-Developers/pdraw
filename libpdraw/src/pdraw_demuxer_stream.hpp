/**
 * Parrot Drones Awesome Video Viewer Library
 * Streaming demuxer
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

#ifndef _PDRAW_DEMUXER_STREAM_HPP_
#define _PDRAW_DEMUXER_STREAM_HPP_

#include "pdraw_demuxer.hpp"
#include "pdraw_avcdecoder.hpp"
#include <video-streaming/vstrm.h>
#include <librtsp.h>
#include <libsdp.h>
#include <h264/h264.h>
#include <libpomp.h>
#include <string>

namespace Pdraw {


#define DEMUXER_STREAM_DEFAULT_LOCAL_STREAM_PORT 55004
#define DEMUXER_STREAM_DEFAULT_LOCAL_CONTROL_PORT 55005


class StreamDemuxer : public Demuxer {
public:
	StreamDemuxer(
		Session *session);

	virtual ~StreamDemuxer(
		void);

	enum demuxer_type getType(
		void) {
		return DEMUXER_TYPE_STREAM;
	}

	bool isConfigured(
		void) {
		return mConfigured;
	}

	int close(
		void);

	int getElementaryStreamCount(
		void);

	enum elementary_stream_type getElementaryStreamType(
		int esIndex);

	int getElementaryStreamVideoDimensions(
		int esIndex,
		unsigned int *width,
		unsigned int *height,
		unsigned int *cropLeft,
		unsigned int *cropRight,
		unsigned int *cropTop,
		unsigned int *cropBottom,
		unsigned int *sarWidth,
		unsigned int *sarHeight);

	int getElementaryStreamVideoFov(
		int esIndex,
		float *hfov,
		float *vfov);

	int setElementaryStreamDecoder(
		int esIndex,
		Decoder *decoder);

	int play(
		float speed = 1.0f);

	bool isPaused(
		void);

	int previous(
		void);

	int next(
		void);

	int seek(
		int64_t delta,
		bool exact = false);

	int seekTo(
		uint64_t timestamp,
		bool exact = false);

	uint64_t getDuration(
		void);

	uint64_t getCurrentTime(
		void);

	Session *getSession(
		void) {
		return mSession;
	}

protected:
	int openWithSdp(
		const std::string &sdp,
		const std::string &ifaceAddr);

	int openRtsp(
		const std::string &url,
		const std::string &ifaceAddr);

	int createReceiver(
		void);

	int destroyReceiver(
		void);

	virtual int openRtpAvp(
		void) = 0;

	virtual int sendCtrl(
		struct vstrm_receiver *stream,
		struct pomp_buffer *buf) = 0;

	std::string mLocalAddr;
	uint16_t mLocalStreamPort;
	uint16_t mLocalControlPort;
	std::string mRemoteAddr;
	uint16_t mRemoteStreamPort;
	uint16_t mRemoteControlPort;
	std::string mIfaceAddr;
	struct vstrm_receiver *mReceiver;

private:
	int internalPlay(
		float speed);

	int internalPause(
		void);

	static void onRtspConnectionState(
		struct rtsp_client *client,
		enum rtsp_conn_state state,
		void *userdata);

	static void onRtspOptionsResp(
		struct rtsp_client *client,
		enum rtsp_req_status status,
		int status_code,
		uint32_t methods,
		void *userdata,
		void *req_userdata);

	static void onRtspDescribeResp(
		struct rtsp_client *client,
		enum rtsp_req_status status,
		int status_code,
		const char *sdp,
		void *userdata,
		void *req_userdata);

	static void onRtspSetupResp(
		struct rtsp_client *client,
		enum rtsp_req_status status,
		int status_code,
		int server_stream_port,
		int server_control_port,
		int ssrc_valid,
		uint32_t ssrc,
		void *userdata,
		void *req_userdata);

	static void onRtspPlayResp(
		struct rtsp_client *client,
		enum rtsp_req_status status,
		int status_code,
		const struct rtsp_range *range,
		float scale,
		int seq_valid,
		uint16_t seq,
		int rtptime_valid,
		uint32_t rtptime,
		void *userdata,
		void *req_userdata);

	static void onRtspPauseResp(
		struct rtsp_client *client,
		enum rtsp_req_status status,
		int status_code,
		const struct rtsp_range *range,
		void *userdata,
		void *req_userdata);

	static void onRtspTeardownResp(
		struct rtsp_client *client,
		enum rtsp_req_status status,
		int status_code,
		void *userdata,
		void *req_userdata);

	static int openDecoder(
		StreamDemuxer *demuxer);

	static void h264UserDataSeiCb(
		struct h264_ctx *ctx,
		const uint8_t *buf,
		size_t len,
		const struct h264_sei_user_data_unregistered *sei,
		void *userdata);

	static int sendCtrlCb(
		struct vstrm_receiver *stream,
		struct pomp_buffer *buf,
		void *userdata);

	static void codecInfoChangedCb(
		struct vstrm_receiver *stream,
		const struct vstrm_codec_info *info,
		void *userdata);

	static void recvFrameCb(
		struct vstrm_receiver *stream,
		struct vstrm_frame *frame,
		void *userdata);

	static void sessionMetadataPeerChangedCb(
		struct vstrm_receiver *stream,
		const struct vmeta_session *meta,
		void *userdata);

	AvcDecoder *mDecoder;
	struct avcdecoder_input_source mDecoderSource;
	uint32_t mDecoderBitstreamFormat;
	struct vbuf_buffer *mCurrentBuffer;
	bool mRtspRunning;
	struct rtsp_client *mRtspClient;
	struct h264_reader *mH264Reader;
	struct vstrm_codec_info mCodecInfo;
	uint32_t mSsrc;
	bool mRunning;
	uint64_t mStartTime;
	uint64_t mDuration;
	uint64_t mCurrentTime;
	uint64_t mPausePoint;
	int64_t mNtpToNptOffset;
	unsigned int mWidth;
	unsigned int mHeight;
	unsigned int mCropLeft;
	unsigned int mCropRight;
	unsigned int mCropTop;
	unsigned int mCropBottom;
	unsigned int mSarWidth;
	unsigned int mSarHeight;
	float mHfov;
	float mVfov;
	float mSpeed;
};

} /* namespace Pdraw */

#endif /* !_PDRAW_DEMUXER_STREAM_HPP_ */
