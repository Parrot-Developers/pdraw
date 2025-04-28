/**
 * Parrot Drones Audio and Video Vector library
 * Session
 *
 * Copyright (c) 2018 Parrot Drones SAS
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PDRAW_SESSION_HPP_
#define _PDRAW_SESSION_HPP_

#include "pdraw_alsa_source.hpp"
#include "pdraw_decoder_audio.hpp"
#include "pdraw_decoder_video.hpp"
#include "pdraw_demuxer.hpp"
#include "pdraw_encoder_audio.hpp"
#include "pdraw_encoder_video.hpp"
#include "pdraw_external_audio_sink.hpp"
#include "pdraw_external_audio_source.hpp"
#include "pdraw_external_coded_video_sink.hpp"
#include "pdraw_external_coded_video_source.hpp"
#include "pdraw_external_raw_video_sink.hpp"
#include "pdraw_external_raw_video_source.hpp"
#include "pdraw_media.hpp"
#include "pdraw_muxer.hpp"
#include "pdraw_renderer_audio.hpp"
#include "pdraw_renderer_video.hpp"
#include "pdraw_scaler_video.hpp"
#include "pdraw_settings.hpp"
#include "pdraw_source.hpp"
#include "pdraw_vipc_source.hpp"

#include <inttypes.h>

#ifdef _WIN32
#	include <winsock2.h>
#	undef OPAQUE
#	undef near
#	undef far
#	define IPTOS_PREC_INTERNETCONTROL 0xc0
#	define IPTOS_PREC_FLASHOVERRIDE 0x80
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <netinet/ip.h>
#endif /* !_WIN32 */

#include <queue>
#include <string>
#include <vector>

#include <futils/futils.h>
#include <libpomp.h>
#include <pdraw/pdraw.hpp>

#ifdef _WIN32
#	define PIPE_BUF 4096
#endif /* _WIN32 */

namespace Pdraw {


class Settings;


class Session : public IPdraw,
		public Element::Listener,
		public Source::Listener {
public:
	enum State {
		STOPPED = 0,
		READY,
		STOPPING,
	};


	class PipelineFactory {
	public:
		PipelineFactory(Session *session);

		~PipelineFactory(void);

		void onElementStateChanged(Element *element,
					   Element::State state);

		void onOutputMediaAdded(Source *source, Media *media);

		void onOutputMediaRemoved(Source *source, Media *media);

		int dumpPipeline(const std::string &fileName);

		int addMediaToVideoRenderer(unsigned int mediaId,
					    Pdraw::VideoRenderer *renderer);

		int addMediaToAudioRenderer(unsigned int mediaId,
					    Pdraw::AudioRenderer *renderer);

		int
		addMediaToMuxer(unsigned int mediaId,
				Pdraw::Muxer *muxer,
				const struct pdraw_muxer_media_params *params);

		int addVideoEncoderForMedia(
			Source *source,
			RawVideoMedia *media,
			const struct venc_config *params,
			IPdraw::IVideoEncoder::Listener *listener,
			Pdraw::VideoEncoder *encoder = nullptr);

		int
		addVideoScalerForMedia(Source *source,
				       RawVideoMedia *media,
				       const struct vscale_config *params,
				       IPdraw::IVideoScaler::Listener *listener,
				       Pdraw::VideoScaler *scaler = nullptr);

		int addAudioEncoderForMedia(
			Source *source,
			AudioMedia *media,
			const struct aenc_config *params,
			IPdraw::IAudioEncoder::Listener *listener,
			Pdraw::AudioEncoder *encoder = nullptr);

	private:
		int addVideoDecoderForMedia(Source *source,
					    CodedVideoMedia *media);

		int addAudioDecoderForMedia(Source *source, AudioMedia *media);

		int addMediaToVideoRenderer(Source *source,
					    RawVideoMedia *media,
					    Pdraw::VideoRenderer *renderer);

		int addMediaToAllVideoRenderers(Source *source,
						RawVideoMedia *media);

		int addAllMediaToVideoRenderer(Pdraw::VideoRenderer *renderer);

		int addMediaToAudioRenderer(Source *source,
					    AudioMedia *media,
					    Pdraw::AudioRenderer *renderer);

		int addMediaToAllAudioRenderers(Source *source,
						AudioMedia *media);

		int addAllMediaToAudioRenderer(Pdraw::AudioRenderer *renderer);

		int
		addMediaToMuxer(Source *source,
				Media *media,
				Pdraw::Muxer *muxer,
				const struct pdraw_muxer_media_params *params);

		Session *mSession;
	};


	Session(struct pomp_loop *loop, IPdraw::Listener *listener);

	~Session(void);


	/*
	 * API methods
	 */

	int stop(void) override;

	int createDemuxer(const std::string &url,
			  const struct pdraw_demuxer_params *params,
			  IPdraw::IDemuxer::Listener *listener,
			  IPdraw::IDemuxer **retObj) override;

	int createDemuxer(const std::string &localAddr,
			  uint16_t localStreamPort,
			  uint16_t localControlPort,
			  const std::string &remoteAddr,
			  uint16_t remoteStreamPort,
			  uint16_t remoteControlPort,
			  const struct pdraw_demuxer_params *params,
			  IPdraw::IDemuxer::Listener *listener,
			  IPdraw::IDemuxer **retObj) override;

	int createDemuxer(const std::string &url,
			  struct mux_ctx *mux,
			  const struct pdraw_demuxer_params *params,
			  IPdraw::IDemuxer::Listener *listener,
			  IPdraw::IDemuxer **retObj) override;

	int createMuxer(const std::string &url,
			const struct pdraw_muxer_params *params,
			IPdraw::IMuxer::Listener *listener,
			IPdraw::IMuxer **retObj) override;

	/* Called on the rendering thread */
	int
	createVideoRenderer(unsigned int mediaId,
			    const struct pdraw_rect *renderPos,
			    const struct pdraw_video_renderer_params *params,
			    IPdraw::IVideoRenderer::Listener *listener,
			    IPdraw::IVideoRenderer **retObj) override;

	int
	createAudioRenderer(unsigned int mediaId,
			    const struct pdraw_audio_renderer_params *params,
			    IPdraw::IAudioRenderer::Listener *listener,
			    IPdraw::IAudioRenderer **retObj) override;

	int createVipcSource(const struct pdraw_vipc_source_params *params,
			     IPdraw::IVipcSource::Listener *listener,
			     IPdraw::IVipcSource **retObj) override;

	int
	createCodedVideoSource(const struct pdraw_video_source_params *params,
			       IPdraw::ICodedVideoSource::Listener *listener,
			       IPdraw::ICodedVideoSource **retObj) override;

	int createRawVideoSource(const struct pdraw_video_source_params *params,
				 IPdraw::IRawVideoSource::Listener *listener,
				 IPdraw::IRawVideoSource **retObj) override;

	int createCodedVideoSink(unsigned int mediaId,
				 const struct pdraw_video_sink_params *params,
				 IPdraw::ICodedVideoSink::Listener *listener,
				 IPdraw::ICodedVideoSink **retObj) override;

	int createRawVideoSink(unsigned int mediaId,
			       const struct pdraw_video_sink_params *params,
			       IPdraw::IRawVideoSink::Listener *listener,
			       IPdraw::IRawVideoSink **retObj) override;

	int createAlsaSource(const struct pdraw_alsa_source_params *params,
			     IPdraw::IAlsaSource::Listener *listener,
			     IPdraw::IAlsaSource **retObj) override;

	int createAudioSource(const struct pdraw_audio_source_params *params,
			      IPdraw::IAudioSource::Listener *listener,
			      IPdraw::IAudioSource **retObj) override;

	int createAudioSink(unsigned int mediaId,
			    IPdraw::IAudioSink::Listener *listener,
			    IPdraw::IAudioSink **retObj) override;

	int createVideoEncoder(unsigned int mediaId,
			       const struct venc_config *params,
			       IPdraw::IVideoEncoder::Listener *listener,
			       IPdraw::IVideoEncoder **retObj) override;

	int createVideoScaler(unsigned int mediaId,
			      const struct vscale_config *params,
			      IPdraw::IVideoScaler::Listener *listener,
			      IPdraw::IVideoScaler **retObj) override;

	int createAudioEncoder(unsigned int mediaId,
			       const struct aenc_config *params,
			       IPdraw::IAudioEncoder::Listener *listener,
			       IPdraw::IAudioEncoder **retObj) override;

	void getFriendlyNameSetting(std::string *friendlyName) override;

	void setFriendlyNameSetting(const std::string &friendlyName) override;

	void getSerialNumberSetting(std::string *serialNumber) override;

	void setSerialNumberSetting(const std::string &serialNumber) override;

	void getSoftwareVersionSetting(std::string *softwareVersion) override;

	void
	setSoftwareVersionSetting(const std::string &softwareVersion) override;

	int dumpPipeline(const std::string &fileName) override;


	/*
	 * Internal methods
	 */

	Settings *getSettings(void)
	{
		return &mSettings;
	}

	struct pomp_loop *getLoop() const
	{
		return mLoop;
	}

	int addMediaToVideoRenderer(unsigned int mediaId,
				    Pdraw::VideoRenderer *renderer);

	int addMediaToAudioRenderer(unsigned int mediaId,
				    Pdraw::AudioRenderer *renderer);

	int addMediaToMuxer(unsigned int mediaId,
			    Pdraw::Muxer *muxer,
			    const struct pdraw_muxer_media_params *params);

	void asyncElementDelete(Element *element);

	/* Called on the loop thread */
	void socketCreated(int fd);

private:
	int internalCreateCodedVideoSink(
		Source *source,
		CodedVideoMedia *media,
		const struct pdraw_video_sink_params *params,
		ICodedVideoSink::Listener *listener,
		ICodedVideoSink **retObj);

	int
	internalCreateRawVideoSink(Source *source,
				   RawVideoMedia *media,
				   const struct pdraw_video_sink_params *params,
				   IRawVideoSink::Listener *listener,
				   IRawVideoSink **retObj);

	int internalCreateAudioSink(Source *source,
				    AudioMedia *media,
				    IPdraw::IAudioSink::Listener *listener,
				    IPdraw::IAudioSink **retObj);

	void setState(enum State state);

	void onElementStateChanged(Element *element,
				   Element::State state) override;

	void asyncElementStateChange(Element *element,
				     Element::State state) override;

	void onOutputMediaAdded(Source *source,
				Media *media,
				void *elementUserData) override;

	void onOutputMediaRemoved(Source *source,
				  Media *media,
				  void *elementUserData) override;

	void stopResp(int status);

	static const char *stateStr(enum State val);

	int deleteElement(Element *element);

	PipelineFactory mFactory;
	IPdraw::Listener *mListener;
	enum State mState;
	struct pomp_loop *mLoop;
	pthread_t mLoopThread;
	pthread_mutex_t mMutex;
	Settings mSettings;
	std::vector<Element *> mElements;

	/* Calls from idle functions */
	pthread_mutex_t mAsyncMutex;
	static void idleElementStateChange(void *userdata);
	std::queue<Element *> mElementStateChangeElementArgs;
	std::queue<Element::State> mElementStateChangeStateArgs;
	static void idleElementDelete(void *userdata);
	std::queue<Element *> mElementDeleteElementArgs;
	static void callStopResponse(void *userdata);
	std::queue<int> mStopRespStatusArgs;
	static void callOnMediaAdded(void *userdata);
	std::queue<struct pdraw_media_info> mMediaAddedInfoArgs;
	std::queue<void *> mMediaAddedElementUserDataArgs;
	static void callOnMediaRemoved(void *userdata);
	std::queue<struct pdraw_media_info> mMediaRemovedInfoArgs;
	std::queue<void *> mMediaRemovedElementUserDataArgs;
	/* callOnSocketCreated omitted. Function has to be synchronous */
};

} /* namespace Pdraw */

#endif /* !_PDRAW_SESSION_HPP_ */
