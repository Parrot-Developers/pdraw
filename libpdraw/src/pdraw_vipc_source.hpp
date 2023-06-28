/**
 * Parrot Drones Awesome Video Viewer Library
 * Video IPC source
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

#ifndef _PDRAW_VIPC_SOURCE_HPP_
#define _PDRAW_VIPC_SOURCE_HPP_

#ifdef BUILD_LIBVIDEO_IPC

#	include "pdraw_element.hpp"

#	include <string>

#	include <inttypes.h>

#	include <media-buffers/mbuf_raw_video_frame.h>
#	include <pdraw/pdraw.hpp>
#	include <video-ipc/vipc_client.h>
#	include <video-ipc/vipc_client_cfg.h>

#	if PDRAW_VIPC_BACKEND_DMABUF
#		include <vipc_backend_dmabuf/vipc_backend_dmabuf.h>
#	endif

#	if PDRAW_VIPC_BACKEND_HISI
#		include <vipc_backend_hisi/vipc_backend_hisi.h>
#	endif

#	if PDRAW_VIPC_BACKEND_NETWORK_HISI
#		include <vipc_backend_network_hisi/vipc_backend_network_hisi.h>
#	endif

#	if PDRAW_VIPC_BACKEND_SHM
#		include <vipc_backend_shm/vipc_backend_shm.h>
#	endif

#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
#		include <vipc_backend_network_cbuf/vipc_backend_network_cbuf.h>
#	endif

namespace Pdraw {


class VipcSource : public SourceElement {
public:
	VipcSource(Session *session,
		   Element::Listener *elementListener,
		   Source::Listener *sourceListener,
		   IPdraw::IVipcSource::Listener *listener,
		   IPdraw::IVipcSource *source,
		   const struct pdraw_vipc_source_params *params);

	~VipcSource(void);

	int start(void) override;

	int stop(void) override;

	bool isReadyToPlay(void);

	bool isPaused(void);

	int play(void);

	int pause(void);

	int configure(const struct vdef_dim *resolution,
		      const struct vdef_rectf *crop);

	int setSessionMetadata(const struct vmeta_session *meta);

	int getSessionMetadata(struct vmeta_session *meta);

	IPdraw::IVipcSource *getVipcSource(void)
	{
		return mVipcSource;
	}

	IPdraw::IVipcSource::Listener *getVipcSourceListener(void)
	{
		return mVipcSourceListener;
	}

private:
	int processFrame(const struct vipc_frame *vipcFrame,
			 struct mbuf_mem *mem);

	int setupMedia(void);

	int createMedia(void);

	int destroyMedia(void);

	int teardownChannels(void);

	int flush(void);

	void completeFlush(void);

	int tryStop(void);

	void completeStop(void);

	void onChannelFlushed(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	const char *getSourceName(void);

	struct FrameCtx {
		struct vipcc_ctx *client;
		const struct vipc_frame *frame;
	};

	enum BackendType {
#	if PDRAW_VIPC_BACKEND_DMABUF
		DMABUF,
#	endif
#	if PDRAW_VIPC_BACKEND_HISI
		HISI,
#	endif
#	if PDRAW_VIPC_BACKEND_SHM
		SHM,
#	endif
#	if PDRAW_VIPC_BACKEND_NETWORK_HISI
		NETWORK_HISI,
#	endif
#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
		NETWORK_CBUF,
#	endif
		BE_COUNT,
	};

	struct Backend {
		const char *name;
		const struct vipc_be_cb *beCbs;
		const struct vipcc_cb *clientCbs;
	};

	static const struct Backend cBackends[BE_COUNT];

	static const struct Backend *
	getBackend(const char *name, bool useDefault, enum BackendType *type);

	static void statusCb(struct vipcc_ctx *ctx,
			     const struct vipc_status *status,
			     void *userdata);

	static void configureCb(struct vipcc_ctx *ctx,
				const struct vipc_configure *config,
				void *userdata);

#	if PDRAW_VIPC_BACKEND_DMABUF || PDRAW_VIPC_BACKEND_HISI ||            \
		PDRAW_VIPC_BACKEND_NETWORK_HISI || PDRAW_VIPC_BACKEND_SHM
	static void releaseFrameCb(void *data, size_t len, void *userdata);
#	endif

#	if PDRAW_VIPC_BACKEND_DMABUF
	static void dmabufFrameCb(struct vipcc_ctx *ctx,
				  const struct vipc_frame *frame,
				  void *be_frame,
				  void *userdata);
#	endif

#	if PDRAW_VIPC_BACKEND_HISI || PDRAW_VIPC_BACKEND_NETWORK_HISI
	static void hisiFrameCb(struct vipcc_ctx *ctx,
				const struct vipc_frame *frame,
				void *be_frame,
				void *userdata);
#	endif

#	if PDRAW_VIPC_BACKEND_SHM
	static void shmFrameCb(struct vipcc_ctx *ctx,
			       const struct vipc_frame *frame,
			       void *be_frame,
			       void *userdata);
#	endif

#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
	static void cbufFrameCb(struct vipcc_ctx *ctx,
				const struct vipc_frame *frame,
				void *be_frame,
				void *userdata);
#	endif

	static void connectionStatusCb(struct vipcc_ctx *ctx,
				       bool connected,
				       void *userdata);

	static void eosCb(struct vipcc_ctx *ctx,
			  enum vipc_eos_reason reason,
			  void *userdata);

	static void watchdogTimerCb(struct pomp_timer *timer, void *userdata);

#	if PDRAW_VIPC_BACKEND_DMABUF
	static const struct vipcc_cb cDmabufCbs;
#	endif

#	if PDRAW_VIPC_BACKEND_HISI || PDRAW_VIPC_BACKEND_NETWORK_HISI
	static const struct vipcc_cb cHisiCbs;
#	endif

#	if PDRAW_VIPC_BACKEND_SHM
	static const struct vipcc_cb cShmCbs;
#	endif

#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
	static const struct vipcc_cb cCbufCbs;
	struct mbuf_pool *mPool;
#	endif

	IPdraw::IVipcSource *mVipcSource;
	IPdraw::IVipcSource::Listener *mVipcSourceListener;
	struct pdraw_vipc_source_params mParams;
	std::string mAddress;
	std::string mFriendlyName;
	std::string mBackendName;
	struct vipcc_ctx *mClient;
	BackendType mBackendType;
	struct vipc_status mStatus;
	enum pdraw_vipc_source_eos_reason mLastEosReason;
	RawVideoMedia *mOutputMedia;
	bool mOutputMediaChanging;
	bool mVipcConnected;
	bool mReady;
	bool mWasReady;
	bool mRunning;
	bool mWasRunning;
	bool mFirstFrame;
	unsigned int mInputFramesCount;
	unsigned int mFrameIndex;
	uint32_t mTimescale;
	uint64_t mLastTimestamp;
	bool mFlushPending;
	struct pomp_timer *mWatchdogTimer;
};

} /* namespace Pdraw */

#endif /* BUILD_LIBVIDEO_IPC */

#endif /* !_PDRAW_VIPC_SOURCE_HPP_ */
