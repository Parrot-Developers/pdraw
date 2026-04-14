/**
 * Parrot Drones Audio and Video Vector library
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

#pragma once

#include "pdraw_element.hpp"

#include <string>

#include <inttypes.h>

#include <media-buffers/mbuf_raw_video_frame.h>
#include <pdraw/pdraw.hpp>

#ifdef BUILD_LIBVIDEO_IPC
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
#endif

namespace Pdraw {


#ifdef BUILD_LIBVIDEO_IPC

constexpr size_t VIPC_SOURCE_DEFAULT_TIMESCALE = 1000000;

class VipcSourceWrapper;


class VipcSource : public SourceElement {
public:
	VipcSource(Session *session,
		   Element::Listener *elementListener,
		   Source::Listener *sourceListener,
		   IPdraw::IVipcSource::Listener *listener,
		   VipcSourceWrapper *wrapper,
		   const struct pdraw_vipc_source_params *params);

	~VipcSource() override;

	int start() override;

	int stop() override;

	bool isReadyToPlay() const;

	bool isPaused() const;

	int play();

	int pause();

	inline int drain()
	{
		return flush(false);
	}

	int configure(const struct vdef_dim *resolution,
		      const struct vdef_rectf *crop) const;

	int insertGreyFrame(uint64_t tsUs);

	int setSessionMetadata(const struct vmeta_session *meta);

	int getSessionMetadata(struct vmeta_session *meta) const;

	IPdraw::IVipcSource *getVipcSource() const
	{
		return mVipcSource;
	}

private:
	int processFrame(const struct vipc_frame *vipcFrame,
			 struct mbuf_mem *mem);

	void incrementUsedFrameCount();

	void decrementUsedFrameCount();

	int setupMedia();

	int createMedia();

	int destroyMedia();

	int teardownChannels();

	int flush(bool discard = true);

	void completeFlush();

	int tryStop();

	void completeStop();

	void playResponse();

	void pauseResponse();

	void onChannelFlushed(Channel *channel) override;

	void onChannelDrained(Channel *channel) override;

	void onChannelUnlink(Channel *channel) override;

	const char *getSourceName() const;

	bool pushLimitReached(void) const
	{
		return (mParams->max_pushed_frame_count > 0 &&
			mPushedFrameCount >= mParams->max_pushed_frame_count);
	}

	static void idleCompleteFlush(void *userdata);

	static void idleCompleteStop(void *userdata);

	/* Vipc source listener calls from idle functions */
	static void callOnMediaAdded(void *userdata);

	static void callPlayResponse(void *userdata);

	static void callPauseResponse(void *userdata);

	struct FrameCtx {
		VipcSource *self = nullptr;
		struct vipcc_ctx *client = nullptr;
		const struct vipc_frame *frame = nullptr;
	};

	enum class BackendType : unsigned int {
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
		Backend(const char *n, const vipc_be_cb *b, const vipcc_cb *c) :
				name(n), beCbs(b), clientCbs(c)
		{
		}

		Backend() = default;

		const char *name = nullptr;
		const struct vipc_be_cb *beCbs = nullptr;
		const struct vipcc_cb *clientCbs = nullptr;
	};

	static constexpr size_t BE_COUNT =
		static_cast<size_t>(BackendType::BE_COUNT);

	static constexpr size_t toIndex(BackendType bt) noexcept
	{
		return static_cast<size_t>(bt);
	}

	static const struct Backend cBackends[BE_COUNT];

	static const struct Backend *
	getBackend(const char *name, bool useDefault, BackendType *type);

	static bool isMbufMemImplemSupported(
		const enum mbuf_mem_implem_type *supportedMemImplems,
		size_t supportedMemImplemCount,
		enum mbuf_mem_implem_type implem,
		enum BackendType type);

	static void statusCb(struct vipcc_ctx *ctx,
			     const struct vipc_status *status,
			     void *userdata);

	static void configureCb(struct vipcc_ctx *ctx,
				const struct vipc_configure *config,
				void *userdata);

#	if PDRAW_VIPC_BACKEND_DMABUF || PDRAW_VIPC_BACKEND_HISI ||            \
		PDRAW_VIPC_BACKEND_NETWORK_HISI || PDRAW_VIPC_BACKEND_SHM
	/* Can be called from any thread */
	static void releaseFrameCb(void *data, size_t len, void *userdata);

	/* Can be called from any thread */
	static void
	releaseFdFrameCb(void *data, size_t len, int fd, void *userdata);
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
	struct mbuf_pool *mPool = nullptr;
#	endif

	IPdraw::IVipcSource *mVipcSource = nullptr;
	IPdraw::IVipcSource::Listener *mVipcSourceListener = nullptr;
	std::unique_ptr<struct pdraw_vipc_source_params,
			decltype(&Pdraw::pdrawVipcSourceParamsFree)>
		mParams{nullptr, &Pdraw::pdrawVipcSourceParamsFree};
	struct vipcc_ctx *mClient = nullptr;
	BackendType mBackendType = (VipcSource::BackendType)0;
	struct vipc_status *mStatus = nullptr;
	enum pdraw_vipc_source_eos_reason mLastEosReason =
		PDRAW_VIPC_SOURCE_EOS_REASON_NONE;
	std::unique_ptr<RawVideoMedia> mOutputMedia{};
	bool mOutputMediaChanging = false;
	bool mKeepMedia = false;
	bool mVipcConnected = false;
	bool mReady = false;
	bool mWasReady = false;
	bool mRunning = false;
	bool mWasRunning = false;
	bool mFirstFrame = true;
	bool mPausePending = false;
	unsigned int mInputFramesCount = 0;
	std::atomic<unsigned int> mUsedFrameCount{0};
	unsigned int mNextFrameIndex = 0;
	uint32_t mTimescale = VIPC_SOURCE_DEFAULT_TIMESCALE;
	uint64_t mLastTimestamp = UINT64_MAX;
	struct pomp_timer *mWatchdogTimer = nullptr;
	uint32_t mPushedFrameCount = 0;
};

#endif /* BUILD_LIBVIDEO_IPC */


class VipcSourceWrapper : public IPdraw::IVipcSource, public ElementWrapper {
public:
	VipcSourceWrapper(Session *session,
			  const struct pdraw_vipc_source_params *params,
			  IPdraw::IVipcSource::Listener *listener);

	~VipcSourceWrapper() override;

	bool isReadyToPlay() override;

	bool isPaused() override;

	int play() override;

	int pause() override;

	int configure(const struct vdef_dim *resolution,
		      const struct vdef_rectf *crop) override;

	int insertGreyFrame(uint64_t tsUs) override;

	int setSessionMetadata(const struct vmeta_session *meta) override;

	int getSessionMetadata(struct vmeta_session *meta) override;

	void clearElement() override
	{
		ElementWrapper::clearElement();
#ifdef BUILD_LIBVIDEO_IPC
		mSource = nullptr;
#endif
	}

#ifdef BUILD_LIBVIDEO_IPC
	Source *getSource() const
	{
		return mSource;
	}

	VipcSource *getVipcSource() const
	{
		return mSource;
	}
#endif

private:
	bool isElementStopped() const override
	{
		return (ElementWrapper::isElementStopped()
#ifdef BUILD_LIBVIDEO_IPC
			|| mSource == nullptr
#endif
		);
	}

#ifdef BUILD_LIBVIDEO_IPC
	VipcSource *mSource = nullptr;
#endif
};

} /* namespace Pdraw */
