/**
 * Parrot Drones Audio and Video Vector library
 * Record muxer
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

#include "pdraw_muxer_record.hpp"
#include <futils/futils.h>
#include <libpomp.hpp>

#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <libmp4.h>

namespace Pdraw {

constexpr uint32_t DEFAULT_MP4_TIMESCALE = 90000;
constexpr uint32_t MAX_NALUS_PER_FRAME = 64;


class IsobmffRecordMuxer : public RecordMuxer {
public:
	IsobmffRecordMuxer(Session *session,
			   Element::Listener *elementListener,
			   IPdraw::IMuxer::Listener *listener,
			   MuxerWrapper *wrapper,
			   const std::string &fileName,
			   const struct pdraw_muxer_params *params);

	~IsobmffRecordMuxer() override = default;

	int addChapter(uint64_t timestamp, const char *name) override;

	int getDynParams(struct pdraw_muxer_dyn_params *dyn_params) override;

	int forceSync() override;

protected:
	class IsobmffMuxerMedia;
	class IsobmffMuxerCodedVideoMedia;
	class IsobmffMuxerRawVideoMedia;
	class IsobmffMuxerAudioMedia;

	std::string getDefaultMediaName(Media::Type type) override;

	int onBeforeMediaCreation(
		MuxerMediaConfig &cfg,
		const struct pdraw_media_info *mediaInfo,
		const struct pdraw_muxer_media_params *params) override;

	std::unique_ptr<RecordMuxer::MuxerMedia>
	createMedia(const MuxerMediaConfig &cfg) override;

	int onAfterMediaCreation(MuxerMedia *track,
				 const MuxerMediaConfig &cfg) override;

	int internalSetThumbnail(enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size) override;

	int internalSetFileMetadata(
		const struct pdraw_muxer_metadata_params *params,
		const uint8_t *data,
		size_t size) override;

	void onInternalStopThread() override;

	int onWriterLoopInit() override;

	int onWriterLoopCleanup() override;

	const char *getThreadName() const override
	{
		static constexpr char name[] = "pdraw_recmx_mp4";
		static_assert(sizeof(name) <= 16,
			      "Thread name is too long for pthread_setname_np");
		return name;
	}

	int onBeforeAddMuxerMedias() override;

	int internalSetDynParams(
		const struct pdraw_muxer_dyn_params *dyn_params) override;

private:
	int internalAddChapter(uint64_t timestamp, const char *name);

	int internalForceSync();

	int internalSync(bool writeTables);

	int addChapters();

	void mergeSessionMetadata();

	void syncCb();

	void tablesSyncCb();

	static void sessionMetaWriteFileCb(enum vmeta_record_type type,
					   const char *key,
					   const char *value,
					   void *userdata);

	struct mp4_mux *mMux = nullptr;
	struct {
		std::string mTablesFile{};
		uint32_t mSyncPeriodMs = 0;
		bool enabled = false;
		bool checkStorageUuid = false;
		bool allocateSpaceForTablesFile = false;
	} mRecovery;
	size_t mFreeSpaceLeft = 0;
	size_t mTablesSizeMb = 0;
	std::unique_ptr<pomp::Timer> mThreadSyncTimer;
	pomp::Timer::HandlerFunc mThreadSyncTimerHandle;
	uint32_t mTablesSyncPeriodMs = 0;
	std::unique_ptr<pomp::Timer> mThreadTablesSyncTimer;
	pomp::Timer::HandlerFunc mThreadTablesSyncTimerHandle;
	uint32_t mChaptersTrackId = 0;
	std::map<uint64_t, std::string> mPendingChapters{};
	bool mHasChapters = false;
};

} /* namespace Pdraw */
