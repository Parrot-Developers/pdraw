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

#define ULOG_TAG pdraw_recmux_isobmff
#include <ulog.h>

#include "pdraw_muxer_record_isobmff.hpp"
#include "pdraw_muxer_record_isobmff_media.hpp"
#include "pdraw_session.hpp"

#include <array>
#include <memory>
#include <time.h>

#include <futils/futils.h>
#include <libmp4.h>
#include <media-buffers/mbuf_coded_video_frame.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


static const std::array<vdef_coded_format, 2> &getSupportedCodedFormats()
{
	static const std::array<vdef_coded_format, 2> formats = {{
		vdef_h264_avcc,
		vdef_h265_hvcc,
	}};
	return formats;
}

static const std::array<vdef_raw_format, 2> &getSupportedRawFormats()
{
	static const std::array<vdef_raw_format, 2> formats = {{
		vdef_raw8,
		vdef_raw16,
	}};
	return formats;
}

static const std::array<adef_format, 8> &getSupportedAudioFormats()
{
	static const std::array<adef_format, 8> formats = {{
		adef_pcm_16b_44100hz_mono,
		adef_pcm_16b_44100hz_stereo,
		adef_pcm_16b_48000hz_mono,
		adef_pcm_16b_48000hz_stereo,
		adef_aac_lc_16b_44100hz_mono_raw,
		adef_aac_lc_16b_44100hz_stereo_raw,
		adef_aac_lc_16b_48000hz_mono_raw,
		adef_aac_lc_16b_48000hz_stereo_raw,
	}};
	return formats;
}


IsobmffRecordMuxer::IsobmffRecordMuxer(
	Session *session,
	Element::Listener *elementListener,
	IPdraw::IMuxer::Listener *listener,
	MuxerWrapper *wrapper,
	const std::string &fileName,
	const struct pdraw_muxer_params *params) :
		RecordMuxer(session,
			    elementListener,
			    listener,
			    wrapper,
			    fileName,
			    params),
		mTablesSizeMb(params->tables_size_mb
				      ? params->tables_size_mb
				      : MP4_MUX_DEFAULT_TABLE_SIZE_MB),
		mTablesSyncPeriodMs(params->tables_sync_period_ms)
{
	Element::setClassName(__func__);

	mRecovery.mTablesFile = params->recovery.tables_file == nullptr
					? ""
					: params->recovery.tables_file;
	mRecovery.mSyncPeriodMs = params->recovery.sync_period_ms;
	mRecovery.checkStorageUuid = params->recovery.check_storage_uuid;
	mRecovery.allocateSpaceForTablesFile =
		params->recovery.allocate_space_for_tables_file;
	mRecovery.enabled = !mRecovery.mTablesFile.empty();

	setCodedVideoMediaFormatCaps(
		getSupportedCodedFormats().data(),
		static_cast<int>(getSupportedCodedFormats().size()));
	setRawVideoMediaFormatCaps(
		getSupportedRawFormats().data(),
		static_cast<int>(getSupportedRawFormats().size()));
	setAudioMediaFormatCaps(
		getSupportedAudioFormats().data(),
		static_cast<int>(getSupportedAudioFormats().size()));

	mStats.type = PDRAW_MUXER_TYPE_RECORD;

	mThreadSyncTimerHandle.set([this] { syncCb(); });
	mThreadTablesSyncTimerHandle.set([this] { tablesSyncCb(); });
}


/* Must be called on the loop thread */
int IsobmffRecordMuxer::addChapter(uint64_t timestamp, const char *name)
{
	int ret;
	PDRAW_CHECK_WRITER_THREAD(false);

	ULOG_ERRNO_RETURN_ERR_IF(name == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(name[0] == '\0', EINVAL);

	std::string nameStr(name);

	ret = postTask(CmdType::ADD_CHAPTER, [this, timestamp, nameStr]() {
		int err = this->internalAddChapter(timestamp, nameStr.c_str());
		if (err < 0)
			PDRAW_LOG_ERRNO("internalAddChapter", -err);
	});
	if (ret < 0) {
		PDRAW_LOG_ERRNO("postTask", -ret);
		return ret;
	}

	return 0;
}


/* Called on the loop thread */
int IsobmffRecordMuxer::getDynParams(struct pdraw_muxer_dyn_params *dyn_params)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	PDRAW_CHECK_WRITER_THREAD(false);

	dyn_params->tables_sync_period_ms = mTablesSyncPeriodMs;

	return 0;
}


/* Called on the loop thread */
int IsobmffRecordMuxer::forceSync()
{
	int ret;

	PDRAW_CHECK_WRITER_THREAD(false);

	if (!canPostTask(true)) {
		ret = -EPROTO;
		PDRAW_LOGE("%s: cannot send command to thread", __func__);
		return ret;
	}

	ret = postTask(CmdType::FORCE_SYNC, [this]() {
		int err = this->internalForceSync();
		if (err < 0)
			PDRAW_LOG_ERRNO("internalForceSync", -err);
	});
	if (ret < 0) {
		PDRAW_LOG_ERRNO("postTask", -ret);
		return ret;
	}

	return 0;
}


std::string IsobmffRecordMuxer::getDefaultMediaName(Media::Type type)
{
	switch (type) {
	case Media::Type::CODED_VIDEO:
		return "DefaultVideo";
	case Media::Type::RAW_VIDEO:
		return "RawVideo";
	case Media::Type::AUDIO:
		return "DefaultAudio";
	default:
		return "Track";
	}
}


/* Called on the writer thread */
int IsobmffRecordMuxer::onBeforeMediaCreation(
	MuxerMediaConfig &cfg,
	const struct pdraw_media_info *mediaInfo,
	const struct pdraw_muxer_media_params *params)
{
	if (mMux == nullptr)
		return -EAGAIN;

	PDRAW_CHECK_WRITER_THREAD(true);

	cfg.isDefault = params->is_default;
	cfg.timescale = (params->timescale > 0) ? params->timescale
						: DEFAULT_MP4_TIMESCALE;

	enum mp4_track_type mp4Type = MP4_TRACK_TYPE_METADATA;
	if (cfg.type == Media::Type::CODED_VIDEO)
		mp4Type = MP4_TRACK_TYPE_VIDEO;
	else if (cfg.type == Media::Type::AUDIO)
		mp4Type = MP4_TRACK_TYPE_AUDIO;

	struct mp4_mux_track_params trackParams = {
		.type = mp4Type,
		.name = cfg.name.c_str(),
		.enabled = cfg.isDefault,
		.in_movie = cfg.isDefault,
		.in_preview = cfg.isDefault,
		.timescale = cfg.timescale,
		.creation_time = cfg.trackTime,
		.modification_time = cfg.trackTime,
	};

	int res = mp4_mux_add_track(mMux, &trackParams);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}

	cfg.trackId = (uint32_t)res;
	return 0;
}


/* Must be called on the writer thread */
std::unique_ptr<RecordMuxer::MuxerMedia>
IsobmffRecordMuxer::IsobmffRecordMuxer::createMedia(const MuxerMediaConfig &cfg)
{
	PDRAW_CHECK_WRITER_THREAD(true);

	try {
		switch (cfg.type) {
		case Media::Type::CODED_VIDEO:
			return std::make_unique<IsobmffMuxerCodedVideoMedia>(
				this, cfg);
		case Media::Type::RAW_VIDEO:
			return std::make_unique<IsobmffMuxerRawVideoMedia>(this,
									   cfg);
		case Media::Type::AUDIO:
			return std::make_unique<IsobmffMuxerAudioMedia>(this,
									cfg);
		default:
			PDRAW_LOGE("unsupported media type: %d",
				   static_cast<int>(cfg.type));
			return nullptr;
		}
	} catch (const std::bad_alloc &) {
		PDRAW_LOGE("output media allocation failed");
		return nullptr;
	}
}


/* Must be called on the writer thread */
int IsobmffRecordMuxer::onAfterMediaCreation(MuxerMedia *track,
					     const MuxerMediaConfig &cfg)
{
	int res = 0;
	auto *m = static_cast<IsobmffMuxerMedia *>(track);

	PDRAW_CHECK_WRITER_THREAD(true);

	if (mHasChapters) {
		res = mp4_mux_add_ref_to_track(
			mMux, cfg.trackId, mChaptersTrackId);
		if (res >= 0)
			m->setChapters(mChaptersTrackId);
	} else if (!mPendingChapters.empty()) {
		addChapters();
	}

	return (res < 0) ? res : 0;
}


static enum mp4_metadata_cover_type
thumbnailTypeToCoverType(enum pdraw_muxer_thumbnail_type type)
{
	switch (type) {
	case PDRAW_MUXER_THUMBNAIL_TYPE_JPEG:
		return MP4_METADATA_COVER_TYPE_JPEG;
	case PDRAW_MUXER_THUMBNAIL_TYPE_PNG:
		return MP4_METADATA_COVER_TYPE_PNG;
	case PDRAW_MUXER_THUMBNAIL_TYPE_BMP:
		return MP4_METADATA_COVER_TYPE_BMP;
	default:
		return MP4_METADATA_COVER_TYPE_UNKNOWN;
	}
}


/* Called on the writer thread */
int IsobmffRecordMuxer::internalSetThumbnail(
	enum pdraw_muxer_thumbnail_type type,
	const uint8_t *data,
	size_t size)
{
	int ret = 0;

	PDRAW_CHECK_WRITER_THREAD(true);

	ret = mp4_mux_set_file_cover(
		mMux, thumbnailTypeToCoverType(type), data, size);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mp4_mux_set_file_cover", -ret);
	return ret;
}


/* Called on the writer thread */
int IsobmffRecordMuxer::internalSetFileMetadata(
	const struct pdraw_muxer_metadata_params *params,
	const uint8_t *data,
	size_t size)
{
	return -ENOSYS;
}


/* Called on the writer thread */
void IsobmffRecordMuxer::onInternalStopThread()
{
	int err;

	PDRAW_CHECK_WRITER_THREAD(true);

	if (mThreadSyncTimer != nullptr) {
		err = mThreadSyncTimer->clear();
		if (err < 0)
			PDRAW_LOG_ERRNO("timer::clear", -err);
	}

	/* Merge session metadata for stop */
	mergeSessionMetadata();

	/* Finalize the MP4 file */
	mIsMuxerReady = false;
	err = mp4_mux_close(mMux);
	if (err < 0)
		PDRAW_LOG_ERRNO("mp4_mux_close", -err);
	mMux = nullptr;
}


int IsobmffRecordMuxer::onWriterLoopInit()
{
	int ret = 0;

	PDRAW_CHECK_WRITER_THREAD(true);

	if (mRecovery.mSyncPeriodMs != 0) {
		try {
			mThreadSyncTimer = std::make_unique<pomp::Timer>(
				getThreadLoop(), &mThreadSyncTimerHandle);
		} catch (const std::bad_alloc &) {
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("pomp::Timer", -ret);
			return ret;
		}
		ret = mThreadSyncTimer->setPeriodic(mRecovery.mSyncPeriodMs,
						    mRecovery.mSyncPeriodMs);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("setPeriodic", -ret);
			return ret;
		}
	}

	if (mThreadTablesSyncTimer == nullptr && mTablesSyncPeriodMs != 0) {
		try {
			mThreadTablesSyncTimer = std::make_unique<pomp::Timer>(
				getThreadLoop(), &mThreadTablesSyncTimerHandle);
		} catch (const std::bad_alloc &) {
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("pomp::Timer", -ret);
			return ret;
		}
		ret = mThreadTablesSyncTimer->setPeriodic(mTablesSyncPeriodMs,
							  mTablesSyncPeriodMs);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("setPeriodic", -ret);
			return ret;
		}
	}

	return 0;
}


/* Must be called on the writer thread */
int IsobmffRecordMuxer::onWriterLoopCleanup()
{
	int err = 0;

	PDRAW_CHECK_WRITER_THREAD(true);

	if (mThreadSyncTimer != nullptr) {
		err = mThreadSyncTimer->clear();
		if (err < 0)
			PDRAW_LOG_ERRNO("timer::clear", -err);
		mThreadSyncTimer.reset();
	}

	if (mThreadTablesSyncTimer != nullptr) {
		err = mThreadTablesSyncTimer->clear();
		if (err < 0)
			PDRAW_LOG_ERRNO("timer::clear", -err);
		mThreadTablesSyncTimer.reset();
	}

	return 0;
}


/* Must be called on the loop thread */
int IsobmffRecordMuxer::onBeforeAddMuxerMedias()
{
	int res;

	PDRAW_CHECK_WRITER_THREAD(false);

	struct mp4_mux_config config = {
		.filename = mFileName.c_str(),
		.filemode = mFileMode,
		.timescale = DEFAULT_MP4_TIMESCALE,
		.creation_time = mMediaDate,
		.modification_time = mMediaDate,
		.tables_size_mbytes = static_cast<uint32_t>(mTablesSizeMb),
		.recovery =
			{
				.tables_file =
					mRecovery.mTablesFile.empty()
						? nullptr
						: mRecovery.mTablesFile.c_str(),
				.check_storage_uuid =
					mRecovery.checkStorageUuid,
				.allocate_space_for_tables_file =
					mRecovery.allocateSpaceForTablesFile,
			},
	};

	/* Create the MP4 muxer */
	res = mp4_mux_open(&config, &mMux);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_open", -res);
		return res;
	}
	mIsMuxerReady = true;

	return 0;
}


/* Called on the writer thread */
int IsobmffRecordMuxer::internalSetDynParams(
	const struct pdraw_muxer_dyn_params *dyn_params)
{
	int ret = 0;
	int err = 0;

	PDRAW_CHECK_WRITER_THREAD(true);

	if (mTablesSyncPeriodMs == dyn_params->tables_sync_period_ms)
		return 0;

	if (mThreadTablesSyncTimer != nullptr) {
		err = mThreadTablesSyncTimer->clear();
		if (err < 0)
			PDRAW_LOG_ERRNO("timer::clear", -err);
		mThreadTablesSyncTimer.reset();
	}

	mTablesSyncPeriodMs = dyn_params->tables_sync_period_ms;
	if (mTablesSyncPeriodMs != 0) {
		try {
			mThreadTablesSyncTimer = std::make_unique<pomp::Timer>(
				getThreadLoop(), &mThreadTablesSyncTimerHandle);
		} catch (const std::bad_alloc &) {
			err = -ENOMEM;
			PDRAW_LOG_ERRNO("pomp::Timer", err);
			goto out;
		}
		ret = mThreadTablesSyncTimer->setPeriodic(mTablesSyncPeriodMs,
							  mTablesSyncPeriodMs);
		if (ret < 0)
			PDRAW_LOG_ERRNO("timer::setPeriodic", -ret);
	}
out:
	return ret;
}


/* Called on the writer thread */
int IsobmffRecordMuxer::internalAddChapter(uint64_t timestamp, const char *name)
{
	int ret = 0;
	unique_c_ptr<uint8_t> buf;
	unsigned int bufLen = 0;
	int64_t dts = 0;

	PDRAW_CHECK_WRITER_THREAD(true);

	if (!mHasChapters) {
		/* Add a first chapter if missing */
		if (timestamp > 0 && mPendingChapters.empty()) {
			/* TODO: choose chapter name? */
			PDRAW_LOGW(
				"adding missing first chapter (00:00: Start)");
			mPendingChapters.try_emplace(0, "Start");
		}
		ret = addChapters();
		if (ret == -EAGAIN) {
			/* Pend chapter and return */
			mPendingChapters.try_emplace(timestamp, name);
			return 0;
		} else if (ret < 0) {
			PDRAW_LOG_ERRNO("addChapters", -ret);
			return ret;
		}
	}

	PDRAW_LOGN("add chapter at %02d:%02d.%02ds: '%s'",
		   (unsigned int)(timestamp / 1000000) / 60,
		   (unsigned int)(timestamp / 1000000) % 60,
		   (unsigned int)(timestamp / 10000) % 100,
		   name);

	{
		uint8_t *rawBuf = nullptr;
		ret = mp4_generate_chapter_sample(name, &rawBuf, &bufLen);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mp4_generate_chapter_sample", -ret);
			return ret;
		}
		buf.reset(rawBuf);
	}
	dts = mp4_convert_timescale(timestamp, 1000000, DEFAULT_MP4_TIMESCALE);
	struct mp4_mux_sample sample = {
		.buffer = buf.get(),
		.len = bufLen,
		.sync = 1,
		.dts = dts,
	};
	ret = mp4_mux_track_add_sample(mMux, mChaptersTrackId, &sample);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mp4_mux_track_add_sample", -ret);

	return ret;
}


/* Called on the writer thread */
int IsobmffRecordMuxer::internalForceSync()
{
	int ret = internalSync(true);
	if (ret < 0)
		PDRAW_LOG_ERRNO("internalSync", -ret);

	return ret;
}


/* Called on the writer thread */
int IsobmffRecordMuxer::internalSync(bool writeTables)
{
	int ret = 0;
	int err = 0;

	PDRAW_CHECK_WRITER_THREAD(true);

	if (mMux != nullptr) {
		if (mMetadataChanged) {
			mergeSessionMetadata();
			mMetadataChanged = false;
		}

		ret = mp4_mux_sync(mMux, writeTables);
		if (ret < 0) {
			if (mThreadTablesSyncTimer != nullptr) {
				err = mThreadTablesSyncTimer->clear();
				if (err < 0)
					PDRAW_LOG_ERRNO("timer::clear", -err);
				mThreadTablesSyncTimer.reset();
			}

			if (ret != -ENOBUFS || !writeTables)
				PDRAW_LOG_ERRNO("mp4_mux_sync", -ret);

			if (!writeTables)
				ret = -EPROTO;

			onUnrecoverableError(ret);
		}
	}

	return ret;
}


/* Called on the writer thread */
int IsobmffRecordMuxer::addChapters()
{
	int res;
	const IsobmffMuxerMedia *ref = nullptr;

	PDRAW_CHECK_WRITER_THREAD(true);

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mHasChapters, EALREADY);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mChaptersTrackId > 0, EALREADY);

	if (mMux == nullptr)
		return -EPROTO;

	/* Search for default coded video track */
	for (const auto &trackBase : mMedias) {
		const auto *track =
			static_cast<IsobmffMuxerMedia *>(trackBase.get());
		if (track->getMediaType() != Media::Type::CODED_VIDEO)
			continue;
		if (!track->isDefault())
			continue;
		ref = track;
	}
	if (ref == nullptr)
		return -EAGAIN;

	/* Add a new chapter track */
	std::string name = ref->getTrackName() + "::Chapters";
	struct mp4_mux_track_params params = {
		.type = MP4_TRACK_TYPE_CHAPTERS,
		.name = name.c_str(),
		.enabled = false,
		.in_movie = false,
		.in_preview = false,
		.timescale = DEFAULT_MP4_TIMESCALE,
		.creation_time = ref->getMediaTime(),
		.modification_time = ref->getMediaTime(),
	};
	res = mp4_mux_add_track(mMux, &params);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}
	mHasChapters = true;
	mChaptersTrackId = (uint32_t)res;

	for (const auto &trackBase : mMedias) {
		auto *track = static_cast<IsobmffMuxerMedia *>(trackBase.get());
		if (!track->hasChapters()) {
			/* Add track reference to every tracks */
			res = mp4_mux_add_ref_to_track(
				mMux, ref->getTrackId(), mChaptersTrackId);
			if (res < 0) {
				PDRAW_LOG_ERRNO("mp4_mux_add_ref_to_track",
						-res);
				return res;
			}
			track->setChapters(mChaptersTrackId);
		}
	}

	/* Process pending chapters if needed */
	for (const auto &[timestamp, title] : mPendingChapters) {
		int err = internalAddChapter(timestamp, title.c_str());
		if (err < 0)
			PDRAW_LOG_ERRNO("internalAddChapter", -err);
	}
	mPendingChapters.clear();

	return 0;
}


/* Called on the writer thread */
void IsobmffRecordMuxer::mergeSessionMetadata()
{
	PDRAW_CHECK_WRITER_THREAD(true);

	struct vmeta_session fileSessionMeta = {};
	std::vector<std::unique_ptr<struct vmeta_session>> tracksSessionMeta;
	std::vector<struct vmeta_session *> sessionPtrs;

	tracksSessionMeta.reserve(mMedias.size());
	sessionPtrs.reserve(mMedias.size());

	for (const auto &trackBase : mMedias) {
		auto *track = static_cast<IsobmffMuxerMedia *>(trackBase.get());
		if (!track->isVideo())
			continue;
		try {
			const struct vmeta_session *trackMeta =
				track->getSessionMeta();
			if (!trackMeta)
				continue;
			auto meta = std::make_unique<struct vmeta_session>();
			*meta = *trackMeta;
			/* Set first_frame_sample_index/capture_ts */
			if (!(track->getMediaType() ==
				      Media::Type::CODED_VIDEO &&
			      track->getFirstSampleIndex() == 0)) {
				meta->first_frame_sample_index =
					track->getFirstSampleIndex();
				meta->first_frame_capture_ts =
					track->getFirstCaptureTs();
			}

			sessionPtrs.push_back(meta.get());
			tracksSessionMeta.push_back(std::move(meta));
		} catch (const std::bad_alloc &) {
			PDRAW_LOG_ERRNO("vmeta_session allocation failed",
					ENOMEM);
			return;
		}
	}

	if (tracksSessionMeta.empty()) {
		PDRAW_LOGI("no video track found, skipping metadata merge");
		return;
	}

	int err = vmeta_session_merge_metadata(
		sessionPtrs.data(), sessionPtrs.size(), &fileSessionMeta);
	if (err < 0)
		PDRAW_LOG_ERRNO("vmeta_session_merge_metadata", -err);

	/* Write track metadata */
	size_t sessionIdx = 0;
	for (const auto &trackBase : mMedias) {
		auto *track = static_cast<IsobmffMuxerMedia *>(trackBase.get());
		if (!track->isVideo())
			continue;
		err = track->writeRecordingMetadata(sessionPtrs[sessionIdx]);
		sessionIdx++;
		if (err < 0)
			PDRAW_LOG_ERRNO("track->writeRecordingMetadata", -err);
	}

	sessionSetMediaDate(fileSessionMeta, false);

	if (fileSessionMeta.title[0] == '\0') {
		err = time_local_format(fileSessionMeta.media_date,
					fileSessionMeta.media_date_gmtoff,
					TIME_FMT_RFC1123,
					fileSessionMeta.title,
					sizeof(fileSessionMeta.title));
		if (err < 0)
			PDRAW_LOG_ERRNO("time_local_format", -err);
	}

	err = vmeta_session_recording_write(
		&fileSessionMeta, &sessionMetaWriteFileCb, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("vmeta_session_recording_write", -err);
}


/* Called on the writer thread */
void IsobmffRecordMuxer::syncCb()
{
	int err = internalSync(false);
	if (err < 0)
		PDRAW_LOG_ERRNO("internalSync", -err);
}


/* Called on the writer thread */
void IsobmffRecordMuxer::tablesSyncCb()
{
	int err = internalSync(true);
	if (err < 0)
		PDRAW_LOG_ERRNO("internalSync", -err);
}


/* Called on the writer thread */
void IsobmffRecordMuxer::sessionMetaWriteFileCb(
	[[maybe_unused]] enum vmeta_record_type type,
	const char *key,
	const char *value,
	void *userdata)
{

	int res;
	auto *self = static_cast<IsobmffRecordMuxer *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	PDRAW_CHECK_WRITER_THREAD_SELF(self, true);

	res = mp4_mux_add_file_metadata(self->mMux, key, value);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_add_file_metadata", -res);
}

} /* namespace Pdraw */
