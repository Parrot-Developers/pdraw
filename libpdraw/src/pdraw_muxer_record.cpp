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

#define ULOG_TAG pdraw_recmuxer
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_record.hpp"
#include "pdraw_session.hpp"

#include <libmp4.h>
#include <time.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <sys/statvfs.h>
#endif /* !_WIN32 */

#include <media-buffers/mbuf_coded_video_frame.h>

#include <futils/futils.h>

#if defined(__APPLE__)
#	include <TargetConditionals.h>
#endif

namespace Pdraw {


enum cmd_type {
	CMD_TYPE_ADD_TRACK,
	CMD_TYPE_ADD_QUEUE_EVENT,
	CMD_TYPE_REMOVE_QUEUE_EVENT,
	CMD_TYPE_SET_THUMBNAIL,
	CMD_TYPE_ADD_CHAPTER,
	CMD_TYPE_SET_METADATA,
	CMD_TYPE_FLUSH,
	CMD_TYPE_STOP_THREAD,
	CMD_TYPE_SET_DYN_PARAMS,
	CMD_TYPE_FORCE_SYNC,
};

struct add_track_params {
	uint32_t mediaId;
	uint64_t trackTime;
	Media::Type type;
	struct pdraw_media_info mediaInfo;
	struct pdraw_muxer_media_params *params;
};

struct cmd_msg {
	enum cmd_type type;
	union {
		struct {
			struct add_track_params params;
		} add_track;
		struct {
			Media::Type mediaType;
			void *queue;
		} add_queue_event;
		struct {
			Media::Type mediaType;
			void *queue;
		} remove_queue_event;
		struct {
			enum pdraw_muxer_thumbnail_type type;
			uint8_t *data;
			size_t size;
		} set_thumbnail;
		struct {
			uint64_t timestamp;
			const char *name;
		} add_chapter;
		struct {
			uint32_t mediaId;
			struct vmeta_session *metadata;
		} set_metadata;
		struct {
			struct pdraw_muxer_dyn_params dyn_params;
		} set_dyn_params;
	};
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_msg) <= PIPE_BUF - 1);


#define NB_SUPPORTED_RAW_FORMATS 2
#define NB_SUPPORTED_CODED_FORMATS 2
#define NB_SUPPORTED_AUDIO_FORMATS 8
static struct vdef_coded_format
	supportedCodedFormats[NB_SUPPORTED_CODED_FORMATS];
static struct vdef_raw_format supportedRawFormats[NB_SUPPORTED_RAW_FORMATS];
static struct adef_format supportedAudioFormats[NB_SUPPORTED_AUDIO_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats(void)
{
	supportedCodedFormats[0] = vdef_h264_avcc;
	supportedCodedFormats[1] = vdef_h265_hvcc;
	supportedRawFormats[0] = vdef_raw8;
	supportedRawFormats[1] = vdef_raw16;
	supportedAudioFormats[0] = adef_pcm_16b_44100hz_mono;
	supportedAudioFormats[1] = adef_pcm_16b_44100hz_stereo;
	supportedAudioFormats[2] = adef_pcm_16b_48000hz_mono;
	supportedAudioFormats[3] = adef_pcm_16b_48000hz_stereo;
	supportedAudioFormats[4] = adef_aac_lc_16b_44100hz_mono_raw;
	supportedAudioFormats[5] = adef_aac_lc_16b_44100hz_stereo_raw;
	supportedAudioFormats[6] = adef_aac_lc_16b_48000hz_mono_raw;
	supportedAudioFormats[7] = adef_aac_lc_16b_48000hz_stereo_raw;
}


struct mbox_mux_loop {
	struct pomp_loop *loop;
	RecordMuxer *recordMuxer;
};


RecordMuxer::RecordMuxer(Session *session,
			 Element::Listener *elementListener,
			 IPdraw::IMuxer::Listener *listener,
			 MuxerWrapper *wrapper,
			 const std::string &fileName,
			 const struct pdraw_muxer_params *params) :
		Muxer(session, elementListener, listener, wrapper, params),
		mFileName(fileName), mFileMode(0), mMux(nullptr), mMediaDate(0),
		mMediaDateGmtOff(0), mHasChaptersTrack(false),
		mChaptersTrackId(0), mMetaBuffer(nullptr), mStats({}),
		mFreeSpaceLeft(0), mPendingStop(false), mTimerSync(nullptr),
		mTablesSyncPeriodMs(0), mTimerTablesSync(nullptr)
{
	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);

	Element::setClassName(__func__);

	mWriterThread.thread = 0;
	mWriterThread.started = false;
	mWriterThread.running = false;
	mWriterThread.shouldStop = false;
	mWriterThread.loop = nullptr;

	mWriterThread.mbox = mbox_new(sizeof(cmd_msg));
	if (mWriterThread.mbox == nullptr)
		PDRAW_LOGE("mbox_new");

	mFileMode = params->filemode;

	mTablesSizeMb = params->tables_size_mb ? params->tables_size_mb
					       : MP4_MUX_DEFAULT_TABLE_SIZE_MB;

	mRecovery.mLinkFile = params->recovery.link_file == nullptr
				      ? ""
				      : params->recovery.link_file;
	mRecovery.mTablesFile = params->recovery.tables_file == nullptr
					? ""
					: params->recovery.tables_file;
	mRecovery.mSyncPeriodMs = params->recovery.sync_period_ms;
	mRecovery.checkStorageUuid = params->recovery.check_storage_uuid;
	mRecovery.enabled =
		!mRecovery.mLinkFile.empty() && !mRecovery.mTablesFile.empty();

	setCodedVideoMediaFormatCaps(supportedCodedFormats,
				     NB_SUPPORTED_CODED_FORMATS);
	setRawVideoMediaFormatCaps(supportedRawFormats,
				   NB_SUPPORTED_RAW_FORMATS);
	setAudioMediaFormatCaps(supportedAudioFormats,
				NB_SUPPORTED_AUDIO_FORMATS);

	mTablesSyncPeriodMs = params->tables_sync_period_ms;
	mRecovery.metadataChanged = true;
}


RecordMuxer::~RecordMuxer(void)
{
	int err;

	err = internalStop();
	if (err < 0)
		PDRAW_LOG_ERRNO("internalStop", -err);

	if (mWriterThread.started) {
		mWriterThread.shouldStop = true;
		if (mWriterThread.running) {
			err = pomp_loop_wakeup(mWriterThread.loop);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_loop_wakeup", -err);
		}
		err = pthread_join(mWriterThread.thread, nullptr);
		if (err < 0)
			PDRAW_LOG_ERRNO("pthread_join", -err);
	}

	if (mWriterThread.mbox != nullptr) {
		mbox_destroy(mWriterThread.mbox);
		mWriterThread.mbox = nullptr;
	}

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);
}


/* Called on the loop thread */
int RecordMuxer::addTrackForMedia(Media *media,
				  const struct pdraw_muxer_media_params *params)
{
	int ret;
	struct cmd_msg *cmd = nullptr;
	CodedVideoMedia *codedMedia = dynamic_cast<CodedVideoMedia *>(media);
	RawVideoMedia *rawMedia = dynamic_cast<RawVideoMedia *>(media);
	AudioMedia *audioMedia = dynamic_cast<AudioMedia *>(media);

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		goto out;
	}
	cmd->type = CMD_TYPE_ADD_TRACK;
	cmd->add_track.params.mediaId = media->id;
	cmd->add_track.params.trackTime = 0;
	media->fillMediaInfo(&cmd->add_track.params.mediaInfo);

	if (codedMedia != nullptr) {
		cmd->add_track.params.type = Media::Type::CODED_VIDEO;
	} else if (rawMedia != nullptr) {
		cmd->add_track.params.type = Media::Type::RAW_VIDEO;
	} else if (audioMedia != nullptr) {
		cmd->add_track.params.type = Media::Type::AUDIO;
	} else {
		ret = -EINVAL;
		goto out;
	}

	if (params != nullptr) {
		/* Deep copy of the parameters */
		cmd->add_track.params.params =
			(struct pdraw_muxer_media_params *)calloc(
				1, sizeof(struct pdraw_muxer_media_params));
		if (cmd->add_track.params.params == nullptr) {
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("calloc", -ret);
			goto out;
		}

		*cmd->add_track.params.params = *params;
		if (params->track_name != nullptr) {
			cmd->add_track.params.params->track_name =
				strdup(params->track_name);
			if (cmd->add_track.params.params->track_name ==
			    nullptr) {
				ret = -ENOMEM;
				PDRAW_LOG_ERRNO("strdup", -ret);
				goto out;
			}
		}
	}

	ret = mbox_push(mWriterThread.mbox, cmd);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -ret);
		goto out;
	}

	ret = 0;

out:
	if ((ret != 0) && (cmd != nullptr) &&
	    (cmd->add_track.params.params != nullptr)) {
		free((void *)cmd->add_track.params.params->track_name);
		free(cmd->add_track.params.params);
	}
	free(cmd);

	return ret;
}


/* Must be called on the loop thread */
int RecordMuxer::addInputMedia(Media *media,
			       const struct pdraw_muxer_media_params *params)
{
	int ret, err;
	struct cmd_msg *cmd = nullptr;
	CodedVideoChannel *codedChannel = nullptr;
	RawVideoChannel *rawChannel = nullptr;
	AudioChannel *audioChannel = nullptr;
	struct mbuf_coded_video_frame_queue *codedQueue = nullptr;
	struct mbuf_raw_video_frame_queue *rawQueue = nullptr;
	struct mbuf_audio_frame_queue *audioQueue = nullptr;

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	if (mWriterThread.mbox == nullptr) {
		PDRAW_LOGE("%s: mbox wasn't created", __func__);
		return -EPROTO;
	}

	if (media == nullptr) {
		PDRAW_LOGE("%s: unsupported input media", __func__);
		return -EINVAL;
	}

	Sink::lock();

	ret = Sink::addInputMedia(media);
	if (ret == -EEXIST) {
		Sink::unlock();
		PDRAW_LOGE("Sink::addInputMedia");
		return ret;
	} else if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("Sink::addInputMedia", -ret);
		return ret;
	}

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		goto error;
	}

	cmd->type = CMD_TYPE_ADD_QUEUE_EVENT;

	codedChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	rawChannel = dynamic_cast<RawVideoChannel *>(getInputChannel(media));
	audioChannel = dynamic_cast<AudioChannel *>(getInputChannel(media));
	if (codedChannel != nullptr) {
		ret = mbuf_coded_video_frame_queue_new(&codedQueue);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_new",
					-ret);
			goto error;
		}
		cmd->add_queue_event.mediaType = Media::Type::CODED_VIDEO;
		cmd->add_queue_event.queue = codedQueue;
		codedChannel->setQueue(this, codedQueue);
	} else if (rawChannel != nullptr) {
		ret = mbuf_raw_video_frame_queue_new(&rawQueue);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_new", -ret);
			goto error;
		}
		cmd->add_queue_event.mediaType = Media::Type::RAW_VIDEO;
		cmd->add_queue_event.queue = rawQueue;
		rawChannel->setQueue(this, rawQueue);
	} else if (audioChannel != nullptr) {
		ret = mbuf_audio_frame_queue_new(&audioQueue);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_new", -ret);
			goto error;
		}
		cmd->add_queue_event.mediaType = Media::Type::AUDIO;
		cmd->add_queue_event.queue = audioQueue;
		audioChannel->setQueue(this, audioQueue);
	} else {
		ret = -EINVAL;
		goto error;
	}

	ret = mbox_push(mWriterThread.mbox, cmd);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -ret);
		goto error;
	}

	ret = addTrackForMedia(media, params);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("addTrackForMedia", -ret);
		goto error;
	}
	free(cmd);

	Sink::unlock();

	return 0;

error:
	free(cmd);

	if (codedChannel)
		codedChannel->setQueue(this, nullptr);
	if (rawChannel)
		rawChannel->setQueue(this, nullptr);
	if (audioChannel)
		audioChannel->setQueue(this, nullptr);
	if (codedQueue) {
		err = mbuf_coded_video_frame_queue_destroy(codedQueue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_destroy",
					-err);
	}
	if (rawQueue) {
		err = mbuf_raw_video_frame_queue_destroy(rawQueue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_destroy",
					-err);
	}
	if (audioQueue) {
		err = mbuf_audio_frame_queue_destroy(audioQueue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_destroy", -err);
	}
	removeInputMedia(media);

	Sink::unlock();

	return ret;
}


/* Must be called on the loop thread */
int RecordMuxer::removeInputMedia(Media *media)
{
	int res;
	Sink::lock();
	struct cmd_msg *cmd = nullptr;
	CodedVideoChannel *codedChannel = nullptr;
	struct mbuf_coded_video_frame_queue *codedQueue = nullptr;
	RawVideoChannel *rawChannel = nullptr;
	struct mbuf_raw_video_frame_queue *rawQueue = nullptr;
	AudioChannel *audioChannel = nullptr;
	struct mbuf_audio_frame_queue *audioQueue = nullptr;

	if (!mWriterThread.running)
		goto remove;

	if (mWriterThread.mbox == nullptr) {
		PDRAW_LOGE("%s: mbox wasn't created", __func__);
		res = -EPROTO;
		goto out;
	}

	codedChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	rawChannel = dynamic_cast<RawVideoChannel *>(getInputChannel(media));
	audioChannel = dynamic_cast<AudioChannel *>(getInputChannel(media));

	if (codedChannel != nullptr)
		codedQueue = codedChannel->getQueue(this);
	else if (rawChannel != nullptr)
		rawQueue = rawChannel->getQueue(this);
	else if (audioChannel != nullptr)
		audioQueue = audioChannel->getQueue(this);
	else {
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		goto out;
	}

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -res);
		goto out;
	}

	cmd->type = CMD_TYPE_REMOVE_QUEUE_EVENT;

	if (codedQueue) {
		cmd->remove_queue_event.mediaType = Media::Type::CODED_VIDEO;
		cmd->remove_queue_event.queue = codedQueue;
	} else if (rawQueue) {
		cmd->remove_queue_event.mediaType = Media::Type::RAW_VIDEO;
		cmd->remove_queue_event.queue = rawQueue;
	} else {
		cmd->remove_queue_event.mediaType = Media::Type::AUDIO;
		cmd->remove_queue_event.queue = audioQueue;
	}

	res = mbox_push(mWriterThread.mbox, cmd);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -res);
		goto out;
	}

	if (codedChannel)
		codedChannel->setQueue(this, nullptr);
	else if (rawChannel)
		rawChannel->setQueue(this, nullptr);
	else
		audioChannel->setQueue(this, nullptr);

remove:
	res = Sink::removeInputMedia(media);
	if (res < 0) {
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -res);
		goto out;
	}

out:
	Sink::unlock();
	free(cmd);
	return res;
}


/* Called on the loop thread */
int RecordMuxer::setDynParams(const struct pdraw_muxer_dyn_params *dyn_params)
{
	int ret = 0;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	struct cmd_msg *cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		goto out;
	}
	cmd->set_dyn_params.dyn_params = *dyn_params;
	cmd->type = CMD_TYPE_SET_DYN_PARAMS;
	ret = mbox_push(mWriterThread.mbox, cmd);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -ret);
		goto error_next;
	}

out:
	free(cmd);
	return ret;

error_next:
	free(cmd);
	return ret;
}


/* Called on the loop thread */
int RecordMuxer::getDynParams(struct pdraw_muxer_dyn_params *dyn_params)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	dyn_params->tables_sync_period_ms = mTablesSyncPeriodMs;

	return 0;
}


/* Called on the loop thread */
int RecordMuxer::forceSync(void)
{
	int ret = 0;

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	if (mMux == nullptr)
		return -EPROTO;

	struct cmd_msg *cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		goto out;
	}
	cmd->type = CMD_TYPE_FORCE_SYNC;
	ret = mbox_push(mWriterThread.mbox, cmd);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -ret);
		goto out;
	}

out:
	free(cmd);
	return ret;
}


/* Called on the writer thread */
int RecordMuxer::internalSync(bool writeTables)
{
	int ret = 0;
	int err = 0;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	if (mMux != nullptr) {
		if (mRecovery.metadataChanged) {
			mergeSessionMetadata();
			mRecovery.metadataChanged = false;
		}

		ret = mp4_mux_sync(mMux, writeTables);
		if (ret < 0) {
			if (mTimerTablesSync != nullptr) {
				err = pomp_timer_clear(mTimerTablesSync);
				if (err < 0)
					PDRAW_LOG_ERRNO("pomp_timer_clear",
							-err);
				err = pomp_timer_destroy(mTimerTablesSync);
				if (err < 0)
					PDRAW_LOG_ERRNO("pomp_timer_destroy",
							-err);
				mTimerTablesSync = nullptr;
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
void RecordMuxer::syncCb(struct pomp_timer *timer, void *userdata)
{
	RecordMuxer *self = reinterpret_cast<RecordMuxer *>(userdata);
	int err = self->internalSync(false);
	if (err < 0)
		PDRAW_LOG_ERRNO("internalSync", -err);
}


/* Called on the writer thread */
void RecordMuxer::tablesSyncCb(struct pomp_timer *timer, void *userdata)
{
	RecordMuxer *self = reinterpret_cast<RecordMuxer *>(userdata);
	int err = self->internalSync(true);
	if (err < 0)
		PDRAW_LOG_ERRNO("internalSync", -err);
}


void *RecordMuxer::writerThread(void *arg)
{
	RecordMuxer *self = (RecordMuxer *)arg;
	int err = 0;
	struct mbox_mux_loop muxArg = {nullptr, nullptr};
	size_t inputMediaCount;

#if defined(__APPLE__)
#	if !TARGET_OS_IPHONE
	err = pthread_setname_np("pdraw_recordmx");
	if (err != 0)
		PDRAW_LOG_ERRNO("pthread_setname_np", err);
#	endif
#else
	err = pthread_setname_np(pthread_self(), "pdraw_recordmx");
	if (err != 0)
		PDRAW_LOG_ERRNO("pthread_setname_np", err);
#endif

	if (self->mWriterThread.mbox == nullptr) {
		PDRAW_LOGE("No mbox");
		goto out;
	}

	self->mWriterThread.loop = pomp_loop_new();
	if (self->mWriterThread.loop == nullptr) {
		PDRAW_LOG_ERRNO("pomp_loop_new", ENOMEM);
		goto out;
	}

	/* Create the metadata buffer */
	self->mMetaBuffer = (uint8_t *)malloc(VMETA_FRAME_MAX_SIZE);
	if (self->mMetaBuffer == nullptr) {
		PDRAW_LOG_ERRNO("malloc", ENOMEM);
		goto out;
	}

	muxArg.loop = self->mWriterThread.loop;
	muxArg.recordMuxer = self;

	err = pomp_loop_add(self->mWriterThread.loop,
			    mbox_get_read_fd(self->mWriterThread.mbox),
			    POMP_FD_EVENT_IN,
			    &mboxCb,
			    &muxArg);
	if (err < 0) {
		PDRAW_LOG_ERRNO("pomp_loop_add", -err);
		goto out;
	}

	if (self->mRecovery.mSyncPeriodMs != 0) {
		self->mTimerSync =
			pomp_timer_new(self->mWriterThread.loop, syncCb, self);
		if (self->mTimerSync == nullptr) {
			PDRAW_LOG_ERRNO("pomp_timer_new", ENOMEM);
			goto out;
		}

		err = pomp_timer_set_periodic(self->mTimerSync,
					      self->mRecovery.mSyncPeriodMs,
					      self->mRecovery.mSyncPeriodMs);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set_periodic", -err);
	}

	if (self->mTimerTablesSync == nullptr &&
	    self->mTablesSyncPeriodMs != 0) {
		self->mTimerTablesSync = pomp_timer_new(
			self->mWriterThread.loop, tablesSyncCb, self);
		if (self->mTimerTablesSync == nullptr) {
			PDRAW_LOG_ERRNO("pomp_timer_new", ENOMEM);
			goto out;
		}
		err = pomp_timer_set_periodic(self->mTimerTablesSync,
					      self->mTablesSyncPeriodMs,
					      self->mTablesSyncPeriodMs);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set_periodic", -err);
	}

	self->mWriterThread.running = true;

	while (!self->mWriterThread.shouldStop) {
		err = pomp_loop_wait_and_process(self->mWriterThread.loop,
						 1000);
		if (err < 0 && err != -ETIMEDOUT)
			PDRAW_LOG_ERRNO("pomp_loop_wait_and_process", -err);
	}

	self->mWriterThread.running = false;

	self->Sink::lock();

	inputMediaCount = self->getInputMediaCount();
	for (size_t i = 0; i < inputMediaCount; i++) {
		Media *media = dynamic_cast<Media *>(self->getInputMedia(i));
		if (media == nullptr) {
			PDRAW_LOGE("getInputMedia");
			continue;
		}
		CodedVideoChannel *codedChannel =
			dynamic_cast<CodedVideoChannel *>(
				self->getInputChannel(media));
		RawVideoChannel *rawChannel = dynamic_cast<RawVideoChannel *>(
			self->getInputChannel(media));
		AudioChannel *audioChannel = dynamic_cast<AudioChannel *>(
			self->getInputChannel(media));
		if (codedChannel != nullptr) {
			struct mbuf_coded_video_frame_queue *queue =
				codedChannel->getQueue(self);
			codedChannel->setQueue(self, nullptr);
			err = self->internalRemoveQueueEvtFromLoop(
				Media::Type::CODED_VIDEO, queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"internalRemoveQueueEvtFromLoop", -err);
		} else if (rawChannel != nullptr) {
			struct mbuf_raw_video_frame_queue *queue =
				rawChannel->getQueue(self);
			rawChannel->setQueue(self, nullptr);
			err = self->internalRemoveQueueEvtFromLoop(
				Media::Type::RAW_VIDEO, queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"internalRemoveQueueEvtFromLoop", -err);
		} else if (audioChannel != nullptr) {
			struct mbuf_audio_frame_queue *queue =
				audioChannel->getQueue(self);
			audioChannel->setQueue(self, nullptr);
			err = self->internalRemoveQueueEvtFromLoop(
				Media::Type::AUDIO, queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"internalRemoveQueueEvtFromLoop", -err);
		}
	}

	self->Sink::unlock();

out:

	if (self->mTimerSync != nullptr) {
		err = pomp_timer_clear(self->mTimerSync);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(self->mTimerSync);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		self->mTimerSync = nullptr;
	}

	if (self->mTimerTablesSync != nullptr) {
		err = pomp_timer_clear(self->mTimerTablesSync);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(self->mTimerTablesSync);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		self->mTimerTablesSync = nullptr;
	}

	if (self->mWriterThread.loop != nullptr) {
		if (self->mWriterThread.mbox != nullptr) {
			err = pomp_loop_remove(
				self->mWriterThread.loop,
				mbox_get_read_fd(self->mWriterThread.mbox));
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_loop_remove", -err);
		}
		err = pomp_loop_destroy(self->mWriterThread.loop);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_destroy", -err);
		self->mWriterThread.loop = nullptr;
	}

	if (self->mState == STOPPING) {
		/* Call completeStop on the loop thread */
		err = pomp_loop_idle_add_with_cookie(self->mSession->getLoop(),
						     &callCompleteStop,
						     self,
						     self);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	}

	/* Free the metadata buffer */
	free(self->mMetaBuffer);

	return nullptr;
}


/* Must be called on the loop thread */
int RecordMuxer::internalStart(void)
{
	int res, inputMediaCount, i, err;

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	err = time_local_get(&mMediaDate, &mMediaDateGmtOff);
	if (err < 0)
		PDRAW_LOG_ERRNO("time_local_get", -err);

	/* Ensure that there is enough free space on the storage */
	res = checkFreeSpace(0);
	if (res < 0) {
		PDRAW_LOG_ERRNO("checkFreeSpace", -res);
		return res;
	}

	struct mp4_mux_config config = {
		.filename = mFileName.c_str(),
		.filemode = mFileMode,
		.timescale = DEFAULT_MP4_TIMESCALE,
		.creation_time = mMediaDate,
		.modification_time = mMediaDate,
		.tables_size_mbytes = static_cast<uint32_t>(mTablesSizeMb),
		.recovery =
			{
				.link_file =
					mRecovery.mLinkFile.empty()
						? nullptr
						: mRecovery.mLinkFile.c_str(),
				.tables_file =
					mRecovery.mTablesFile.empty()
						? nullptr
						: mRecovery.mTablesFile.c_str(),
				.check_storage_uuid =
					mRecovery.checkStorageUuid,
			},
	};

	/* Create the MP4 muxer */
	res = mp4_mux_open(&config, &mMux);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_open", -res);
		return res;
	}

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Add a track for all existing medias (async) */
	for (i = 0; i < inputMediaCount; i++) {
		Media *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		res = addTrackForMedia(media, nullptr);
		if (res < 0) {
			PDRAW_LOG_ERRNO("addTrackForMedia", -res);
			continue;
		}
	}

	Sink::unlock();

	mWriterThread.started = true;

	res = pthread_create(
		&mWriterThread.thread, nullptr, &writerThread, this);
	if (res < 0) {
		PDRAW_LOG_ERRNO("pthread_create", res);
		mWriterThread.started = false;
	}

	return res;
}


/* Must be called on the loop thread */
int RecordMuxer::internalStop(void)
{
	int ret, err;
	struct cmd_msg *cmd = nullptr;

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	/* Writer thread is not running and stop is not pending, return
	 * immediately */
	if (!mWriterThread.running && !mPendingStop)
		return 0;

	mReadyToStop = false;

	/* Stop is pending, idleCompleteStop will be called when the writer
	 * thread exits */
	if (mPendingStop)
		return 0;

	mPendingStop = true;

	/* Stop the thread */
	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		mPendingStop = false;
		return ret;
	}
	cmd->type = CMD_TYPE_STOP_THREAD;
	err = mbox_push(mWriterThread.mbox, cmd);
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -err);
		mPendingStop = false;
	}
	free(cmd);

	return 0;
}


/* Must be called on the loop thread */
int RecordMuxer::setThumbnail(enum pdraw_muxer_thumbnail_type type,
			      const uint8_t *data,
			      size_t size)
{
	int ret = 0, err;
	struct cmd_msg *cmd = nullptr;

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	ULOG_ERRNO_RETURN_ERR_IF(type == PDRAW_MUXER_THUMBNAIL_TYPE_UNKNOWN,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size == 0, EINVAL);

	if (mMux == nullptr)
		return -EPROTO;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		return ret;
	}
	cmd->type = CMD_TYPE_SET_THUMBNAIL;
	cmd->set_thumbnail.type = type;
	cmd->set_thumbnail.size = size;
	cmd->set_thumbnail.data = (uint8_t *)calloc(1, size);
	if (cmd->set_thumbnail.data == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		goto error;
	}
	memcpy(cmd->set_thumbnail.data, data, size);
	err = mbox_push(mWriterThread.mbox, cmd);
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -err);
		goto error;
	}

	free(cmd);
	return 0;

error:
	if (cmd != nullptr)
		free(cmd->set_thumbnail.data);
	free(cmd);
	return ret;
}


/* Must be called on the loop thread */
int RecordMuxer::addChapter(uint64_t timestamp, const char *name)
{
	int ret = 0, err;
	struct cmd_msg *cmd = nullptr;

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	ULOG_ERRNO_RETURN_ERR_IF(name == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(strlen(name) < 1, EINVAL);

	if (mMux == nullptr)
		return -EPROTO;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		return ret;
	}
	cmd->type = CMD_TYPE_ADD_CHAPTER;
	cmd->add_chapter.timestamp = timestamp;
	cmd->add_chapter.name = xstrdup(name);
	err = mbox_push(mWriterThread.mbox, cmd);
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -err);
		goto error;
	}

	free(cmd);
	return 0;

error:
	if (cmd != nullptr)
		free(cmd->set_thumbnail.data);
	free(cmd);
	return ret;
}


/* Called on the loop thread */
void RecordMuxer::onChannelFlush(Channel *channel)
{
	int err;
	struct cmd_msg *cmd = nullptr;

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	mAsyncFlush = (!!mWriterThread.running);

	Muxer::onChannelFlush(channel);

	if (!mAsyncFlush)
		return;

	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		err = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -err);
		return;
	}
	cmd->type = CMD_TYPE_FLUSH;
	err = mbox_push(mWriterThread.mbox, cmd);
	if (err < 0)
		PDRAW_LOG_ERRNO("mbox_push", -err);
	free(cmd);
}


/* Called on the loop thread */
void RecordMuxer::onChannelSessionMetaUpdate(Channel *channel)
{
	size_t inputMediaCount;

	if (pthread_self() == mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	Sink::onChannelSessionMetaUpdate(channel);

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();

	inputMediaCount = getInputMediaCount();
	for (size_t i = 0; i < inputMediaCount; i++) {
		struct cmd_msg *cmd = nullptr;
		int err;
		Media *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOGE("getInputMedia");
			continue;
		}
		CodedVideoMedia *codedMedia =
			dynamic_cast<CodedVideoMedia *>(media);
		RawVideoMedia *rawMedia = dynamic_cast<RawVideoMedia *>(media);
		if (codedMedia == nullptr && rawMedia == nullptr)
			continue;
		if (getInputChannel(media) != channel)
			continue;
		/* Media found */
		cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
		if (cmd == nullptr) {
			err = -ENOMEM;
			PDRAW_LOG_ERRNO("calloc", -err);
			continue;
		}
		cmd->type = CMD_TYPE_SET_METADATA;
		cmd->set_metadata.mediaId = media->id;
		cmd->set_metadata.metadata = (struct vmeta_session *)calloc(
			1, sizeof(struct vmeta_session));
		if (cmd->set_metadata.metadata == nullptr) {
			err = -ENOMEM;
			PDRAW_LOG_ERRNO("calloc", -err);
			goto error_next;
		}
		*cmd->set_metadata.metadata = codedMedia != nullptr
						      ? codedMedia->sessionMeta
						      : rawMedia->sessionMeta;
		err = mbox_push(mWriterThread.mbox, cmd);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbox_push", -err);
			goto error_next;
		}
		free(cmd);
		continue;

		/* clang-format off */
error_next:
		/* clang-format on */
		if (cmd)
			free(cmd->set_metadata.metadata);
		free(cmd);
	}

	Sink::unlock();
}


/* Called on the writer thread */
void RecordMuxer::mergeSessionMetadata(void)
{
	int err;
	struct vmeta_session fileSessionMeta = {};
	size_t i = 0;
	size_t videoTrackCount = 0;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		/* Ignoring audio tracks */
		if (it->second.mMediaType != Pdraw::Media::Type::CODED_VIDEO &&
		    it->second.mMediaType != Pdraw::Media::Type::RAW_VIDEO)
			continue;
		videoTrackCount++;
	}

	struct vmeta_session **tracksSessionMeta =
		static_cast<struct vmeta_session **>(
			calloc(videoTrackCount, sizeof(*tracksSessionMeta)));
	if (tracksSessionMeta == nullptr) {
		PDRAW_LOG_ERRNO("calloc", ENOMEM);
		return;
	}

	/* Set first_frame_sample_index/first_frame_capture_ts */
	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		/* Ignoring audio tracks */
		if (it->second.mMediaType != Pdraw::Media::Type::CODED_VIDEO &&
		    it->second.mMediaType != Pdraw::Media::Type::RAW_VIDEO)
			continue;
		/* First frame capture timestamp is always needed on raw video
		 * tracks because capture_ts is not serialized in the MP4 */
		if (it->second.mMediaType == Pdraw::Media::Type::CODED_VIDEO &&
		    it->second.mFirstSampleIndex == 0)
			continue;
		it->second.mSessionMeta.first_frame_sample_index =
			it->second.mFirstSampleIndex;
		it->second.mSessionMeta.first_frame_capture_ts =
			it->second.mFirstCaptureTs;
	}

	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		/* Ignoring audio tracks */
		if (it->second.mMediaType != Pdraw::Media::Type::CODED_VIDEO &&
		    it->second.mMediaType != Pdraw::Media::Type::RAW_VIDEO)
			continue;
		tracksSessionMeta[i] = (vmeta_session *)calloc(
			sizeof(*tracksSessionMeta[i]), 1);
		*tracksSessionMeta[i] = it->second.mSessionMeta;
		i++;
	}

	err = vmeta_session_merge_metadata(
		tracksSessionMeta, videoTrackCount, &fileSessionMeta);
	if (err < 0)
		PDRAW_LOG_ERRNO("vmeta_session_merge", -err);

	i = 0;
	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		/* Ignoring audio tracks */
		if (it->second.mMediaType != Pdraw::Media::Type::CODED_VIDEO &&
		    it->second.mMediaType != Pdraw::Media::Type::RAW_VIDEO)
			continue;
		Track &track = it->second;
		struct vmeta_session *trackSessionMeta = tracksSessionMeta[i];
		struct SessionMetaWriteTrackCbUserdata ud = {
			.muxer = this,
			.trackId = track.mTrackId,
		};
		err = vmeta_session_recording_write(
			trackSessionMeta, &sessionMetaWriteTrackCb, &ud);
		if (err < 0)
			PDRAW_LOG_ERRNO("vmeta_session_recording_write", -err);
		free(trackSessionMeta);
		i++;
	}

	if (fileSessionMeta.media_date == 0) {
		/* Use the MP4 file creation date */
		fileSessionMeta.media_date = mMediaDate;
		fileSessionMeta.media_date_gmtoff = mMediaDateGmtOff;
	}
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

	free(tracksSessionMeta);
}


/* Called on the writer thread */
int RecordMuxer::internalAddTrackForMedia(const struct add_track_params *params)
{
	int res, trackId;
	uint64_t trackTime = 0;
	struct mp4_video_decoder_config cfg = {};
	bool codedMedia, rawMedia, audioMedia, videoMedia;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(params == nullptr, EAGAIN);

	codedMedia = (params->type == Media::Type::CODED_VIDEO);
	rawMedia = (params->type == Media::Type::RAW_VIDEO);
	videoMedia = (codedMedia || rawMedia);
	audioMedia = (params->type == Media::Type::AUDIO);

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mMux == nullptr, EAGAIN);

	if (trackTime == 0)
		trackTime = time(nullptr);

	/* Add a new track */
	std::string name;
	if (params->params != nullptr &&
	    params->params->track_name != nullptr) {
		name = params->params->track_name;
	} else {
		if (codedMedia)
			name = "DefaultVideo";
		else if (rawMedia)
			name = "RawVideo";
		else if (audioMedia)
			name = "DefaultAudio";
		else
			name = "Unknown";
		/* size + 1 to start at index 1 */
		name += std::to_string(mTracks.size() + 1);
	}
	struct mp4_mux_track_params trackParams = {
		.type = codedMedia ? MP4_TRACK_TYPE_VIDEO
				   : (audioMedia ? MP4_TRACK_TYPE_AUDIO
						 : MP4_TRACK_TYPE_METADATA),
		.name = name.c_str(),
		.enabled = params->params != nullptr
				   ? params->params->is_default
				   : true,
		.in_movie = params->params != nullptr
				    ? params->params->is_default
				    : true,
		.in_preview = params->params != nullptr
				      ? params->params->is_default
				      : true,
		.timescale = (params->params == nullptr ||
			      params->params->timescale == 0)
				     ? DEFAULT_MP4_TIMESCALE
				     : params->params->timescale,
		.creation_time = trackTime,
		.modification_time = trackTime,
	};
	res = mp4_mux_add_track(mMux, &trackParams);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}
	trackId = res;

	if (codedMedia || rawMedia || audioMedia) {
		const struct vmeta_session *sessionMeta = nullptr;
		if ((codedMedia || rawMedia) &&
		    params->mediaInfo.video.session_meta) {
			sessionMeta =
				params->mediaInfo.video.session_meta; /* TODO */
		}
		auto it = mTracks.insert(
			{trackId,
			 Track(name,
			       params->type,
			       params->mediaId,
			       (uint32_t)trackId,
			       0,
			       0,
			       trackTime,
			       INT64_MAX,
			       INT64_MAX,
			       trackParams.timescale,
			       params->params != nullptr
				       ? params->params->is_default
				       : 1,
			       sessionMeta)});
		if (mHasChaptersTrack) {
			/* Add reference to chapters track */
			res = mp4_mux_add_ref_to_track(
				mMux, trackId, mChaptersTrackId);
			if (res < 0) {
				PDRAW_LOG_ERRNO("mp4_mux_add_ref_to_track",
						-res);
				return res;
			}
			it.first->second.mHasChaptersTrack = true;
			it.first->second.mChaptersTrackId = mChaptersTrackId;
		} else if (mPendingChapters.size() > 0) {
			res = addChaptersTrack();
			if (res < 0 && res != -EAGAIN) {
				PDRAW_LOG_ERRNO("addChaptersTrack", -res);
				return res;
			}
		}
	}

	if (codedMedia) {
		switch (params->mediaInfo.video.coded.format.encoding) {
		case VDEF_ENCODING_H264:
			/* Set the H.264 parameter sets */
			cfg.codec = MP4_VIDEO_CODEC_AVC;
			cfg.width = params->mediaInfo.video.coded.info
					    .resolution.width;
			cfg.height = params->mediaInfo.video.coded.info
					     .resolution.height;
			cfg.avc.c_sps = params->mediaInfo.video.coded.h264.sps;
			cfg.avc.sps_size =
				params->mediaInfo.video.coded.h264.spslen;
			cfg.avc.c_pps = params->mediaInfo.video.coded.h264.pps;
			cfg.avc.pps_size =
				params->mediaInfo.video.coded.h264.ppslen;
			break;
		case VDEF_ENCODING_H265:
			/* Set the H.265 parameter sets */
			cfg.codec = MP4_VIDEO_CODEC_HEVC;
			cfg.width = params->mediaInfo.video.coded.info
					    .resolution.width;
			cfg.height = params->mediaInfo.video.coded.info
					     .resolution.height;
			cfg.hevc.c_vps = params->mediaInfo.video.coded.h265.vps;
			cfg.hevc.vps_size =
				params->mediaInfo.video.coded.h265.vpslen;
			cfg.hevc.c_sps = params->mediaInfo.video.coded.h265.sps;
			cfg.hevc.sps_size =
				params->mediaInfo.video.coded.h265.spslen;
			cfg.hevc.c_pps = params->mediaInfo.video.coded.h265.pps;
			cfg.hevc.pps_size =
				params->mediaInfo.video.coded.h265.ppslen;
			/* TODO: fill cfg.hevc.hvcc_info */
			break;
		default:
			break;
		}

		res = mp4_mux_track_set_video_decoder_config(
			mMux, trackId, &cfg);
		if (res < 0) {
			PDRAW_LOG_ERRNO(
				"mp4_mux_track_set_video_decoder_config", -res);
			return res;
		}
	} else if (rawMedia) {
		/* Raw media: add the format as MIME type */
		char *mime = nullptr, *format_str = nullptr;
		char *info_str = nullptr;

		res = vdef_raw_format_to_csv(
			&params->mediaInfo.video.raw.format, &format_str);
		if (res < 0) {
			PDRAW_LOG_ERRNO("vdef_raw_format_to_csv", -res);
			goto skip_mime;
		}
		res = vdef_format_info_to_csv(&params->mediaInfo.video.raw.info,
					      &info_str);
		if (res < 0) {
			PDRAW_LOG_ERRNO("vdef_format_info_to_csv", -res);
			goto skip_mime;
		}
		res = asprintf(&mime,
			       VDEF_RAW_MIME_TYPE ";%s;%s",
			       format_str,
			       info_str);
		if (res < 0) {
			res = -ENOMEM;
			PDRAW_LOG_ERRNO("asprintf", -res);
			goto skip_mime;
		}

		res = mp4_mux_track_set_metadata_mime_type(
			mMux, trackId, "", mime ? mime : "");
		if (res < 0) {
			PDRAW_LOG_ERRNO("mp4_mux_track_set_metadata_mime_type",
					-res);
			goto skip_mime;
		}

		/* clang-format off */
skip_mime:
		/* clang-format on */
		free(info_str);
		free(format_str);
		free(mime);
	} else if (audioMedia) {
		switch (params->mediaInfo.audio.format.encoding) {
		case ADEF_ENCODING_AAC_LC:
			break;
		default:
			res = -EINVAL;
			PDRAW_LOGE("unsupported encoding");
			return res;
		}
		res = mp4_mux_track_set_audio_specific_config(
			mMux,
			trackId,
			params->mediaInfo.audio.aac_lc.asc,
			params->mediaInfo.audio.aac_lc.asclen,
			params->mediaInfo.audio.format.channel_count,
			1024, /* TODO: dynamic? */
			params->mediaInfo.audio.format.sample_rate);
		if (res < 0) {
			PDRAW_LOG_ERRNO(
				"mp4_mux_track_set_audio_specific_config",
				-res);
			return res;
		}
	}

	return 0;
}


/* Called on the writer thread */
int RecordMuxer::addChaptersTrack(void)
{
	int res;
	Track *ref = nullptr;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mHasChaptersTrack, EALREADY);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mChaptersTrackId > 0, EALREADY);

	if (mMux == nullptr)
		return -EPROTO;

	/* Search for default coded video track */
	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		Track &track = it->second;
		if (track.mMediaType != Media::Type::CODED_VIDEO)
			continue;
		if (!track.mIsDefault)
			continue;
		ref = &track;
	}
	if (ref == nullptr)
		return -EAGAIN;

	/* Add a new chapter track */
	std::string name = ref->mName + "::Chapters";
	struct mp4_mux_track_params params = {
		.type = MP4_TRACK_TYPE_CHAPTERS,
		.name = name.c_str(),
		.enabled = false,
		.in_movie = false,
		.in_preview = false,
		.timescale = DEFAULT_MP4_TIMESCALE,
		.creation_time = ref->mTrackTime,
		.modification_time = ref->mTrackTime,
	};
	res = mp4_mux_add_track(mMux, &params);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}
	mHasChaptersTrack = true;
	mChaptersTrackId = (uint32_t)res;

	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		Track &track = it->second;
		if (!track.mHasChaptersTrack) {
			/* Add track reference to every tracks */
			res = mp4_mux_add_ref_to_track(
				mMux, ref->mTrackId, mChaptersTrackId);
			if (res < 0) {
				PDRAW_LOG_ERRNO("mp4_mux_add_ref_to_track",
						-res);
				return res;
			}
			track.mChaptersTrackId = mChaptersTrackId;
			track.mHasChaptersTrack = true;
		}
	}

	/* Process pending chapters if needed */
	for (auto it = mPendingChapters.begin(); it != mPendingChapters.end();
	     it++) {
		int err = internalAddChapter(it->first, it->second.c_str());
		if (err < 0)
			PDRAW_LOG_ERRNO("internalAddChapter", -err);
	}
	mPendingChapters.clear();

	return 0;
}


/* Called on the writer thread */
int RecordMuxer::addMetadataTrack(Track *ref, enum vmeta_frame_type metaType)
{
	int res;
	const char *mimeType = nullptr;
	const char *contentEncoding = nullptr;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(ref == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(ref->mHasMetadataTrack, EALREADY);

	switch (metaType) {
	case VMETA_FRAME_TYPE_V2:
		mimeType = VMETA_FRAME_V2_MIME_TYPE;
		contentEncoding = VMETA_FRAME_V2_CONTENT_ENCODING;
		break;
	case VMETA_FRAME_TYPE_V3:
		mimeType = VMETA_FRAME_V3_MIME_TYPE;
		contentEncoding = VMETA_FRAME_V3_CONTENT_ENCODING;
		break;
	case VMETA_FRAME_TYPE_PROTO:
		mimeType = VMETA_FRAME_PROTO_MIME_TYPE;
		contentEncoding = VMETA_FRAME_PROTO_CONTENT_ENCODING;
		break;
	default:
		PDRAW_LOGE("%s: unsupported metadata type '%s'",
			   __func__,
			   vmeta_frame_type_str(metaType));
		return -ENOSYS;
	}

	/* Add a new metadata track */
	std::string name = ref->mName + "::TimedMetadata";
	struct mp4_mux_track_params params = {
		.type = MP4_TRACK_TYPE_METADATA,
		.name = name.c_str(),
		.enabled = false,
		.in_movie = false,
		.in_preview = false,
		.timescale = ref->mTimescale,
		.creation_time = ref->mTrackTime,
		.modification_time = ref->mTrackTime,
	};
	res = mp4_mux_add_track(mMux, &params);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}
	ref->mMetaTrackId = (uint32_t)res;

	/* Set the metadata mime type */
	res = mp4_mux_track_set_metadata_mime_type(
		mMux, ref->mMetaTrackId, contentEncoding, mimeType);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_set_metadata_mime_type", -res);
		ref->mMetaTrackId = 0;
		return res;
	}

	/* Add track reference */
	res = mp4_mux_add_ref_to_track(mMux, ref->mMetaTrackId, ref->mTrackId);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_ref_to_track", -res);
		ref->mMetaTrackId = 0;
		return res;
	}

	ref->mHasMetadataTrack = true;

	/* Add an initial empty sample if needed to ensure that the newly-
	 * created metadata track is synchronized with its referenced track. */
	if ((ref->mLastSampleTs != INT64_MAX) &&
	    (ref->mFirstSampleTs != INT64_MAX) &&
	    (ref->mLastSampleTs > ref->mFirstSampleTs)) {
		const uint64_t emptyCookie = VMETA_FRAME_PROTO_EMPTY_COOKIE;
		struct mp4_mux_sample emptyMetaSample = {
			.buffer = (uint8_t *)(&emptyCookie),
			.len = sizeof(emptyCookie),
			.sync = 1,
			.dts = ref->mFirstSampleTs,
		};
		PDRAW_LOGI("%s(%s): add first metadata sample",
			   __func__,
			   ref->mName.c_str());
		res = mp4_mux_track_add_sample(
			mMux, ref->mMetaTrackId, &emptyMetaSample);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mp4_mux_track_add_sample", -res);
			return res;
		}
	}

	return 0;
}


/* Must be called on the writer thread */
int RecordMuxer::process(void)
{
	int inputMediaCount, i;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	if (mState != STARTED)
		return 0;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mMux == nullptr, EAGAIN);

	Sink::lock();
	inputMediaCount = getInputMediaCount();
	Sink::unlock();
	for (i = 0; i < inputMediaCount; i++) {
		/* Process each media by index (without locking) */
		processMedia(i);
	}

	return 0;
}


/* Called on the writer thread */
int RecordMuxer::processMedia(int index)
{
	int res = 0, err;
	uint32_t trackId = UINT32_MAX;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	Sink::lock();

	Media *media = getInputMedia(index);
	if (media == nullptr) {
		Sink::unlock();
		res = -ENOENT;
		PDRAW_LOG_ERRNO("getInputMedia", -res);
		return res;
	}

	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		Track &track = it->second;
		if (track.mMediaId == media->id) {
			trackId = track.mTrackId;
			break;
		}
	}
	if (trackId == UINT32_MAX) {
		Sink::unlock();
		res = -ENOENT;
		PDRAW_LOGE("track not found");
		return res;
	}

	Track &track = mTracks[trackId];

	if (track.mTrackId == 0) {
		Sink::unlock();
		res = -ENOENT;
		PDRAW_LOG_ERRNO("track->mTrackId", -res);
		return res;
	}

	CodedVideoChannel *codedChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	RawVideoChannel *rawChannel =
		dynamic_cast<RawVideoChannel *>(getInputChannel(media));
	AudioChannel *audioChannel =
		dynamic_cast<AudioChannel *>(getInputChannel(media));
	struct mbuf_coded_video_frame_queue *codedQueue =
		codedChannel ? codedChannel->getQueue(this) : nullptr;
	struct mbuf_raw_video_frame_queue *rawQueue =
		rawChannel ? rawChannel->getQueue(this) : nullptr;
	struct mbuf_audio_frame_queue *audioQueue =
		audioChannel ? audioChannel->getQueue(this) : nullptr;

	Sink::unlock();

	/* TODO: This loops drops frames if the processFrame function
	 * fails. This is a problem for most coded streams, so the
	 * current behavior will result in a buggy output file. We
	 * should instead check what the error was, and decide to
	 * either:
	 * - Retry later, which can be achieved by using queue_peek()
	 * instead of queue_pop(), or
	 * - Discard the frame, flush the queue, and ask the upstream
	 * elements for a complete resync (or mabye even terminate the
	 * record and warn the app ?) The second choice is important to
	 * have, because if an error persists, then this whole element
	 * will be stuck on the buggy frame. */
	do {
		if (codedQueue) {
			/* Process each buffer */
			struct mbuf_coded_video_frame *frame = nullptr;
			res = mbuf_coded_video_frame_queue_pop(codedQueue,
							       &frame);
			if (res == -EAGAIN)
				continue;
			else if (res < 0) {
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_pop",
					-res);
				continue;
			}
			res = processFrame(&track, frame);
			err = mbuf_coded_video_frame_unref(frame);
			if (err < 0)
				PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref",
						-err);
		} else if (rawQueue) {
			/* Process each buffer */
			struct mbuf_raw_video_frame *frame = nullptr;
			res = mbuf_raw_video_frame_queue_pop(rawQueue, &frame);
			if (res == -EAGAIN)
				continue;
			else if (res < 0) {
				PDRAW_LOG_ERRNO(
					"mbuf_raw_video_frame_queue_pop", -res);
				continue;
			}
			res = processFrame(&track, frame);
			err = mbuf_raw_video_frame_unref(frame);
			if (err < 0)
				PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref",
						-err);
		} else if (audioQueue) {
			/* Process each buffer */
			struct mbuf_audio_frame *frame = nullptr;
			res = mbuf_audio_frame_queue_pop(audioQueue, &frame);
			if (res == -EAGAIN)
				continue;
			else if (res < 0) {
				PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_pop",
						-res);
				continue;
			}
			res = processFrame(&track, frame);
			err = mbuf_audio_frame_unref(frame);
			if (err < 0)
				PDRAW_LOG_ERRNO("mbuf_audio_frame_unref", -err);
		} else
			break;

	} while (res == 0);
	return res;
}


/* Called from any thread */
int RecordMuxer::checkFreeSpace(size_t spaceNeeded)
{
	size_t foundDir;
	std::string path;

	/* Bypass free space limit if required */
	if (mParams.free_space_limit == 0)
		return 0;

	foundDir = mFileName.find_last_of("/\\");
	path = (foundDir == std::string::npos) ? mFileName
					       : mFileName.substr(0, foundDir);
#ifdef _WIN32
	char volume[MAX_PATH] = "";
	ULARGE_INTEGER freeBytes = {};

	GetVolumePathNameA(path.c_str(), volume, sizeof(volume));
	GetDiskFreeSpaceExA(volume, &freeBytes, nullptr, nullptr);

	mFreeSpaceLeft = freeBytes.QuadPart;
#else
	int ret;
	struct statvfs stats = {};

	ret = statvfs(path.c_str(), &stats);
	if (ret < 0) {
		ret = -errno;
		PDRAW_LOG_ERRNO("statvfs", -ret);
		return ret;
	}
	mFreeSpaceLeft = stats.f_bavail * stats.f_bsize;
#endif

	if (mFreeSpaceLeft < (mParams.free_space_limit + spaceNeeded)) {
		PDRAW_LOGW(
			"free space left (%.1f MiB) "
			"below free space limit (%.1f MiB)",
			(float)mFreeSpaceLeft / 1024 / 1024,
			(float)(mParams.free_space_limit + spaceNeeded) / 1024 /
				1024);
		return -ENOSPC;
	}
	return 0;
}


/* Must be called on the loop thread */
void RecordMuxer::callCompleteStop(void *userdata)
{
	RecordMuxer *self = reinterpret_cast<RecordMuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (pthread_self() == self->mWriterThread.thread)
		PDRAW_LOGW("%s called from the writer thread", __func__);

	self->mPendingStop = false;
	idleCompleteStop(userdata);
}


/* Called on the writer thread */
int RecordMuxer::processFrame(Track *track,
			      struct mbuf_coded_video_frame *frame)
{
	int res;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	struct vmeta_frame *metadata = nullptr;
	struct vdef_coded_frame info = {};
	CodedVideoMedia::Frame *meta;
	struct mp4_mux_scattered_sample sample = {};
	const void *aData;
	const void **frameNalus = nullptr;
	size_t *frameNalusSize = nullptr;
	uint64_t spaceNeeded = 0;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	if (mUnrecoverableError)
		return -EPROTO;

	if (mMux == nullptr || !mWriterThread.running)
		return -EPROTO;

	sample.nbuffers = mbuf_coded_video_frame_get_nalu_count(frame);
	if (sample.nbuffers < 0) {
		res = sample.nbuffers;
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_nalu_count", -res);
		goto out;
	}
	frameNalus =
		(const void **)calloc(sample.nbuffers, sizeof(*sample.buffers));
	frameNalusSize = (size_t *)calloc(sample.nbuffers, sizeof(*sample.len));
	if (frameNalus == nullptr || frameNalusSize == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -res);
		goto out;
	}

	for (int i = 0; i < sample.nbuffers; i++) {
		struct vdef_nalu nalu;
		res = mbuf_coded_video_frame_get_nalu(
			frame, i, &frameNalus[i], &nalu);
		if (res < 0) {
			PDRAW_LOG_ERRNO(
				"mbuf_coded_video_frame_get_nalu(%d)", -res, i);
			goto out;
		}
		frameNalusSize[i] = nalu.size;
		spaceNeeded += nalu.size;
	}

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		goto out;
	}

	if ((track->mFirstSampleIndex == INT32_MAX) &&
	    (track->mFirstCaptureTs == INT64_MAX) &&
	    !(info.info.flags & VDEF_FRAME_FLAG_FAKE)) {
		/* First valid frame */
		track->mFirstSampleIndex = track->mSampleCount;
		track->mFirstCaptureTs = info.info.capture_timestamp;
	}

	res = mbuf_coded_video_frame_get_ancillary_data(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&ancillaryData);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_ancillary_data",
				-res);
		goto out;
	}
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, nullptr);
	meta = (CodedVideoMedia::Frame *)aData;

	/* Add a video sample to the MP4 muxer */
	sample.buffers = (const uint8_t *const *)frameNalus;
	sample.len = (const size_t *)frameNalusSize;
	sample.sync = meta->isSync;
	sample.dts = mp4_convert_timescale(
		info.info.timestamp, info.info.timescale, track->mTimescale);

	/* Decoding timestamps must be monotonic; avoid duplicate or
	 * rollback timestamps by faking the timestamp as the previous
	 * timestamp + 1 */
	if ((track->mLastSampleTs != INT64_MAX) &&
	    (sample.dts == track->mLastSampleTs)) {
		PDRAW_LOGW("duplicate timestamp (%" PRIu64 "), incrementing",
			   track->mLastSampleTs);
		sample.dts = track->mLastSampleTs + 1;
	} else if ((track->mLastSampleTs != INT64_MAX) &&
		   (sample.dts < track->mLastSampleTs)) {
		PDRAW_LOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
			   ", incrementing",
			   track->mLastSampleTs,
			   sample.dts);
		sample.dts = track->mLastSampleTs + 1;
	}
	if (track->mFirstSampleTs == INT64_MAX)
		track->mFirstSampleTs = sample.dts;
	track->mLastSampleTs = sample.dts;

	res = checkFreeSpace(spaceNeeded);
	if (res < 0) {
		if (res != -ENOSPC)
			PDRAW_LOG_ERRNO("checkFreeSpace", -res);
		onUnrecoverableError(res);
		goto out;
	}
	res = mp4_mux_track_add_scattered_sample(
		mMux, track->mTrackId, &sample);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_add_scattered_sample", -res);
		if (res == -ENOENT) {
			/* File has been removed */
			onUnrecoverableError(res);
		}
		goto out;
	} else {
		track->mSampleCount++;
		mStats.record.coded_video_frames++;
	}

	res = mbuf_coded_video_frame_get_metadata(frame, &metadata);
	if (res == -ENOENT) {
		/* No metadata, skip to the end */
		res = 0;
		goto out;
	} else if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_metadata", -res);
		goto out;
	}

	if (!track->mHasMetadataTrack) {
		int err = addMetadataTrack(track, metadata->type);
		if (err < 0)
			PDRAW_LOG_ERRNO("addMetadataTrack", -err);
	}
	if (track->mMetaTrackId) {
		/* Add a metadata sample to the MP4 muxer */
		const uint8_t *metaContent = nullptr;
		size_t metaLen = 0;
		if (metadata->type != VMETA_FRAME_TYPE_PROTO) {
			struct vmeta_buffer metaBuf;
			vmeta_buffer_set_data(
				&metaBuf, mMetaBuffer, VMETA_FRAME_MAX_SIZE, 0);
			res = vmeta_frame_write(&metaBuf, metadata);
			if (res < 0) {
				PDRAW_LOG_ERRNO("vmeta_frame_write", -res);
				goto out;
			}
			metaContent = metaBuf.data;
			metaLen = metaBuf.pos;
		} else {
			res = vmeta_frame_proto_get_buffer(
				metadata, &metaContent, &metaLen);
			if (res < 0) {
				PDRAW_LOG_ERRNO("vmeta_frame_proto_get_buffer",
						-res);
			}
		}
		if ((metaContent != nullptr) && (metaLen > 0)) {
			struct mp4_mux_sample metaSample = {
				.buffer = metaContent,
				.len = metaLen,
				.sync = 1,
				.dts = sample.dts,
			};
			res = mp4_mux_track_add_sample(
				mMux, track->mMetaTrackId, &metaSample);
			if (res < 0)
				PDRAW_LOG_ERRNO("mp4_mux_track_add_sample",
						-res);
		}
		if (metadata->type == VMETA_FRAME_TYPE_PROTO) {
			vmeta_frame_proto_release_buffer(metadata, metaContent);
		}
	}

out:
	if (metadata)
		vmeta_frame_unref(metadata);
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	if (frame && frameNalus) {
		for (int i = 0; i < sample.nbuffers; i++) {
			if (frameNalus[i] == nullptr)
				continue;
			int err = mbuf_coded_video_frame_release_nalu(
				frame, i, frameNalus[i]);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_release_nalu",
					-err);
		}
	}
	free(frameNalus);
	free(frameNalusSize);

	return res;
}


/* Called on the writer thread */
int RecordMuxer::processFrame(Track *track, struct mbuf_raw_video_frame *frame)
{
	int res = 0;
	const void *buf = nullptr;
	size_t len;
	struct vdef_raw_frame info;
	struct mp4_mux_sample sample;
	struct vmeta_frame *metadata = nullptr;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	RawVideoMedia::Frame *meta;
	const void *aData;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	if (mUnrecoverableError)
		return -EPROTO;

	if (mMux == nullptr || !mWriterThread.running)
		return -EPROTO;

	res = mbuf_raw_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -res);
		return res;
	}

	if ((track->mFirstSampleIndex == INT32_MAX) &&
	    (track->mFirstCaptureTs == INT64_MAX) &&
	    !(info.info.flags & VDEF_FRAME_FLAG_FAKE)) {
		/* First valid frame */
		track->mFirstSampleIndex = track->mSampleCount;
		track->mFirstCaptureTs = info.info.capture_timestamp;
	}

	res = mbuf_raw_video_frame_get_packed_buffer(frame, &buf, &len);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_packed_buffer", -res);
		goto out;
	}

	res = mbuf_raw_video_frame_get_ancillary_data(
		frame, PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME, &ancillaryData);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data",
				-res);
		goto out;
	}
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, nullptr);
	meta = (RawVideoMedia::Frame *)aData;

	/* Add a video sample to the MP4 muxer */
	sample.buffer = (const uint8_t *)buf;
	sample.len = len;
	sample.sync = 1;
	sample.dts = mp4_convert_timescale(
		info.info.timestamp, info.info.timescale, track->mTimescale);

	/* Decoding timestamps must be monotonic; avoid duplicate or
	 * rollback timestamps by faking the timestamp as the previous
	 * timestamp + 1 */
	if ((track->mLastSampleTs != INT64_MAX) &&
	    (sample.dts == track->mLastSampleTs)) {
		PDRAW_LOGW("duplicate timestamp (%" PRIu64 "), incrementing",
			   track->mLastSampleTs);
		sample.dts = track->mLastSampleTs + 1;
	} else if ((track->mLastSampleTs != INT64_MAX) &&
		   (sample.dts < track->mLastSampleTs)) {
		PDRAW_LOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
			   ", incrementing",
			   track->mLastSampleTs,
			   sample.dts);
		sample.dts = track->mLastSampleTs + 1;
	}
	if (track->mFirstSampleTs == INT64_MAX)
		track->mFirstSampleTs = sample.dts;
	track->mLastSampleTs = sample.dts;

	res = checkFreeSpace(len);
	if (res < 0) {
		if (res != -ENOSPC)
			PDRAW_LOG_ERRNO("checkFreeSpace", -res);
		onUnrecoverableError(res);
		goto out;
	}
	res = mp4_mux_track_add_sample(mMux, track->mTrackId, &sample);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_track_add_sample", -res);
	else {
		track->mSampleCount++;
		mStats.record.raw_video_frames++;
	}

	res = mbuf_raw_video_frame_get_metadata(frame, &metadata);
	if (res == -ENOENT) {
		/* No metadata, skip to the end */
		res = 0;
		goto out;
	} else if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -res);
		goto out;
	}

	if (!track->mHasMetadataTrack) {
		int err = addMetadataTrack(track, metadata->type);
		if (err < 0)
			PDRAW_LOG_ERRNO("addMetadataTrack", -err);
	}
	if (track->mMetaTrackId) {
		/* Add a metadata sample to the MP4 muxer */
		const uint8_t *metaContent = nullptr;
		size_t metaLen = 0;
		if (metadata->type != VMETA_FRAME_TYPE_PROTO) {
			struct vmeta_buffer metaBuf;
			vmeta_buffer_set_data(
				&metaBuf, mMetaBuffer, VMETA_FRAME_MAX_SIZE, 0);
			res = vmeta_frame_write(&metaBuf, metadata);
			if (res < 0) {
				PDRAW_LOG_ERRNO("vmeta_frame_write", -res);
				goto out;
			}
			metaContent = metaBuf.data;
			metaLen = metaBuf.pos;
		} else {
			res = vmeta_frame_proto_get_buffer(
				metadata, &metaContent, &metaLen);
			if (res < 0) {
				PDRAW_LOG_ERRNO("vmeta_frame_proto_get_buffer",
						-res);
			}
		}
		if ((metaContent != nullptr) && (metaLen > 0)) {
			struct mp4_mux_sample metaSample = {
				.buffer = metaContent,
				.len = metaLen,
				.sync = 1,
				.dts = sample.dts,
			};
			res = mp4_mux_track_add_sample(
				mMux, track->mMetaTrackId, &metaSample);
			if (res < 0) {
				PDRAW_LOG_ERRNO("mp4_mux_track_add_sample",
						-res);
			}
		}
		if (metadata->type == VMETA_FRAME_TYPE_PROTO)
			vmeta_frame_proto_release_buffer(metadata, metaContent);
	}

out:
	if (metadata)
		vmeta_frame_unref(metadata);
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	if (buf)
		mbuf_raw_video_frame_release_packed_buffer(frame, buf);

	return res;
}


/* Called on the writer thread */
int RecordMuxer::processFrame(Track *track, struct mbuf_audio_frame *frame)
{
	int res = 0;
	const void *buf = nullptr;
	size_t len;
	struct adef_frame info;
	struct mp4_mux_sample sample;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	AudioMedia::Frame *meta;
	const void *aData;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	if (mUnrecoverableError)
		return -EPROTO;

	if (mMux == nullptr || !mWriterThread.running)
		return -EPROTO;

	res = mbuf_audio_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_frame_info", -res);
		return res;
	}

	res = mbuf_audio_frame_get_buffer(frame, &buf, &len);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_buffer", -res);
		goto out;
	}

	res = mbuf_audio_frame_get_ancillary_data(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_AUDIOMEDIAFRAME,
		&ancillaryData);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_audio_frame_get_ancillary_data", -res);
		goto out;
	}
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, nullptr);
	meta = (AudioMedia::Frame *)aData;

	/* Add an audio sample to the MP4 muxer */
	sample.buffer = (const uint8_t *)buf;
	sample.len = len;
	sample.sync = 1;
	sample.dts = mp4_convert_timescale(
		info.info.timestamp, info.info.timescale, track->mTimescale);

	/* Decoding timestamps must be monotonic; avoid duplicate or
	 * rollback timestamps by faking the timestamp as the previous
	 * timestamp + 1 */
	if ((track->mLastSampleTs != INT64_MAX) &&
	    (sample.dts == track->mLastSampleTs)) {
		PDRAW_LOGW("duplicate timestamp (%" PRIu64 "), incrementing",
			   track->mLastSampleTs);
		sample.dts = track->mLastSampleTs + 1;
	} else if ((track->mLastSampleTs != INT64_MAX) &&
		   (sample.dts < track->mLastSampleTs)) {
		PDRAW_LOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
			   ", incrementing",
			   track->mLastSampleTs,
			   sample.dts);
		sample.dts = track->mLastSampleTs + 1;
	}
	if (track->mFirstSampleTs == INT64_MAX)
		track->mFirstSampleTs = sample.dts;
	track->mLastSampleTs = sample.dts;

	res = checkFreeSpace(len);
	if (res < 0) {
		if (res != -ENOSPC)
			PDRAW_LOG_ERRNO("checkFreeSpace", -res);
		onUnrecoverableError(res);
		goto out;
	}
	res = mp4_mux_track_add_sample(mMux, track->mTrackId, &sample);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_track_add_sample", -res);
	else {
		track->mSampleCount++;
		mStats.record.audio_frames++;
	}

out:
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	if (buf)
		mbuf_audio_frame_release_buffer(frame, buf);

	return res;
}


/* Called on the writer thread */
int RecordMuxer::internalSetDynParams(
	const struct pdraw_muxer_dyn_params *dyn_params)
{
	int ret = 0;
	int err = 0;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	if (mTablesSyncPeriodMs == dyn_params->tables_sync_period_ms)
		return 0;

	if (mTimerTablesSync != nullptr) {
		err = pomp_timer_clear(mTimerTablesSync);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mTimerTablesSync);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		mTimerTablesSync = nullptr;
	}

	mTablesSyncPeriodMs = dyn_params->tables_sync_period_ms;
	if (mTablesSyncPeriodMs != 0) {
		mTimerTablesSync =
			pomp_timer_new(mWriterThread.loop, tablesSyncCb, this);
		if (mTimerTablesSync == nullptr) {
			PDRAW_LOG_ERRNO("pomp_timer_new", ENOMEM);
			goto out;
		}
		ret = pomp_timer_set_periodic(mTimerTablesSync,
					      mTablesSyncPeriodMs,
					      mTablesSyncPeriodMs);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set_periodic", -ret);
	}
out:
	return ret;
}


/* Called on the writer thread */
int RecordMuxer::internalForceSync(void)
{
	int ret = internalSync(true);
	if (ret < 0)
		PDRAW_LOG_ERRNO("internalSync", -ret);

	return ret;
}


/* Called on the writer thread */
void RecordMuxer::mboxCb(int fd, uint32_t revents, void *userdata)
{
	struct mbox_mux_loop *param = (struct mbox_mux_loop *)userdata;
	RecordMuxer *self = param->recordMuxer;
	int err;
	void *message;

	if (param->loop == nullptr || self == nullptr)
		return;

	if (pthread_self() != self->mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	message = malloc(sizeof(cmd_msg));
	if (message == nullptr) {
		PDRAW_LOG_ERRNO("malloc", ENOMEM);
		return;
	}

	do {
		struct cmd_msg *msg = (struct cmd_msg *)message;

		/* Read from the mailbox */
		err = mbox_peek(self->mWriterThread.mbox, msg);
		if (err < 0) {
			if (err != -EAGAIN)
				PDRAW_LOG_ERRNO("mbox_peek", -err);
			break;
		}

		switch (msg->type) {
		case CMD_TYPE_ADD_TRACK:
			err = self->internalAddTrackForMedia(
				&msg->add_track.params);
			if (err < 0)
				PDRAW_LOG_ERRNO("internalAddTrackForMedia",
						-err);
			Media::cleanupMediaInfo(
				&msg->add_track.params.mediaInfo);
			if (msg->add_track.params.params != nullptr) {
				free((void *)msg->add_track.params.params
					     ->track_name);
				free(msg->add_track.params.params);
			}
			break;
		case CMD_TYPE_ADD_QUEUE_EVENT:
			err = self->internalAddQueueEvtToLoop(
				msg->add_queue_event.mediaType,
				msg->add_queue_event.queue);
			if (err < 0)
				PDRAW_LOG_ERRNO("internalAddQueueEvtToLoop",
						-err);
			break;
		case CMD_TYPE_REMOVE_QUEUE_EVENT:
			err = self->internalRemoveQueueEvtFromLoop(
				msg->remove_queue_event.mediaType,
				msg->remove_queue_event.queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"internalRemoveQueueEvtFromLoop", -err);
			break;
		case CMD_TYPE_SET_THUMBNAIL:
			err = self->internalSetThumbnail(
				msg->set_thumbnail.type,
				msg->set_thumbnail.data,
				msg->set_thumbnail.size);
			if (err < 0)
				PDRAW_LOG_ERRNO("internalSetThumbnail", -err);
			free(msg->set_thumbnail.data);
			break;
		case CMD_TYPE_ADD_CHAPTER:
			err = self->internalAddChapter(
				msg->add_chapter.timestamp,
				msg->add_chapter.name);
			if (err < 0)
				PDRAW_LOG_ERRNO("internalAddChapter", -err);
			free((void *)msg->add_chapter.name);
			break;
		case CMD_TYPE_SET_METADATA:
			err = self->internalSetMetadata(
				msg->set_metadata.mediaId,
				msg->set_metadata.metadata);
			if (err < 0)
				PDRAW_LOG_ERRNO("internalSetMetadata", -err);
			free(msg->set_metadata.metadata);
			break;
		case CMD_TYPE_FLUSH:
			err = self->internalFlush();
			if (err < 0)
				PDRAW_LOG_ERRNO("internalFlush", -err);
			break;
		case CMD_TYPE_STOP_THREAD:
			err = self->internalStopThread();
			if (err < 0)
				PDRAW_LOG_ERRNO("internalStopThread", -err);
			break;
		case CMD_TYPE_SET_DYN_PARAMS:
			err = self->internalSetDynParams(
				&msg->set_dyn_params.dyn_params);
			if (err < 0)
				PDRAW_LOG_ERRNO("internalSetDynParams", -err);
			break;
		case CMD_TYPE_FORCE_SYNC:
			err = self->internalForceSync();
			if (err < 0)
				PDRAW_LOG_ERRNO("internalForceSync", -err);
			break;
		default:
			PDRAW_LOGE("unknown command: %d", msg->type);
			break;
		}
	} while (err == 0);
	free(message);
}


/* Called on the writer thread */
int RecordMuxer::internalAddQueueEvtToLoop(Media::Type type, void *queue)
{
	int ret;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	switch (type) {
	case Media::Type::CODED_VIDEO: {
		struct mbuf_coded_video_frame_queue *_queue =
			reinterpret_cast<struct mbuf_coded_video_frame_queue *>(
				queue);
		ret = addQueueEvtToLoop(_queue, mWriterThread.loop);
		if (ret < 0)
			PDRAW_LOG_ERRNO("addQueueEvtToLoop", -ret);
		return ret;
	}
	case Media::Type::RAW_VIDEO: {
		struct mbuf_raw_video_frame_queue *_queue =
			reinterpret_cast<struct mbuf_raw_video_frame_queue *>(
				queue);
		ret = addQueueEvtToLoop(_queue, mWriterThread.loop);
		if (ret < 0)
			PDRAW_LOG_ERRNO("addQueueEvtToLoop", -ret);
		return ret;
	}
	case Media::Type::AUDIO: {
		struct mbuf_audio_frame_queue *_queue =
			reinterpret_cast<struct mbuf_audio_frame_queue *>(
				queue);
		ret = addQueueEvtToLoop(_queue, mWriterThread.loop);
		if (ret < 0)
			PDRAW_LOG_ERRNO("addQueueEvtToLoop", -ret);
		return ret;
	}
	default:
		return -EINVAL;
	}
}


/* Called on the writer thread */
int RecordMuxer::internalRemoveQueueEvtFromLoop(Media::Type type, void *queue)
{
	int err;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	switch (type) {
	case Media::Type::CODED_VIDEO: {
		struct mbuf_coded_video_frame_queue *_queue =
			reinterpret_cast<struct mbuf_coded_video_frame_queue *>(
				queue);
		err = removeQueueEvtFromLoop(_queue, mWriterThread.loop);
		if (err < 0)
			PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -err);
		err = mbuf_coded_video_frame_queue_flush(_queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_flush",
					-err);
		err = mbuf_coded_video_frame_queue_destroy(_queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_destroy",
					-err);
		return 0;
	}
	case Media::Type::RAW_VIDEO: {
		struct mbuf_raw_video_frame_queue *_queue =
			reinterpret_cast<struct mbuf_raw_video_frame_queue *>(
				queue);
		err = removeQueueEvtFromLoop(_queue, mWriterThread.loop);
		if (err < 0)
			PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -err);
		err = mbuf_raw_video_frame_queue_flush(_queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_flush",
					-err);
		err = mbuf_raw_video_frame_queue_destroy(_queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_destroy",
					-err);
		return 0;
	}
	case Media::Type::AUDIO: {
		struct mbuf_audio_frame_queue *_queue =
			reinterpret_cast<struct mbuf_audio_frame_queue *>(
				queue);
		err = removeQueueEvtFromLoop(_queue, mWriterThread.loop);
		if (err < 0)
			PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -err);
		err = mbuf_audio_frame_queue_flush(_queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_flush", -err);
		err = mbuf_audio_frame_queue_destroy(_queue);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_audio_frame_queue_destroy", -err);
		return 0;
	}
	default:
		return -EINVAL;
	}
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
int RecordMuxer::internalSetThumbnail(enum pdraw_muxer_thumbnail_type type,
				      const uint8_t *data,
				      size_t size)
{
	int ret = 0;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	ret = mp4_mux_set_file_cover(
		mMux, thumbnailTypeToCoverType(type), data, size);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mp4_mux_set_file_cover", -ret);
	return ret;
}


/* Called on the writer thread */
int RecordMuxer::internalAddChapter(uint64_t timestamp, const char *name)
{
	int ret = 0;
	uint8_t *buf = nullptr;
	unsigned int bufLen = 0;
	uint32_t dts = 0;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	if (!mHasChaptersTrack) {
		/* Add a first chapter if missing */
		if (timestamp > 0 && mPendingChapters.size() == 0) {
			/* TODO: choose chapter name? */
			PDRAW_LOGW(
				"adding missing first chapter (00:00: Start)");
			mPendingChapters.insert(
				std::pair<uint64_t, std::string>(
					0, std::string("Start")));
		}
		ret = addChaptersTrack();
		if (ret == -EAGAIN) {
			/* Pend chapter and return */
			mPendingChapters.insert(
				std::pair<uint64_t, std::string>(
					timestamp, std::string(name)));
			return 0;
		} else if (ret < 0) {
			PDRAW_LOG_ERRNO("addChaptersTrack", -ret);
			return ret;
		}
	}

	PDRAW_LOGN("add chapter at %02d:%02d.%02ds: '%s'",
		   (unsigned int)(timestamp / 1000000) / 60,
		   (unsigned int)(timestamp / 1000000) % 60,
		   (unsigned int)(timestamp / 10000) % 100,
		   name);

	ret = mp4_generate_chapter_sample(name, &buf, &bufLen);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mp4_generate_chapter_sample", -ret);
		return ret;
	}
	dts = mp4_convert_timescale(timestamp, 1000000, DEFAULT_MP4_TIMESCALE);
	struct mp4_mux_sample sample = {
		.buffer = (uint8_t *)buf,
		.len = bufLen,
		.sync = 1,
		.dts = dts,
	};
	ret = mp4_mux_track_add_sample(mMux, mChaptersTrackId, &sample);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mp4_mux_track_add_sample", -ret);

	free(buf);
	return ret;
}


/* Called on the writer thread */
int RecordMuxer::internalSetMetadata(uint32_t mediaId,
				     const struct vmeta_session *metadata)
{
	ULOG_ERRNO_RETURN_ERR_IF(metadata == nullptr, EINVAL);

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		Track &track = it->second;
		if (track.mMediaId == mediaId) {
			track.mSessionMeta = *metadata;
			mRecovery.metadataChanged = true;
			return 0;
		}
	}

	return -ENOENT;
}


/* Called on the writer thread */
int RecordMuxer::internalFlush(void)
{
	int ret;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	ret = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), &idleCompleteFlush, this, this);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
		return ret;
	}
	return 0;
}


/* Called on the writer thread */
int RecordMuxer::internalStopThread(void)
{
	int err;

	if (pthread_self() != mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	if (mTimerSync != nullptr) {
		err = pomp_timer_clear(mTimerSync);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
	}

	/* Merge session metadata for stop */
	mergeSessionMetadata();

	/* Finalize the MP4 file */
	err = mp4_mux_close(mMux);
	if (err < 0)
		PDRAW_LOG_ERRNO("mp4_mux_close", -err);
	mMux = nullptr;
	mWriterThread.shouldStop = true;
	return 0;
}


/* Called on the writer thread */
void RecordMuxer::sessionMetaWriteFileCb(enum vmeta_record_type type,
					 const char *key,
					 const char *value,
					 void *userdata)
{
	int res;
	RecordMuxer *self = (RecordMuxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (pthread_self() != self->mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	res = mp4_mux_add_file_metadata(self->mMux, key, value);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_add_file_metadata", -res);
}


/* Called on the writer thread */
void RecordMuxer::sessionMetaWriteTrackCb(enum vmeta_record_type type,
					  const char *key,
					  const char *value,
					  void *userdata)
{
	int res;
	struct SessionMetaWriteTrackCbUserdata *ud =
		(struct SessionMetaWriteTrackCbUserdata *)userdata;

	ULOG_ERRNO_RETURN_IF(ud == nullptr, EINVAL);

	RecordMuxer *self = ud->muxer;
	uint32_t trackId = ud->trackId;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (pthread_self() != self->mWriterThread.thread)
		PDRAW_LOGW("%s not called from the writer thread", __func__);

	res = mp4_mux_add_track_metadata(self->mMux, trackId, key, value);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_add_track_metadata", -res);
}


int RecordMuxer::getStats(struct pdraw_muxer_stats *stats)
{
	ULOG_ERRNO_RETURN_ERR_IF(stats == nullptr, EINVAL);

	*stats = mStats;

	return 0;
}

} /* namespace Pdraw */
