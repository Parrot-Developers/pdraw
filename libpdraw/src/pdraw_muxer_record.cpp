/**
 * Parrot Drones Awesome Video Viewer Library
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

namespace Pdraw {

enum cmd_type {
	CMD_TYPE_ADD_RAW_EVENT,
	CMD_TYPE_REMOVE_RAW_EVENT,
	CMD_TYPE_ADD_CODED_EVENT,
	CMD_TYPE_REMOVE_CODED_EVENT,
	CMD_TYPE_STOP_THREAD,
};
struct cmd_msg {
	enum cmd_type type;
	union {
		struct {
			struct mbuf_coded_video_frame_queue *queue;
		} add_coded_evt;
		struct {
			struct mbuf_coded_video_frame_queue *queue;
		} remove_coded_evt;
		struct {
			struct mbuf_raw_video_frame_queue *queue;
		} add_raw_evt;
		struct {
			struct mbuf_raw_video_frame_queue *queue;
		} remove_raw_evt;
	};
};
PDRAW_STATIC_ASSERT(sizeof(struct cmd_msg) <= PIPE_BUF - 1);


#define NB_SUPPORTED_RAW_FORMATS 2
#define NB_SUPPORTED_CODED_FORMATS 2
static struct vdef_coded_format
	supportedCodedFormats[NB_SUPPORTED_CODED_FORMATS];
static struct vdef_raw_format supportedRawFormats[NB_SUPPORTED_RAW_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats(void)
{
	supportedCodedFormats[0] = vdef_h264_avcc;
	supportedCodedFormats[1] = vdef_h265_hvcc;
	supportedRawFormats[0] = vdef_raw8;
	supportedRawFormats[1] = vdef_raw16;
}


struct mbox_mux_loop {
	struct pomp_loop *loop;
	RecordMuxer *recordMuxer;
};


RecordMuxer::RecordMuxer(Session *session,
			 Element::Listener *elementListener,
			 IPdraw::IMuxer::Listener *listener,
			 IPdraw::IMuxer *muxer,
			 const std::string &fileName,
			 const struct pdraw_muxer_params *params) :
		Muxer(session, elementListener, listener, muxer, params),
		mFileName(fileName), mMux(nullptr), mMediaDate(0),
		mMetaBuffer(nullptr), mFreeSpaceLeft(0), mNoSpaceLeft(false)
{
	int err = 0;

	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);

	Element::setClassName(__func__);

	err = pthread_mutex_init(&mMp4Mutex, nullptr);
	if (err != 0)
		PDRAW_LOG_ERRNO("pthread_mutex_init", err);

	mWriterThread.thread = 0;
	mWriterThread.started = false;
	mWriterThread.loop = nullptr;

	mWriterThread.mbox = mbox_new(sizeof(cmd_msg));
	if (mWriterThread.mbox == nullptr)
		PDRAW_LOGE("mbox_new");

	setCodedVideoMediaFormatCaps(supportedCodedFormats,
				     NB_SUPPORTED_CODED_FORMATS);
	setRawVideoMediaFormatCaps(supportedRawFormats,
				   NB_SUPPORTED_RAW_FORMATS);
}


RecordMuxer::~RecordMuxer(void)
{
	int err;
	struct cmd_msg *cmd = nullptr;

	err = internalStop();

	/* Stop the thread */
	cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
	if (cmd == nullptr) {
		err = -ENOMEM;
		ULOG_ERRNO("calloc", -err);
		mWriterThread.started = false;
		if (mWriterThread.loop)
			pomp_loop_wakeup(mWriterThread.loop);
		pthread_join(mWriterThread.thread, nullptr);
	} else {
		cmd->type = CMD_TYPE_STOP_THREAD;
		if (mWriterThread.started) {
			err = mbox_push(mWriterThread.mbox, cmd);
			if (err < 0)
				PDRAW_LOG_ERRNO("mbox_push", -err);
			pthread_join(mWriterThread.thread, nullptr);
		}
		free(cmd);
	}
	if (mWriterThread.mbox) {
		mbox_destroy(mWriterThread.mbox);
		mWriterThread.mbox = nullptr;
	}

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	pthread_mutex_destroy(&mMp4Mutex);
	return;
}


/* Must be called on the main loop thread */
int RecordMuxer::addInputMedia(
	Media *media,
	const struct pdraw_muxer_video_media_params *params)
{
	int res, err;
	struct cmd_msg *cmd = nullptr;
	CodedVideoChannel *codedChannel = nullptr;
	RawVideoChannel *rawChannel = nullptr;
	struct mbuf_coded_video_frame_queue *codedQueue = nullptr;
	struct mbuf_raw_video_frame_queue *rawQueue = nullptr;

	if (mWriterThread.mbox == nullptr) {
		PDRAW_LOGE("%s: mbox wasn't created", __func__);
		return -EPROTO;
	}

	if (media == nullptr) {
		PDRAW_LOGE("%s: unsupported input media", __func__);
		return -EINVAL;
	}

	res = Sink::addInputMedia(media);
	if (res == -EEXIST) {
		PDRAW_LOGE("Sink::addInputMedia");
		return res;
	} else if (res < 0) {
		PDRAW_LOG_ERRNO("Sink::addInputMedia", -res);
		goto error;
	}

	codedChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	rawChannel = dynamic_cast<RawVideoChannel *>(getInputChannel(media));
	if (codedChannel) {
		res = mbuf_coded_video_frame_queue_new(&codedQueue);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_new",
					-res);
			goto error;
		}
		codedChannel->setQueue(this, codedQueue);
		cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
		if (cmd == nullptr) {
			res = -ENOMEM;
			ULOG_ERRNO("calloc", -res);
			goto error;
		}
		cmd->type = CMD_TYPE_ADD_CODED_EVENT;
		cmd->add_coded_evt.queue = codedQueue;
	} else if (rawChannel) {
		res = mbuf_raw_video_frame_queue_new(&rawQueue);
		if (res < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_queue_new", -res);
			goto error;
		}
		rawChannel->setQueue(this, rawQueue);
		cmd = (struct cmd_msg *)calloc(1, sizeof(cmd_msg));
		if (cmd == nullptr) {
			res = -ENOMEM;
			ULOG_ERRNO("calloc", -res);
			goto error;
		}
		cmd->type = CMD_TYPE_ADD_RAW_EVENT;
		cmd->add_raw_evt.queue = rawQueue;
	} else {
		res = -ENODEV;
		PDRAW_LOG_ERRNO("Sink::getInputChannel", -res);
		goto error;
	}

	if (mMux != nullptr) {
		/* Add a track for the media if the muxer is already opened */
		res = addTrackForMedia(media, 0, params);
		if (res < 0) {
			PDRAW_LOG_ERRNO("addTrackForMedia", -res);
			goto error;
		}
	}

	res = mbox_push(mWriterThread.mbox, cmd);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -res);
		goto error;
	}
	free(cmd);

	return 0;

error:
	free(cmd);
	if (codedChannel)
		codedChannel->setQueue(this, nullptr);
	if (rawChannel)
		rawChannel->setQueue(this, nullptr);
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
	removeInputMedia(media);
	return res;
}


int RecordMuxer::removeInputMedia(Media *media)
{
	int res;
	Sink::lock();
	struct cmd_msg *cmd = nullptr;
	CodedVideoChannel *codedChannel = nullptr;
	struct mbuf_coded_video_frame_queue *codedQueue = nullptr;
	RawVideoChannel *rawChannel = nullptr;
	struct mbuf_raw_video_frame_queue *rawQueue = nullptr;

	if (mWriterThread.mbox == nullptr) {
		PDRAW_LOGE("%s: mbox wasn't created", __func__);
		res = -EPROTO;
		goto out;
	}

	codedChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	rawChannel = dynamic_cast<RawVideoChannel *>(getInputChannel(media));

	if (codedChannel != nullptr)
		codedQueue = codedChannel->getQueue(this);
	else if (rawChannel != nullptr)
		rawQueue = rawChannel->getQueue(this);
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

	if (codedQueue) {
		cmd->type = CMD_TYPE_REMOVE_CODED_EVENT;
		cmd->remove_coded_evt.queue = codedQueue;
	} else {
		cmd->type = CMD_TYPE_REMOVE_RAW_EVENT;
		cmd->remove_raw_evt.queue = rawQueue;
	}

	res = mbox_push(mWriterThread.mbox, cmd);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbox_push", -res);
		goto out;
	}

	if (codedChannel)
		codedChannel->setQueue(this, nullptr);
	else
		rawChannel->setQueue(this, nullptr);

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


void *RecordMuxer::writerThread(void *arg)
{
	RecordMuxer *self = (RecordMuxer *)arg;
	int err = 0;
	struct mbox_mux_loop muxArg = {nullptr, nullptr};
	size_t inputMediaCount;

	if (self->mWriterThread.mbox == nullptr) {
		PDRAW_LOGE("No mbox");
		goto out;
	}

	self->mWriterThread.loop = pomp_loop_new();
	if (self->mWriterThread.loop == nullptr) {
		PDRAW_LOG_ERRNO("pomp_loop_new", ENOMEM);
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

	while (self->mWriterThread.started)
		pomp_loop_wait_and_process(self->mWriterThread.loop, -1);

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
		if (codedChannel != nullptr) {
			struct mbuf_coded_video_frame_queue *queue =
				codedChannel->getQueue(self);
			err = self->removeQueueEvtFromLoop(
				queue, self->mWriterThread.loop);
			if (err < 0)
				PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -err);
			err = mbuf_coded_video_frame_queue_flush(queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_flush",
					-err);
			codedChannel->setQueue(self, nullptr);
			err = mbuf_coded_video_frame_queue_destroy(queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_destroy",
					-err);
		} else if (rawChannel != nullptr) {
			struct mbuf_raw_video_frame_queue *queue =
				rawChannel->getQueue(self);
			err = self->removeQueueEvtFromLoop(
				queue, self->mWriterThread.loop);
			if (err < 0)
				PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -err);
			err = mbuf_raw_video_frame_queue_flush(queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_raw_video_frame_queue_flush",
					-err);
			rawChannel->setQueue(self, nullptr);
			err = mbuf_raw_video_frame_queue_destroy(queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_raw_video_frame_queue_destroy",
					-err);
		}
	}
	self->Sink::unlock();

	err = pomp_loop_remove(self->mWriterThread.loop,
			       mbox_get_read_fd(self->mWriterThread.mbox));
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_remove", -err);

out:
	if (self->mWriterThread.loop) {
		pomp_loop_destroy(self->mWriterThread.loop);
		self->mWriterThread.loop = nullptr;
	}
	return nullptr;
}


int RecordMuxer::internalStart(void)
{
	int res, inputMediaCount, i;
	uint64_t now;

	mMediaDate = time(nullptr);
	now = mMediaDate;

	/* Create the metadata buffer */
	mMetaBuffer = (uint8_t *)malloc(VMETA_FRAME_MAX_SIZE);
	if (mMetaBuffer == nullptr) {
		res = -ENOMEM;
		PDRAW_LOG_ERRNO("malloc", -res);
		return res;
	}

	/* Create the MP4 muxer */
	pthread_mutex_lock(&mMp4Mutex);
	res = mp4_mux_open(
		mFileName.c_str(), DEFAULT_MP4_TIMESCALE, now, now, &mMux);
	pthread_mutex_unlock(&mMp4Mutex);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_open", -res);
		return res;
	}

	/* Ensure that there is enough free space on the storage */
	res = checkFreeSpace(0, mFreeSpaceLeft);
	if (res < 0) {
		PDRAW_LOG_ERRNO("checkFreeSpace", -res);
		return res;
	}

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Add a track for all existing medias */
	for (i = 0; i < inputMediaCount; i++) {
		Media *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		res = addTrackForMedia(media, now, nullptr);
		if (res < 0)
			continue;
	}
	res = pthread_create(
		&mWriterThread.thread, nullptr, &writerThread, this);
	if (res == 0)
		mWriterThread.started = true;
	else
		PDRAW_LOG_ERRNO("pthread_create", res);

	Sink::unlock();

	return res;
}


int RecordMuxer::internalStop(void)
{
	int err;

	if (mMux != nullptr) {
		mergeSessionMetadata();

		/* Finalize the MP4 file */
		pthread_mutex_lock(&mMp4Mutex);
		err = mp4_mux_close(mMux);
		if (err < 0)
			PDRAW_LOG_ERRNO("mp4_mux_close", -err);
		mMux = nullptr;
		pthread_mutex_unlock(&mMp4Mutex);
	}

	/* Free the metadata buffer */
	free(mMetaBuffer);
	mMetaBuffer = nullptr;

	return 0;
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


int RecordMuxer::setThumbnail(enum pdraw_muxer_thumbnail_type type,
			      const uint8_t *data,
			      size_t size)
{
	int res = 0;

	ULOG_ERRNO_RETURN_ERR_IF(type == PDRAW_MUXER_THUMBNAIL_TYPE_UNKNOWN,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size == 0, EINVAL);

	if (mMux == nullptr)
		return -EPROTO;

	pthread_mutex_lock(&mMp4Mutex);
	res = mp4_mux_set_file_cover(
		mMux, thumbnailTypeToCoverType(type), data, size);
	pthread_mutex_unlock(&mMp4Mutex);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_set_file_cover", -res);

	return res;
}


void RecordMuxer::mergeSessionMetadata(void)
{
	int err;
	struct vmeta_session fileSessionMeta = {};

	/* First pass: fill fileSessionMeta with identical values */
	bool first = true;
	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		Media *media = it->first;

		if (first) {
			fileSessionMeta = media->sessionMeta;
			first = false;
			continue;
		}

		if (strcmp(fileSessionMeta.friendly_name,
			   media->sessionMeta.friendly_name) != 0)
			fileSessionMeta.friendly_name[0] = '\0';

		if (strcmp(fileSessionMeta.maker, media->sessionMeta.maker) !=
		    0)
			fileSessionMeta.maker[0] = '\0';

		if (strcmp(fileSessionMeta.model, media->sessionMeta.model) !=
		    0)
			fileSessionMeta.model[0] = '\0';

		if (strcmp(fileSessionMeta.model_id,
			   media->sessionMeta.model_id) != 0)
			fileSessionMeta.model_id[0] = '\0';

		if (strcmp(fileSessionMeta.serial_number,
			   media->sessionMeta.serial_number) != 0)
			fileSessionMeta.serial_number[0] = '\0';

		if (strcmp(fileSessionMeta.software_version,
			   media->sessionMeta.software_version) != 0)
			fileSessionMeta.software_version[0] = '\0';

		if (strcmp(fileSessionMeta.build_id,
			   media->sessionMeta.build_id) != 0)
			fileSessionMeta.build_id[0] = '\0';

		if (strcmp(fileSessionMeta.title, media->sessionMeta.title) !=
		    0)
			fileSessionMeta.title[0] = '\0';

		if (strcmp(fileSessionMeta.comment,
			   media->sessionMeta.comment) != 0)
			fileSessionMeta.comment[0] = '\0';

		if (strcmp(fileSessionMeta.copyright,
			   media->sessionMeta.copyright) != 0)
			fileSessionMeta.copyright[0] = '\0';

		if (fileSessionMeta.media_date !=
		    media->sessionMeta.media_date) {
			fileSessionMeta.media_date = 0;
			fileSessionMeta.media_date_gmtoff = 0;
		}

		if (fileSessionMeta.run_date != media->sessionMeta.run_date) {
			fileSessionMeta.run_date = 0;
			fileSessionMeta.run_date_gmtoff = 0;
		}

		if (strcmp(fileSessionMeta.run_id, media->sessionMeta.run_id) !=
		    0)
			fileSessionMeta.run_id[0] = '\0';

		if (strcmp(fileSessionMeta.boot_id,
			   media->sessionMeta.boot_id) != 0)
			fileSessionMeta.boot_id[0] = '\0';

		if (strcmp(fileSessionMeta.flight_id,
			   media->sessionMeta.flight_id) != 0)
			fileSessionMeta.flight_id[0] = '\0';

		if (strcmp(fileSessionMeta.custom_id,
			   media->sessionMeta.custom_id) != 0)
			fileSessionMeta.custom_id[0] = '\0';

		if (memcmp(&fileSessionMeta.takeoff_loc,
			   &media->sessionMeta.takeoff_loc,
			   sizeof(struct vmeta_location)) != 0)
			fileSessionMeta.takeoff_loc.valid = 0;

		if (memcmp(&fileSessionMeta.picture_fov,
			   &media->sessionMeta.picture_fov,
			   sizeof(struct vmeta_fov)) != 0) {
			fileSessionMeta.picture_fov.has_horz = 0;
			fileSessionMeta.picture_fov.has_vert = 0;
		}

		if ((fileSessionMeta.has_thermal !=
		     media->sessionMeta.has_thermal) ||
		    (memcmp(&fileSessionMeta.thermal,
			    &media->sessionMeta.thermal,
			    sizeof(struct vmeta_thermal)) != 0))
			fileSessionMeta.has_thermal = 0;

		if (fileSessionMeta.camera_type !=
		    media->sessionMeta.camera_type)
			fileSessionMeta.camera_type = VMETA_CAMERA_TYPE_UNKNOWN;

		if (fileSessionMeta.video_mode != media->sessionMeta.video_mode)
			fileSessionMeta.video_mode = VMETA_VIDEO_MODE_UNKNOWN;
	}

	/* Second pass: cancel identical values from trackSessionMeta
	 * and write track-level metadata */
	for (auto it = mTracks.begin(); it != mTracks.end(); it++) {
		Media *media = it->first;
		Track &track = it->second;
		struct vmeta_session trackSessionMeta = media->sessionMeta;

		if (fileSessionMeta.friendly_name[0] != '\0')
			trackSessionMeta.friendly_name[0] = '\0';

		if (fileSessionMeta.maker[0] != '\0')
			trackSessionMeta.maker[0] = '\0';

		if (fileSessionMeta.model[0] != '\0')
			trackSessionMeta.model[0] = '\0';

		if (fileSessionMeta.model_id[0] != '\0')
			trackSessionMeta.model_id[0] = '\0';

		if (fileSessionMeta.serial_number[0] != '\0')
			trackSessionMeta.serial_number[0] = '\0';

		if (fileSessionMeta.software_version[0] != '\0')
			trackSessionMeta.software_version[0] = '\0';

		if (fileSessionMeta.build_id[0] != '\0')
			trackSessionMeta.build_id[0] = '\0';

		if (fileSessionMeta.title[0] != '\0')
			trackSessionMeta.title[0] = '\0';

		if (fileSessionMeta.comment[0] != '\0')
			trackSessionMeta.comment[0] = '\0';

		if (fileSessionMeta.copyright[0] != '\0')
			trackSessionMeta.copyright[0] = '\0';

		if (fileSessionMeta.media_date != 0) {
			trackSessionMeta.media_date = 0;
			trackSessionMeta.media_date_gmtoff = 0;
		}

		if (fileSessionMeta.run_date != 0) {
			trackSessionMeta.run_date = 0;
			trackSessionMeta.run_date_gmtoff = 0;
		}

		if (fileSessionMeta.run_id[0] != '\0')
			trackSessionMeta.run_id[0] = '\0';

		if (fileSessionMeta.boot_id[0] != '\0')
			trackSessionMeta.boot_id[0] = '\0';

		if (fileSessionMeta.flight_id[0] != '\0')
			trackSessionMeta.flight_id[0] = '\0';

		if (fileSessionMeta.custom_id[0] != '\0')
			trackSessionMeta.custom_id[0] = '\0';

		if (fileSessionMeta.takeoff_loc.valid)
			trackSessionMeta.takeoff_loc.valid = 0;

		if (fileSessionMeta.picture_fov.has_horz ||
		    fileSessionMeta.picture_fov.has_vert) {
			trackSessionMeta.picture_fov.has_horz = 0;
			trackSessionMeta.picture_fov.has_vert = 0;
		}

		if (fileSessionMeta.has_thermal)
			trackSessionMeta.has_thermal = 0;

		if (fileSessionMeta.camera_type != VMETA_CAMERA_TYPE_UNKNOWN) {
			trackSessionMeta.camera_type =
				VMETA_CAMERA_TYPE_UNKNOWN;
		}

		if (fileSessionMeta.video_mode != VMETA_VIDEO_MODE_UNKNOWN)
			trackSessionMeta.video_mode = VMETA_VIDEO_MODE_UNKNOWN;

		struct SessionMetaWriteTrackCbUserdata ud = {
			.muxer = this,
			.trackId = track.trackId,
		};
		err = vmeta_session_recording_write(
			&trackSessionMeta, &sessionMetaWriteTrackCb, &ud);
		if (err < 0)
			PDRAW_LOG_ERRNO("vmeta_session_recording_write", -err);
	}

	if (fileSessionMeta.media_date == 0) {
		uint64_t mediaDate = 0;
		int32_t mediaDateGmtoff = 0;
		err = time_local_get(&mediaDate, &mediaDateGmtoff);
		if (err < 0)
			PDRAW_LOG_ERRNO("time_local_get", -err);
		fileSessionMeta.media_date = mediaDate;
		fileSessionMeta.media_date_gmtoff = mediaDateGmtoff;
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
}


int RecordMuxer::addTrackForMedia(
	Media *media,
	uint64_t trackTime,
	const struct pdraw_muxer_video_media_params *videoMediaParams)
{
	int res, trackId;
	const uint8_t *vps = nullptr, *sps = nullptr, *pps = nullptr;
	size_t vpsSize = 0, spsSize = 0, ppsSize = 0;
	struct mp4_video_decoder_config cfg = {};
	CodedVideoMedia *codedMedia = dynamic_cast<CodedVideoMedia *>(media);
	RawVideoMedia *rawMedia = dynamic_cast<RawVideoMedia *>(media);

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mMux == nullptr, EAGAIN);

	if (trackTime == 0)
		trackTime = time(nullptr);

	/* Add a new video track */
	/* size + 1 to start at index 1 */
	std::string name;
	if (videoMediaParams == nullptr ||
	    videoMediaParams->track_name == nullptr)
		name = (codedMedia ? "DefaultVideo" : "RawVideo") +
		       std::to_string(mTracks.size() + 1);
	else
		name = videoMediaParams->track_name;
	struct mp4_mux_track_params params = {
		.type = codedMedia ? MP4_TRACK_TYPE_VIDEO
				   : MP4_TRACK_TYPE_METADATA,
		.name = name.c_str(),
		.enabled = videoMediaParams != nullptr
				   ? videoMediaParams->is_default
				   : true,
		.in_movie = videoMediaParams != nullptr
				    ? videoMediaParams->is_default
				    : true,
		.in_preview = videoMediaParams != nullptr
				      ? videoMediaParams->is_default
				      : true,
		.timescale = (videoMediaParams == nullptr ||
			      videoMediaParams->timescale == 0)
				     ? DEFAULT_MP4_TIMESCALE
				     : videoMediaParams->timescale,
		.creation_time = trackTime,
		.modification_time = trackTime,
	};
	pthread_mutex_lock(&mMp4Mutex);
	res = mp4_mux_add_track(mMux, &params);
	pthread_mutex_unlock(&mMp4Mutex);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}
	trackId = res;
	mTracks.insert({media,
			Track(name,
			      (uint32_t)trackId,
			      0,
			      trackTime,
			      INT64_MAX,
			      params.timescale,
			      videoMediaParams != nullptr
				      ? videoMediaParams->is_default
				      : 1)});

	if (rawMedia != nullptr) {
		/* Raw media: add the format as MIME type */
		char *mime = NULL, *format_str = NULL, *info_str = NULL;

		res = vdef_raw_format_to_csv(&rawMedia->format, &format_str);
		if (res < 0) {
			ULOG_ERRNO("vdef_raw_format_to_csv", -res);
			goto skip_mime;
		}
		res = vdef_format_info_to_csv(&rawMedia->info, &info_str);
		if (res < 0) {
			ULOG_ERRNO("vdef_format_info_to_csv", -res);
			goto skip_mime;
		}
		res = asprintf(&mime,
			       VDEF_RAW_MIME_TYPE ";%s;%s",
			       format_str,
			       info_str);
		if (res < 0) {
			res = -ENOMEM;
			ULOG_ERRNO("asprintf", -res);
			goto skip_mime;
		}
		pthread_mutex_lock(&mMp4Mutex);
		mp4_mux_track_set_metadata_mime_type(
			mMux, trackId, "", mime ? mime : "");
		pthread_mutex_unlock(&mMp4Mutex);
		/* clang-format off */
skip_mime:
		/* clang-format on */
		free(info_str);
		free(format_str);
		free(mime);
		return res;
	} else if (codedMedia == nullptr) {
		return 0;
	}

	switch (codedMedia->format.encoding) {
	case VDEF_ENCODING_H264:
		/* Set the H.264 parameter sets */
		res = codedMedia->getPs(
			nullptr, nullptr, &sps, &spsSize, &pps, &ppsSize);
		if (res < 0) {
			PDRAW_LOG_ERRNO("CodedVideoMedia::getPs", -res);
			return res;
		}
		cfg.codec = MP4_VIDEO_CODEC_AVC;
		cfg.width = codedMedia->info.resolution.width;
		cfg.height = codedMedia->info.resolution.height;
		cfg.avc.c_sps = sps;
		cfg.avc.sps_size = spsSize;
		cfg.avc.c_pps = pps;
		cfg.avc.pps_size = ppsSize;
		break;
	case VDEF_ENCODING_H265:
		/* Set the H.265 parameter sets */
		res = codedMedia->getPs(
			&vps, &vpsSize, &sps, &spsSize, &pps, &ppsSize);
		if (res < 0) {
			PDRAW_LOG_ERRNO("CodedVideoMedia::getPs", -res);
			return res;
		}
		cfg.codec = MP4_VIDEO_CODEC_HEVC;
		cfg.width = codedMedia->info.resolution.width;
		cfg.height = codedMedia->info.resolution.height;
		cfg.hevc.c_vps = vps;
		cfg.hevc.vps_size = vpsSize;
		cfg.hevc.c_sps = sps;
		cfg.hevc.sps_size = spsSize;
		cfg.hevc.c_pps = pps;
		cfg.hevc.pps_size = ppsSize;
		/* TODO: fill cfg.hevc.hvcc_info */
		break;
	default:
		break;
	}

	pthread_mutex_lock(&mMp4Mutex);
	res = mp4_mux_track_set_video_decoder_config(mMux, trackId, &cfg);
	pthread_mutex_unlock(&mMp4Mutex);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_set_video_decoder_config", -res);
		return res;
	}
	return 0;
}


/* mMp4Mutex should be locked before calling this function */
int RecordMuxer::addMetadataTrack(Track *ref, enum vmeta_frame_type metaType)
{
	int res;
	const char *mimeType = nullptr;
	const char *contentEncoding = nullptr;

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
		.creation_time = ref->trackTime,
		.modification_time = ref->trackTime,
	};
	res = mp4_mux_add_track(mMux, &params);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_track", -res);
		return res;
	}
	ref->metaTrackId = (uint32_t)res;

	/* Set the metadata mime type */
	res = mp4_mux_track_set_metadata_mime_type(
		mMux, ref->metaTrackId, contentEncoding, mimeType);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_set_metadata_mime_type", -res);
		ref->metaTrackId = 0;
		return res;
	}

	/* Add track reference */
	res = mp4_mux_add_ref_to_track(mMux, ref->metaTrackId, ref->trackId);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_add_ref_to_track", -res);
		ref->metaTrackId = 0;
		return res;
	}

	ref->mHasMetadataTrack = true;

	return 0;
}


int RecordMuxer::process(void)
{
	int res, inputMediaCount, i;

	if (mState != STARTED)
		return 0;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mMux == nullptr, EAGAIN);

	Sink::lock();

	inputMediaCount = getInputMediaCount();
	for (i = 0; i < inputMediaCount; i++) {
		Media *media = getInputMedia(i);
		if (media == nullptr) {
			res = -ENOENT;
			PDRAW_LOG_ERRNO("getInputMedia", -res);
			continue;
		}

		/* Process each media */
		processMedia(media);
	}

	Sink::unlock();

	return 0;
}


int RecordMuxer::processMedia(Media *media)
{
	int res = 0, err;

	CodedVideoMedia *codedMedia = dynamic_cast<CodedVideoMedia *>(media);
	RawVideoMedia *rawMedia = dynamic_cast<RawVideoMedia *>(media);

	Track &track = mTracks[media];

	if (track.trackId == 0) {
		res = -ENOENT;
		PDRAW_LOG_ERRNO("track->trackId", -res);
		return res;
	}

	CodedVideoChannel *codedChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(media));
	RawVideoChannel *rawChannel =
		dynamic_cast<RawVideoChannel *>(getInputChannel(media));
	struct mbuf_coded_video_frame_queue *codedQueue =
		codedChannel ? codedChannel->getQueue(this) : nullptr;
	struct mbuf_raw_video_frame_queue *rawQueue =
		rawChannel ? rawChannel->getQueue(this) : nullptr;

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
			pthread_mutex_lock(&mMp4Mutex);
			res = processFrame(codedMedia, &track, frame);
			pthread_mutex_unlock(&mMp4Mutex);
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
			pthread_mutex_lock(&mMp4Mutex);
			res = processFrame(rawMedia, &track, frame);
			pthread_mutex_unlock(&mMp4Mutex);
			err = mbuf_raw_video_frame_unref(frame);
			if (err < 0)
				PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref",
						-err);
		} else
			break;

	} while (res == 0);
	return res;
}


int RecordMuxer::checkFreeSpace(size_t spaceNeeded, size_t &spaceLeft)
{
	/* Bypass free space limit if required */
	if (mParams.free_space_limit == 0)
		return 0;

#ifdef _WIN32
	char volume[MAX_PATH] = "";
	ULARGE_INTEGER freeBytes = {};

	GetVolumePathNameA(mFileName.c_str(), volume, sizeof(volume));
	GetDiskFreeSpaceExA(volume, &freeBytes, NULL, NULL);

	spaceLeft = freeBytes.QuadPart;
#else
	int ret;
	struct statvfs stats = {};

	ret = statvfs(mFileName.c_str(), &stats);
	if (ret < 0) {
		ret = -errno;
		PDRAW_LOG_ERRNO("statvfs", -ret);
		return ret;
	}
	spaceLeft = stats.f_bavail * stats.f_bsize;
#endif

	if (spaceLeft < (mParams.free_space_limit + spaceNeeded)) {
		PDRAW_LOGW("free space limit %.1f MiB reached",
			   (float)mParams.free_space_limit / 1024 / 1024);
		return -ENOSPC;
	}
	return 0;
}


/* Must be called on the loop thread */
void RecordMuxer::callNoSpaceLeft(void *userdata)
{
	RecordMuxer *self = reinterpret_cast<RecordMuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->getMuxerListener() != nullptr)
		self->getMuxerListener()->onMuxerNoSpaceLeft(
			self->mSession,
			self->getMuxer(),
			self->mParams.free_space_limit,
			self->mFreeSpaceLeft);
}


int RecordMuxer::processFrame(CodedVideoMedia *media,
			      Track *track,
			      struct mbuf_coded_video_frame *frame)
{
	int res;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	struct vmeta_frame *metadata = nullptr;
	CodedVideoMedia::Frame *meta;
	struct mp4_mux_scattered_sample sample = {};
	const void *aData;
	const void **frameNalus = nullptr;
	size_t *frameNalusSize = nullptr;
	uint64_t spaceNeeded = 0;

	if (mNoSpaceLeft)
		return -ENOSPC;

	if (mMux == nullptr || !mWriterThread.started)
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
	sample.dts = mp4_usec_to_sample_time(meta->ntpRawTimestamp,
					     track->mTimescale);

	/* Decoding timestamps must be monotonic; avoid duplicate or
	 * rollback timestamps by faking the timestamp as the previous
	 * timestamp + 1 */
	if ((track->lastSampleTs != INT64_MAX) &&
	    (sample.dts == track->lastSampleTs)) {
		PDRAW_LOGW("duplicate timestamp (%" PRIu64 "), incrementing",
			   track->lastSampleTs);
		sample.dts = track->lastSampleTs + 1;
	} else if ((track->lastSampleTs != INT64_MAX) &&
		   (sample.dts < track->lastSampleTs)) {
		PDRAW_LOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
			   ", incrementing",
			   track->lastSampleTs,
			   sample.dts);
		sample.dts = track->lastSampleTs + 1;
	}
	track->lastSampleTs = sample.dts;

	res = checkFreeSpace(spaceNeeded, mFreeSpaceLeft);
	if (res < 0) {
		if (res != -ENOSPC)
			PDRAW_LOG_ERRNO("checkFreeSpace", -res);
		else
			mNoSpaceLeft = true;
		if (res == -ENOSPC) {
			int err = pomp_loop_idle_add_with_cookie(
				mSession->getLoop(),
				&callNoSpaceLeft,
				this,
				this);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"pomp_loop_idle_add_with_cookie", -err);
		}
		goto out;
	}
	res = mp4_mux_track_add_scattered_sample(mMux, track->trackId, &sample);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mp4_mux_track_add_scattered_sample", -res);
		goto out;
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
	if (!track->mHasMetadataTrack)
		addMetadataTrack(track, metadata->type);
	if (track->metaTrackId) {
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
				mMux, track->metaTrackId, &metaSample);
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


int RecordMuxer::processFrame(RawVideoMedia *media,
			      Track *track,
			      struct mbuf_raw_video_frame *frame)
{
	int res = 0;
	const void *buf = nullptr;
	size_t len;
	struct vdef_raw_frame info;
	unsigned int nplanes = 0;
	struct mp4_mux_sample sample;
	struct vmeta_frame *metadata = nullptr;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	RawVideoMedia::Frame *meta;
	const void *aData;

	if (mNoSpaceLeft)
		return -ENOSPC;

	if (mMux == nullptr || !mWriterThread.started)
		return -EPROTO;

	res = mbuf_raw_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -res);
		return res;
	}

	nplanes = vdef_get_raw_frame_plane_count(&info.format);

	res = mbuf_raw_video_frame_get_packed_buffer(frame, &buf, &len);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_packed_buffer", -res);
		goto release;
	}

	res = mbuf_raw_video_frame_get_ancillary_data(
		frame, PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME, &ancillaryData);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_ancillary_data",
				-res);
		goto release;
	}
	aData = mbuf_ancillary_data_get_buffer(ancillaryData, nullptr);
	meta = (RawVideoMedia::Frame *)aData;

	/* Add a video sample to the MP4 muxer */
	sample.buffer = (const uint8_t *)buf;
	sample.len = len;
	sample.sync = 1;
	sample.dts = mp4_usec_to_sample_time(meta->ntpRawTimestamp,
					     track->mTimescale);

	/* Decoding timestamps must be monotonic; avoid duplicate or
	 * rollback timestamps by faking the timestamp as the previous
	 * timestamp + 1 */
	if ((track->lastSampleTs != INT64_MAX) &&
	    (sample.dts == track->lastSampleTs)) {
		PDRAW_LOGW("duplicate timestamp (%" PRIu64 "), incrementing",
			   track->lastSampleTs);
		sample.dts = track->lastSampleTs + 1;
	} else if ((track->lastSampleTs != INT64_MAX) &&
		   (sample.dts < track->lastSampleTs)) {
		PDRAW_LOGW("timestamp rollback from %" PRIu64 " to %" PRIu64
			   ", incrementing",
			   track->lastSampleTs,
			   sample.dts);
		sample.dts = track->lastSampleTs + 1;
	}
	track->lastSampleTs = sample.dts;

	res = checkFreeSpace(len, mFreeSpaceLeft);
	if (res < 0) {
		if (res != -ENOSPC)
			PDRAW_LOG_ERRNO("checkFreeSpace", -res);
		else
			mNoSpaceLeft = true;
		if (res == -ENOSPC) {
			int err = pomp_loop_idle_add_with_cookie(
				mSession->getLoop(),
				&callNoSpaceLeft,
				this,
				this);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"pomp_loop_idle_add_with_cookie", -err);
		}
		goto release;
	}
	res = mp4_mux_track_add_sample(mMux, track->trackId, &sample);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_track_add_sample", -res);

	res = mbuf_raw_video_frame_get_metadata(frame, &metadata);
	if (res == -ENOENT) {
		/* No metadata, skip to the end */
		res = 0;
		goto release;
	} else if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -res);
		goto release;
	}
	if (!track->mHasMetadataTrack)
		addMetadataTrack(track, metadata->type);
	if (track->metaTrackId) {
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
				goto release;
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
				mMux, track->metaTrackId, &metaSample);
			if (res < 0)
				PDRAW_LOG_ERRNO("mp4_mux_track_add_sample",
						-res);
		}
		if (metadata->type == VMETA_FRAME_TYPE_PROTO)
			vmeta_frame_proto_release_buffer(metadata, metaContent);
	}
release:
	if (metadata)
		vmeta_frame_unref(metadata);
	if (ancillaryData)
		mbuf_ancillary_data_unref(ancillaryData);
	if (buf)
		mbuf_raw_video_frame_release_packed_buffer(frame, buf);
	return res;
}


void RecordMuxer::mboxCb(int fd, uint32_t revents, void *userdata)
{
	struct mbox_mux_loop *param = (struct mbox_mux_loop *)userdata;
	RecordMuxer *self = param->recordMuxer;
	int err;
	void *message;

	if (param->loop == nullptr || self == nullptr)
		return;
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
		case CMD_TYPE_ADD_CODED_EVENT: {
			err = self->addQueueEvtToLoop(msg->add_coded_evt.queue,
						      param->loop);
			if (err < 0)
				PDRAW_LOG_ERRNO("addQueueEvtToLoop", -err);
			break;
		}
		case CMD_TYPE_ADD_RAW_EVENT: {
			err = self->addQueueEvtToLoop(msg->add_raw_evt.queue,
						      param->loop);
			if (err < 0)
				PDRAW_LOG_ERRNO("addQueueEvtToLoop", -err);
			break;
		}
		case CMD_TYPE_REMOVE_CODED_EVENT: {
			err = self->removeQueueEvtFromLoop(
				msg->remove_coded_evt.queue, param->loop);
			if (err < 0)
				PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -err);
			err = mbuf_coded_video_frame_queue_flush(
				msg->remove_coded_evt.queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_flush",
					-err);
			err = mbuf_coded_video_frame_queue_destroy(
				msg->remove_coded_evt.queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_destroy",
					-err);
			break;
		}
		case CMD_TYPE_REMOVE_RAW_EVENT: {
			err = self->removeQueueEvtFromLoop(
				msg->remove_raw_evt.queue, param->loop);
			if (err < 0)
				PDRAW_LOG_ERRNO("removeQueueEvtFromLoop", -err);
			err = mbuf_raw_video_frame_queue_flush(
				msg->remove_raw_evt.queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_raw_video_frame_queue_flush",
					-err);
			err = mbuf_raw_video_frame_queue_destroy(
				msg->remove_raw_evt.queue);
			if (err < 0)
				PDRAW_LOG_ERRNO(
					"mbuf_raw_video_frame_queue_destroy",
					-err);
			break;
		}
		case CMD_TYPE_STOP_THREAD: {
			self->mWriterThread.started = false;
			break;
		}
		default:
			PDRAW_LOGE("unknown command: %d", msg->type);
			break;
		}
	} while (err == 0);
	free(message);
}


void RecordMuxer::sessionMetaWriteFileCb(enum vmeta_record_type type,
					 const char *key,
					 const char *value,
					 void *userdata)
{
	int res;
	RecordMuxer *self = (RecordMuxer *)userdata;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	pthread_mutex_lock(&self->mMp4Mutex);
	res = mp4_mux_add_file_metadata(self->mMux, key, value);
	pthread_mutex_unlock(&self->mMp4Mutex);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_add_file_metadata", -res);
}


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

	pthread_mutex_lock(&self->mMp4Mutex);
	res = mp4_mux_add_track_metadata(self->mMux, trackId, key, value);
	pthread_mutex_unlock(&self->mMp4Mutex);
	if (res < 0)
		PDRAW_LOG_ERRNO("mp4_mux_add_track_metadata", -res);
}

} /* namespace Pdraw */
