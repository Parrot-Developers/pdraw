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

#define ULOG_TAG pdraw_recmux
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_record.hpp"
#include "pdraw_muxer_record_media.hpp"
#include "pdraw_session.hpp"

#include <array>
#include <time.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <sys/statvfs.h>
#endif /* !_WIN32 */

#include <futils/futils.h>

#if defined(__APPLE__)
#	include <TargetConditionals.h>
#endif

namespace Pdraw {


constexpr uint64_t CHECK_FREE_SPACE_FREQUENCY_US = 1000000ULL;


/* codecheck_ignore[COMPLEX_MACRO] */
#define ENUM_CLASS_CASE(_enum, _name)                                          \
	case _enum::_name:                                                     \
		return #_name


static const char *cmdTypeToStr(CmdType type)
{
	/* clang-format off */
	switch (type) {
	ENUM_CLASS_CASE(CmdType, ADD_TRACK);
	ENUM_CLASS_CASE(CmdType, ADD_QUEUE_EVENT);
	ENUM_CLASS_CASE(CmdType, REMOVE_QUEUE_EVENT);
	ENUM_CLASS_CASE(CmdType, SET_THUMBNAIL);
	ENUM_CLASS_CASE(CmdType, SET_FILE_METADATA);
	ENUM_CLASS_CASE(CmdType, SET_METADATA);
	ENUM_CLASS_CASE(CmdType, FLUSH);
	ENUM_CLASS_CASE(CmdType, DRAIN);
	ENUM_CLASS_CASE(CmdType, STOP_THREAD);
	ENUM_CLASS_CASE(CmdType, SET_DYN_PARAMS);
	ENUM_CLASS_CASE(CmdType, ADD_CHAPTER);
	ENUM_CLASS_CASE(CmdType, FORCE_SYNC);
	default:
		return "UNKNOWN";
	}
	/* clang-format on */
}


RecordMuxer::RecordMuxer(Session *session,
			 Element::Listener *elementListener,
			 IPdraw::IMuxer::Listener *listener,
			 MuxerWrapper *wrapper,
			 const std::string &fileName,
			 const struct pdraw_muxer_params *params) :
		Muxer(session, elementListener, listener, wrapper, params),
		mFileName(fileName)
{
	Element::setClassName(__func__);

	try {
		mThread.evt = make_unique<pomp::Event>();
	} catch (const std::bad_alloc &e) {
		PDRAW_LOGE("allocation failed: %s", e.what());
	}

	size_t foundDir = mFileName.find_last_of("/\\");
	if (foundDir == std::string::npos) {
		mStorageDirPath = ".";
	} else {
		mStorageDirPath = mFileName.substr(0, foundDir);
	}

	mFileMode = params->filemode;
	mStats.type = PDRAW_MUXER_TYPE_RECORD;

	mThread.evtHandler.set([this] { processTasks(); });
}


RecordMuxer::~RecordMuxer()
{
	int err;

	err = internalStop();
	if (err < 0)
		PDRAW_LOG_ERRNO("internalStop", -err);

	if (isThreadAlive()) {
		mThread.shouldStop = true;
		if (isThreadRunning()) {
			err = mThread.loop->wakeup();
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_loop_wakeup", -err);
		}
	}
	if (isThreadJoinable())
		mThread.thread.join();

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);
}


/* Must be called on the loop thread */
int RecordMuxer::addInputMedia(Media *media,
			       const struct pdraw_muxer_media_params *params)
{
	int ret;
	mbuf::Queue *queue = nullptr;

	PDRAW_CHECK_WRITER_THREAD(false);

	if (media == nullptr) {
		PDRAW_LOGE("%s: unsupported input media", __func__);
		return -EINVAL;
	}

	if (!canPostTask(false)) {
		PDRAW_LOGE("%s: cannot send command to thread", __func__);
		return -EPROTO;
	}

	Sink::lock();

	ret = Sink::addInputMedia(media);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("Sink::addInputMedia", -ret);
		goto out_unlock;
	}

	{
		Channel *channel = getInputChannel(media);
		if (!channel) {
			PDRAW_LOGE("No channel found for media %p", media);
			ret = -ENODEV;
			goto error_remove;
		}

		ret = createInputQueue(media->type, &queue);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("createInputQueue", -ret);
			goto error_remove;
		}

		channel->setQueue(this, queue);
	}

	{
		Media::Type mediaType = media->type;
		ret = postTask(
			CmdType::ADD_QUEUE_EVENT, [this, mediaType, queue]() {
				int err = this->internalAddQueueEvtToLoop(
					mediaType, queue);
				if (err < 0)
					PDRAW_LOG_ERRNO(
						"internalAddQueueEvtToLoop",
						-err);
			});

		if (ret < 0) {
			PDRAW_LOG_ERRNO("postTask", -ret);
			goto error_remove;
		}
	}

	ret = addMuxerMedia(media, params);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("addMuxerMedia", -ret);
		goto error_remove;
	}

	Sink::unlock();
	return 0;

error_remove:
	removeInputMedia(media);

out_unlock:
	Sink::unlock();
	return ret;
}


/* Must be called on the loop thread */
int RecordMuxer::removeInputMedia(Media *media)
{
	int res = 0;
	mbuf::Queue *queue = nullptr;

	PDRAW_CHECK_WRITER_THREAD(false);

	Sink::lock();

	Channel *channel = getInputChannel(media);
	if (!channel) {
		PDRAW_LOGE("no channel found for media %p", media);
		res = -ENODEV;
		goto out;
	}

	queue = channel->getQueue(this);

	if (queue) {
		channel->setQueue(this, nullptr);
		bool taskPosted = false;

		if (canPostTask(false)) {
			Media::Type mediaType = media->type;
			auto task = [this, mediaType, queue]() {
				int err = internalRemoveQueueEvtFromLoop(
					mediaType, queue);
				if (err < 0) {
					PDRAW_LOG_ERRNO(
						"internalRemoveQueueEvtFromLoop",
						-err);
				}
			};
			res = postTask(CmdType::REMOVE_QUEUE_EVENT, task);
			if (res == 0)
				taskPosted = true;
			else
				PDRAW_LOG_ERRNO("postTask", -res);
		}
		if (!taskPosted)
			delete queue;
	}

	res = Sink::removeInputMedia(media);
	if (res < 0)
		PDRAW_LOG_ERRNO("Sink::removeInputMedia", -res);

out:
	Sink::unlock();
	return res;
}


/* Must be called on the loop thread */
int RecordMuxer::setThumbnail(enum pdraw_muxer_thumbnail_type type,
			      const uint8_t *data,
			      size_t size)
{
	int ret;

	PDRAW_CHECK_WRITER_THREAD(false);

	ULOG_ERRNO_RETURN_ERR_IF(type == PDRAW_MUXER_THUMBNAIL_TYPE_UNKNOWN,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size == 0, EINVAL);

	ULOGI("setThumbnail type=%d, size=%zu", type, size);

	std::vector<uint8_t> buffer(data, data + size);

	ret = postTask(CmdType::SET_THUMBNAIL, [this, type, buffer]() {
		int err = this->internalSetThumbnail(
			type, buffer.data(), buffer.size());
		if (err < 0)
			PDRAW_LOG_ERRNO("internalSetThumbnail", -err);
	});
	if (ret < 0) {
		PDRAW_LOG_ERRNO("postTask", -ret);
		return ret;
	}

	return 0;
}


/* Must be called on the loop thread */
int RecordMuxer::setFileMetadata(enum pdraw_muxer_metadata_type type,
				 const uint8_t *data,
				 size_t size,
				 const void *params,
				 size_t paramsSize)
{
	int ret;

	PDRAW_CHECK_WRITER_THREAD(false);

	ULOG_ERRNO_RETURN_ERR_IF(type == PDRAW_MUXER_METADATA_TYPE_UNKNOWN,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size == 0, EINVAL);

	ULOGI("setFileMetadata type=%d, size=%zu", type, size);

	std::vector<uint8_t> buffer(data, data + size);
	std::vector<uint8_t> paramsBuffer;
	if (params && paramsSize > 0) {
		const uint8_t *p = static_cast<const uint8_t *>(params);
		paramsBuffer.assign(p, p + paramsSize);
	}

	ret = postTask(CmdType::SET_FILE_METADATA,
		       [this, type, buffer, paramsBuffer]() {
			       int err = this->internalSetFileMetadata(
				       type,
				       buffer.data(),
				       buffer.size(),
				       paramsBuffer.data(),
				       paramsBuffer.size());
			       if (err < 0)
				       PDRAW_LOG_ERRNO(
					       "internalSetFileMetadata", -err);
		       });
	if (ret < 0) {
		PDRAW_LOG_ERRNO("postTask", -ret);
		return ret;
	}

	return 0;
}


class SafeDynParams {
public:
	explicit SafeDynParams(const struct pdraw_muxer_dyn_params *src)
	{
		if (!src)
			return;

		mParams = *src;

		if (src->file_name_pattern) {
			mFilePattern = src->file_name_pattern;
			mParams.file_name_pattern = mFilePattern.c_str();
		} else {
			mParams.file_name_pattern = nullptr;
		}
	}

	const struct pdraw_muxer_dyn_params *get() const
	{
		return &mParams;
	}

private:
	struct pdraw_muxer_dyn_params mParams;
	std::string mFilePattern;
};


/* Called on the loop thread */
int RecordMuxer::setDynParams(const struct pdraw_muxer_dyn_params *dyn_params)
{
	int ret = 0;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	PDRAW_CHECK_WRITER_THREAD(false);

	if (!canPostTask(true)) {
		ret = -EPROTO;
		PDRAW_LOGE("%s: cannot send command to thread", __func__);
		return ret;
	}

	auto safeParams = std::make_shared<SafeDynParams>(dyn_params);

	ret = postTask(CmdType::SET_DYN_PARAMS, [this, safeParams]() {
		int err = this->internalSetDynParams(safeParams->get());
		if (err < 0)
			PDRAW_LOG_ERRNO("internalSetDynParams", -err);
	});
	if (ret < 0) {
		PDRAW_LOG_ERRNO("postTask", -ret);
		return ret;
	}

	return 0;
}


/* Called on the loop thread */
int RecordMuxer::getDynParams(struct pdraw_muxer_dyn_params *dyn_params)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	PDRAW_CHECK_WRITER_THREAD(false);

	return 0;
}


/* Called on the loop thread */
int RecordMuxer::getStats(struct pdraw_muxer_stats *stats)
{
	ULOG_ERRNO_RETURN_ERR_IF(stats == nullptr, EINVAL);

	PDRAW_CHECK_WRITER_THREAD(false);

	*stats = mStats;

	return 0;
}


/* Called on the writer thread */
int RecordMuxer::internalSetMetadata(uint32_t mediaId,
				     const struct vmeta_session *metadata)
{
	ULOG_ERRNO_RETURN_ERR_IF(metadata == nullptr, EINVAL);

	PDRAW_CHECK_WRITER_THREAD(true);

	for (const auto &track : mMedias) {
		if (track->getMediaId() == mediaId) {
			track->setSessionMeta(metadata);
			mMetadataChanged = true;
			return 0;
		}
	}

	return -ENOENT;
}


void RecordMuxer::logThreadCheckWarning(const char *funcName,
					bool shouldBeWriterThread) const
{
	bool calledFromWriter = (std::this_thread::get_id() == mThread.id);
	if (shouldBeWriterThread && !calledFromWriter)
		PDRAW_LOGW("%s not called from the writer thread", funcName);
	else if (!shouldBeWriterThread && calledFromWriter)
		PDRAW_LOGW("%s called from the writer thread", funcName);
}


/* Called on the loop thread */
bool RecordMuxer::canPostTask(bool needsMux) const
{
	if (!mThread.evt || !isThreadAlive() ||
	    getThreadState() == State::STOPPING || mPendingStop)
		return false;

	if (needsMux && !mIsMuxerReady)
		return false;

	return true;
}


/* Called on the loop thread */
int RecordMuxer::postTask(CmdType type, const RecordTask &task)
{
	int res = 0;

	PDRAW_CHECK_WRITER_THREAD(false);

	if (mThread.shouldStop.load())
		return -EPIPE;

	pomp::Event *evtToSignal = nullptr;
	size_t taskId;

	{
		std::lock_guard<std::mutex> lock(mTasksMutex);

		evtToSignal = mThread.evt.get();
		if (!evtToSignal)
			return -EINVAL;

		taskId = ++mThread.taskIdCounter;
		mPendingTasks.emplace([this, type, taskId, task]() mutable {
			PDRAW_LOGD("received command #%zu '%s'",
				   taskId,
				   cmdTypeToStr(type));
			task();
		});

		res = evtToSignal->signal();
	}

	if (res < 0) {
		PDRAW_LOG_ERRNO("signal", -res);
		return res;
	}

	PDRAW_LOGD("sending command #%zu '%s'", taskId, cmdTypeToStr(type));

	return 0;
}


static ssize_t getFreeSpace(const std::string &filePath)
{
	size_t freeSpaceLeft;

#ifdef _WIN32
	char volume[MAX_PATH] = "";
	ULARGE_INTEGER freeBytes = {};

	GetVolumePathNameA(filePath.c_str(), volume, sizeof(volume));
	GetDiskFreeSpaceExA(volume, &freeBytes, nullptr, nullptr);

	freeSpaceLeft = freeBytes.QuadPart;
#else
	int ret;
	struct statvfs stats = {};

	ret = statvfs(filePath.c_str(), &stats);
	if (ret < 0) {
		ret = -errno;
		ULOG_ERRNO("statvf(%s)", -ret, filePath.c_str());
		return ret;
	}
	freeSpaceLeft = stats.f_bavail * stats.f_bsize;
#endif
	return freeSpaceLeft;
}


/* Called from any thread
 * No mutex needed because only called by the loop thread during init, then only
 * by the writer thread. */
int RecordMuxer::ensureFreeSpace(size_t spaceNeeded)
{
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;

	/* Bypass free space limit if required */
	if (mParams.free_space_limit == 0)
		return 0;

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);

	int64_t elapsedUs = curTime - mLastCheckFreeSpaceTime;

	if (elapsedUs >= (int64_t)CHECK_FREE_SPACE_FREQUENCY_US) {
		ssize_t ret = getFreeSpace(mStorageDirPath);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("getFreeSpace", (int)-ret);
			return (int)ret;
		}
		mFreeSpaceLeft = (size_t)ret;
		mLastCheckFreeSpaceTime = curTime;
	}

	if (mFreeSpaceLeft < (mParams.free_space_limit + spaceNeeded)) {
		PDRAW_LOGW(
			"free space left (%.1f MiB) "
			"below free space limit (%.1f MiB)",
			(float)mFreeSpaceLeft / (1024 * 1024),
			(float)(mParams.free_space_limit + spaceNeeded) /
				(1024 * 1024));
		return -ENOSPC;
	}

	if (mFreeSpaceLeft >= spaceNeeded)
		mFreeSpaceLeft -= spaceNeeded;
	else
		mFreeSpaceLeft = 0;

	return 0;
}


/* Called on the loop thread */
int RecordMuxer::addMuxerMedia(Media *media,
			       const struct pdraw_muxer_media_params *params)
{
	int ret;

	PDRAW_CHECK_WRITER_THREAD(false);

	if (!canPostTask(true)) {
		PDRAW_LOGE("%s: cannot send command to thread", __func__);
		return -EPROTO;
	}

	uint32_t mediaId = media->id;
	Media::Type mediaType;
	struct pdraw_media_info mediaInfo;
	memset(&mediaInfo, 0, sizeof(mediaInfo));

	media->fillMediaInfo(&mediaInfo);

	if (dynamic_cast<const CodedVideoMedia *>(media)) {
		mediaType = Media::Type::CODED_VIDEO;
	} else if (dynamic_cast<const RawVideoMedia *>(media)) {
		mediaType = Media::Type::RAW_VIDEO;
	} else if (dynamic_cast<const AudioMedia *>(media)) {
		mediaType = Media::Type::AUDIO;
	} else {
		Media::cleanupMediaInfo(&mediaInfo);
		return -EINVAL;
	}

	pdraw_muxer_media_params p = {};
	std::string trackName;
	if (params != nullptr) {
		p = *params;
		if (params->track_name != nullptr) {
			trackName = params->track_name;
		}
	}

	ret = postTask(
		CmdType::ADD_TRACK,
		[this, mediaId, mediaType, mediaInfo, p, trackName]() mutable {
			pdraw_muxer_media_params finalParams = p;
			if (!trackName.empty()) {
				finalParams.track_name = trackName.c_str();
			}

			int err = this->internalAddMuxerMedia(
				mediaId, mediaType, &mediaInfo, &finalParams);
			if (err < 0)
				PDRAW_LOG_ERRNO("internalAddMuxerMedia", -err);

			Media::cleanupMediaInfo(&mediaInfo);
		});
	if (ret < 0)
		PDRAW_LOG_ERRNO("postTask", -ret);

	return ret;
}


/* Must be called on the writer thread */
int RecordMuxer::internalAddMuxerMedia(
	uint32_t mediaId,
	Media::Type mediaType,
	const struct pdraw_media_info *mediaInfo,
	const struct pdraw_muxer_media_params *params)
{
	int res;
	MuxerMediaConfig cfg;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(params == nullptr, EAGAIN);
	PDRAW_CHECK_WRITER_THREAD(true);

	cfg.type = mediaType;
	cfg.mediaId = mediaId;
	cfg.trackTime = time(nullptr);

	if (params->track_name) {
		cfg.name = params->track_name;
	} else {
		std::string baseName = getDefaultMediaName(mediaType);
		cfg.name = baseName + std::to_string(mMedias.size() + 1);
	}

	if (MuxerMedia::isVideo(mediaType) && mediaInfo &&
	    mediaInfo->video.session_meta) {
		cfg.sessionMeta = mediaInfo->video.session_meta;
	}

	res = onBeforeMediaCreation(cfg, mediaInfo, params);
	if (res < 0)
		return res;

	Sink::lock();
	Channel *channel = nullptr;
	unsigned int mediaCount = getInputMediaCount();
	for (unsigned int i = 0; i < mediaCount; i++) {
		const Media *m = getInputMedia(i);
		if (m && m->id == mediaId) {
			channel = getInputChannel(m);
			break;
		}
	}
	Sink::unlock();

	if (!channel) {
		PDRAW_LOGE("failed to get channel for media %u", mediaId);
		return -ENOENT;
	}

	std::unique_ptr<RecordMuxer::MuxerMedia> track = createMedia(cfg);
	if (!track)
		return -ENOMEM;

	track->setQueue(channel->getQueue(this));

	res = track->setup(mediaInfo, params);
	if (res < 0)
		return res;

	MuxerMedia *trackPtr = track.get();
	mMedias.push_back(std::move(track));

	return onAfterMediaCreation(trackPtr, cfg);
}


/* Called on the writer thread */
int RecordMuxer::internalAddQueueEvtToLoop(Media::Type type, mbuf::Queue *queue)
{
	PDRAW_CHECK_WRITER_THREAD(true);

	int ret = queue->attachToLoop(mThread.loop->get(), &queueEventCb, this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("queue::attachToLoop", -ret);

	return ret;
}


/* Called on the writer thread */
int RecordMuxer::internalRemoveQueueEvtFromLoop(Media::Type type,
						mbuf::Queue *queue)
{
	PDRAW_CHECK_WRITER_THREAD(true);

	if (!queue)
		return -EINVAL;

	void *qPtr = queue->getQueuePtr();

	for (const auto &track : mMedias) {
		if ((track->getMediaType() == type) &&
		    (track->getQueue() == qPtr)) {
			track->setQueue(nullptr);
		}
	}

	queue->detachFromLoop(mThread.loop->get());

	queue->flush();
	delete queue;

	return 0;
}


/* Must be called on the loop thread */
int RecordMuxer::internalStart()
{
	int res;
	int err;
	int inputMediaCount;

	PDRAW_CHECK_WRITER_THREAD(false);

	err = time_local_get(&mMediaDate, &mMediaDateGmtOff);
	if (err < 0)
		PDRAW_LOG_ERRNO("time_local_get", -err);

	/* Ensure that there is enough free space on the storage */
	res = ensureFreeSpace(0);
	if (res < 0) {
		PDRAW_LOG_ERRNO("ensureFreeSpace", -res);
		return res;
	}

	res = onBeforeAddMuxerMedias();
	if (res < 0) {
		PDRAW_LOG_ERRNO("onBeforeAddTracks", -res);
		return res;
	}

	Sink::lock();

	inputMediaCount = getInputMediaCount();

	/* Add a track for all existing medias (async) */
	for (int i = 0; i < inputMediaCount; i++) {
		Media *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOG_ERRNO("getInputMedia", ENOENT);
			continue;
		}
		res = addMuxerMedia(media, nullptr);
		if (res < 0) {
			PDRAW_LOG_ERRNO("addMuxerMedia", -res);
			continue;
		}
	}

	Sink::unlock();

	setThreadState(State::CREATED);

	try {
		mThread.thread = std::thread([this]() {
			this->mThread.id = std::this_thread::get_id();
			this->writerThread();
		});
	} catch (const std::system_error &e) {
		res = -EPROTO;
		PDRAW_LOGE("failed to create thread: %s", e.what());
		setThreadState(State::INVALID);
	}

	return res;
}


/* Must be called on the loop thread */
int RecordMuxer::internalStop()
{
	int err;

	PDRAW_CHECK_WRITER_THREAD(false);

	/* Writer thread is not running and stop is not pending, return
	 * immediately */
	if (!canPostTask(false))
		return 0;

	mReadyToStop = false;

	/* Stop is pending, idleCompleteStop will be called when the writer
	 * thread exits */
	if (mPendingStop)
		return 0;

	/* All media must be removed before stopping the thread */
	err = removeInputMedias();
	if (err < 0)
		PDRAW_LOG_ERRNO("removeInputMedias", -err);

	mPendingStop = true;

	/* Stop the thread */
	err = postTask(CmdType::STOP_THREAD, [this]() {
		int res = this->internalStopThread();
		if (res < 0)
			PDRAW_LOG_ERRNO("internalStopThread", -res);
	});
	if (err < 0) {
		PDRAW_LOG_ERRNO("postTask", -err);
		mPendingStop = false;
	}

	return 0;
}


/* Called on the loop thread */
void RecordMuxer::onChannelFlush(Channel *channel)
{
	PDRAW_CHECK_WRITER_THREAD(false);

	mAsyncFlush = canPostTask(false);

	if (!mAsyncFlush) {
		/* Thread is not running, call flush on the loop thread */
		Muxer::onChannelFlush(channel);
		return;
	}

	int err = postTask(CmdType::FLUSH, [this, channel]() {
		int err = this->internalFlush(channel, true);
		if (err < 0)
			PDRAW_LOG_ERRNO("internalFlush", -err);
	});
	if (err < 0)
		PDRAW_LOG_ERRNO("postTask", -err);
}


/* Called on the loop thread */
void RecordMuxer::onChannelDrain(Channel *channel)
{
	PDRAW_CHECK_WRITER_THREAD(false);

	mAsyncFlush = canPostTask(false);

	if (!mAsyncFlush) {
		/* Thread is not running, call drain on the loop thread */
		Muxer::onChannelDrain(channel);
		return;
	}

	int err = postTask(CmdType::DRAIN, [this, channel]() {
		int err = this->internalFlush(channel, false);
		if (err < 0)
			PDRAW_LOG_ERRNO("internalFlush", -err);
	});
	if (err < 0)
		PDRAW_LOG_ERRNO("postTask", -err);
}


/* Called on the loop thread */
void RecordMuxer::onChannelSessionMetaUpdate(Channel *channel)
{
	size_t inputMediaCount;

	PDRAW_CHECK_WRITER_THREAD(false);

	Sink::onChannelSessionMetaUpdate(channel);

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	if (!canPostTask(true)) {
		PDRAW_LOGE("%s: cannot send command to thread", __func__);
		return;
	}

	Sink::lock();

	inputMediaCount = getInputMediaCount();
	for (unsigned int i = 0; i < inputMediaCount; i++) {
		const Media *media = getInputMedia(i);
		if (!media || getInputChannel(media) != channel)
			continue;

		const struct vmeta_session *sourceMeta = nullptr;
		auto *codedMedia = dynamic_cast<const CodedVideoMedia *>(media);
		auto *rawMedia = dynamic_cast<const RawVideoMedia *>(media);

		if (codedMedia)
			sourceMeta = &codedMedia->sessionMeta;
		else if (rawMedia)
			sourceMeta = &rawMedia->sessionMeta;
		else
			continue;

		uint32_t mediaId = media->id;

		try {
			auto metaPtr = std::make_shared<struct vmeta_session>(
				*sourceMeta);
			int err = postTask(CmdType::SET_METADATA,
					   [this, mediaId, metaPtr]() {
						   this->internalSetMetadata(
							   mediaId,
							   metaPtr.get());
					   });
			if (err < 0)
				PDRAW_LOG_ERRNO("postTask", -err);
		} catch (const std::bad_alloc &) {
			PDRAW_LOG_ERRNO("vmeta_session allocation failed",
					ENOMEM);
		}
	}

	Sink::unlock();
}


/* Called on the writer thread */
int RecordMuxer::internalFlush(Channel *channel, bool discard)
{
	int ret;

	PDRAW_CHECK_WRITER_THREAD(true);

	if (discard) {
		Muxer::onChannelFlush(channel);
	} else {
		/* Drain the queues */
		process();
		Muxer::onChannelDrain(channel);
	}

	ret = asyncCompleteFlush(channel, discard);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("asyncCompleteFlush", -ret);
		return ret;
	}
	return 0;
}


/* Must be called on the writer thread */
int RecordMuxer::process()
{
	int res = 0;

	PDRAW_CHECK_WRITER_THREAD(true);

	if (getThreadState() != State::STARTED)
		return 0;

	for (const auto &track : mMedias) {
		int trackRes;
		do {
			trackRes = track->process();
		} while (trackRes == 0);

		if (trackRes < 0 && trackRes != -EAGAIN)
			res = trackRes;
	}

	return res;
}


void RecordMuxer::writerThread()
{
	int err = 0;
	size_t inputMediaCount;

	setThreadState(State::STARTING);

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

	try {
		mMetaBuffer.resize(VMETA_FRAME_MAX_SIZE);
		mThread.loop = make_unique<pomp::Loop>();
	} catch (const std::bad_alloc &e) {
		err = -ENOMEM;
		PDRAW_LOGE("allocation failed: %s", e.what());
		goto out;
	}

	err = onWriterLoopInit();
	if (err < 0) {
		PDRAW_LOGE("onWriterLoopInit failed");
		goto out;
	}

	err = mThread.evt->attachToLoop(mThread.loop.get(),
					&mThread.evtHandler);
	if (err < 0)
		PDRAW_LOG_ERRNO("event::attachToLoop", -err);

	setThreadState(State::STARTED);

	while (!mThread.shouldStop) {
		err = mThread.loop->waitAndProcess(1000);
		if (err < 0 && err != -ETIMEDOUT)
			PDRAW_LOG_ERRNO("pomp_loop_wait_and_process", -err);
	}

	setThreadState(State::STOPPING);

	Sink::lock();

	inputMediaCount = getInputMediaCount();
	for (unsigned int i = 0; i < inputMediaCount; i++) {
		const Media *media = getInputMedia(i);
		if (media == nullptr) {
			PDRAW_LOGE("getInputMedia");
			continue;
		}

		Channel *channel = getInputChannel(media);
		if (channel == nullptr)
			continue;

		mbuf::Queue *queue = channel->getQueue(this);
		if (queue == nullptr)
			continue;

		channel->setQueue(this, nullptr);

		err = internalRemoveQueueEvtFromLoop(media->type, queue);
		if (err < 0) {
			PDRAW_LOG_ERRNO("internalRemoveQueueEvtFromLoop", -err);
		}
	}

	Sink::unlock();

out:
	(void)onWriterLoopCleanup();

	if ((mThread.evt != nullptr) && (mThread.loop != nullptr)) {
		{
			std::lock_guard<std::mutex> lock(mTasksMutex);
			mPendingTasks = {};
		}

		err = mThread.evt->detachFromLoop(mThread.loop.get());
		if (err < 0)
			ULOG_ERRNO("event::detachFromLoop", -err);

		mThread.evt.reset();
	}

	mThread.loop.reset();

	if (mState.load() == State::STOPPING) {
		/* Call completeStop on the loop thread */
		err = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), &callCompleteStop, this, this);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	}

	mMetaBuffer.clear();
	mMetaBuffer.shrink_to_fit();

	setThreadState(State::STOPPED);
}


/* Must be called on the writer thread */
void RecordMuxer::processTasks()
{
	PDRAW_CHECK_WRITER_THREAD(true);

	std::queue<RecordTask> tasks;

	{
		std::lock_guard<std::mutex> lock(mTasksMutex);
		tasks.swap(mPendingTasks);
	}

	while (!tasks.empty()) {
		RecordTask task = std::move(tasks.front());
		tasks.pop();
		if (task) {
			task();
		}
	}
}


/* Called on the writer thread */
int RecordMuxer::internalStopThread()
{
	int err;

	PDRAW_CHECK_WRITER_THREAD(true);

	onInternalStopThread();

	mThread.shouldStop = true;
	mStopThreadReceived = true;

	err = mThread.loop->wakeup();
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_wakeup", -err);

	return 0;
}


/* Must be called on the loop thread */
void RecordMuxer::callCompleteStop(void *userdata)
{
	auto *self = static_cast<RecordMuxer *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	PDRAW_CHECK_WRITER_THREAD_SELF(self, false);

	self->mPendingStop = false;
	idleCompleteStop(userdata);
}


/* Called from any thread */
Element::State RecordMuxer::getThreadState() const
{
	return mThread.state.load();
}


/* Called from any thread */
void RecordMuxer::setThreadState(Element::State state)
{
	Element::State old = mThread.state.exchange(state);

	if (old == state)
		return;

	PDRAW_LOGI("thread state change to %s", getElementStateStr(state));
}


/* Called from any thread */
bool RecordMuxer::isThreadAlive() const
{
	switch (getThreadState()) {
	case State::CREATED:
	case State::STARTING:
	case State::STARTED:
	case State::STOPPING:
		return true;
	case State::STOPPED:
	case State::INVALID:
	default:
		return false;
	}
}

} /* namespace Pdraw */
