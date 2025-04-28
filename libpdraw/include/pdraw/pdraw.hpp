/**
 * Parrot Drones Audio and Video Vector library
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

#ifndef _PDRAW_HPP_
#define _PDRAW_HPP_

#include <inttypes.h>

#include <string>

#include "pdraw_defs.h"

namespace Pdraw {


/* PDrAW object interface;
 * see the createPdraw() function for creating a PDrAW instance */
class IPdraw {
public:
	/* PDrAW listener object */
	class Listener {
	public:
		/**
		 * PDrAW listener object destructor.
		 */
		virtual ~Listener(void) {}

		/**
		 * Stop response function, called when a stop operation is
		 * complete or has failed. The status parameter is the stop
		 * operation status: 0 on success, or a negative errno value
		 * in case of error.
		 * @param pdraw: PDrAW instance handle
		 * @param status: 0 on success, negative errno value in case
		 *                of error
		 */
		virtual void stopResponse(IPdraw *pdraw, int status) = 0;

		/**
		 * Media added function, called when a media has been added
		 * internally in the PDrAW pipeline. Medias are for example
		 * raw or coded video medias. The info structure gives the
		 * media identifier that can be used for example to create a
		 * video sink on this media.
		 * @param pdraw: PDrAW instance handle
		 * @param info: pointer on the media information
		 * @param elementUserData: optional element user data pointer
		 *                         that corresponds to the pipeline
		 *                         element that created the media
		 */
		virtual void onMediaAdded(IPdraw *pdraw,
					  const struct pdraw_media_info *info,
					  void *elementUserData) = 0;

		/**
		 * Media removed function, called when a media has been removed
		 * internally from the PDrAW pipeline. Medias are for example
		 * raw or coded video medias. When a media is removed, any
		 * video sink created on this media must then be stopped.
		 * @warning when this function is called the pipeline element
		 * that created the media is likely being destroyed, therefore
		 * the elementUserData should not be casted to the pipeline
		 * element object.
		 * @param pdraw: PDrAW instance handle
		 * @param info: pointer on the media information
		 * @param elementUserData: optional element user data pointer
		 *                         that corresponds to the pipeline
		 *                         element that created the media
		 */
		virtual void onMediaRemoved(IPdraw *pdraw,
					    const struct pdraw_media_info *info,
					    void *elementUserData) = 0;

		/**
		 * Socket creation function, called immediately after a socket
		 * creation with its file descriptor as parameter.
		 * @param pdraw: PDrAW instance handle
		 * @param fd: socket file descriptor
		 */
		virtual void onSocketCreated(IPdraw *pdraw, int fd) = 0;
	};


	/**
	 * Instance management API
	 */

	/**
	 * Destroy a PDrAW instance.
	 * This function frees all resources associated with a PDrAW instance.
	 * The stop() function must be called prior to destroying and one must
	 * wait for the stopResponse() listener function to be called prior to
	 * destroying the PDrAW instance.
	 */
	virtual ~IPdraw(void) {}

	/**
	 * Stop a session.
	 * This function stops a session and all the associated objects.
	 * The function returns before the actual stopping is done. If the
	 * function returns 0, the stopResponse() listener function will be
	 * called once the stopping is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the stopResponse() listener function
	 * will not be called. After a successful stop, the PDrAW instance
	 * must be destroyed.
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int stop(void) = 0;


	/**
	 * Demuxer API
	 * @see the createDemuxer() function for the demuxer object creation
	 */
	class IDemuxer {
	public:
		/* Demuxer listener object */
		class Listener {
		public:
			/**
			 * Demuxer listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Open response function, called when an open operation
			 * is complete or has failed. The status parameter is
			 * the open operation status: 0 on success, or a
			 * negative errno value in case of error.
			 * If this function reports an error, the demuxer still
			 * needs to be closed: the close() function must be
			 * called and one must wait for the
			 * demuxerCloseResponse() listener function to be called
			 * prior to destroying the demuxer.
			 * @param pdraw: PDrAW instance handle
			 * @param demuxer: demuxer handle
			 * @param status: 0 on success, negative errno value in
			 *                case of error
			 */
			virtual void
			demuxerOpenResponse(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    int status) = 0;

			/**
			 * Close response function, called when a close
			 * operation is complete or has failed. The status
			 * parameter is the close operation status: 0 on
			 * success, or a negative errno value in case of error.
			 * @param pdraw: PDrAW instance handle
			 * @param demuxer: demuxer handle
			 * @param status: 0 on success, negative errno value in
			 *                case of error
			 */
			virtual void
			demuxerCloseResponse(IPdraw *pdraw,
					     IPdraw::IDemuxer *demuxer,
					     int status) = 0;

			/**
			 * Unrecoverable error function, called when a
			 * previously opened demuxer is no longer running.
			 * When this function is called, the demuxer is no
			 * longer running; the close() function must be called
			 * and one must wait for the demuxerCloseResponse()
			 * listener function to be called prior to destroying
			 * the demuxer.
			 * @param pdraw: PDrAW instance handle
			 * @param demuxer: demuxer handle
			 */
			virtual void onDemuxerUnrecoverableError(
				IPdraw *pdraw,
				IPdraw::IDemuxer *demuxer) = 0;

			/**
			 * Demuxer media selection function, called with a list
			 * of video medias found from which the application must
			 * choose one or more to process in the pipeline. The
			 * return value of the function must be a bitfield of
			 * the identifiers of the chosen medias (from the
			 * pdraw_demuxer_media structure), or 0 to choose the
			 * default medias. If the return value is -ENOSYS, the
			 * callback is considered not implemented and the
			 * default medias are chosen. If the return value is
			 * -ECANCELED no media is chosen and the open operation
			 * is aborted. If the return value is another negative
			 * errno or an invalid bitfield the
			 * demuxerOpenResponse() function will be called if an
			 * open operation is in progress, or the
			 * onDemuxerUnrecoverableError() function otherwise.
			 * @param pdraw: PDrAW instance handle
			 * @param demuxer: demuxer handle
			 * @param medias: array of demuxer media
			 * @param count: demuxer media array element count
			 * @param selectedMedias: bitfield of the identifiers of
			 *                        the currently selected medias
			 * @return a bitfield of the identifiers of the chosen
			 *         medias, 0 or -ENOSYS to choose the default
			 *         medias, -ECANCELED to choose no media and
			 *         abort the open operation, or another negative
			 *         errno value in case of error
			 */
			virtual int demuxerSelectMedia(
				IPdraw *pdraw,
				IPdraw::IDemuxer *demuxer,
				const struct pdraw_demuxer_media *medias,
				size_t count,
				uint32_t selectedMedias) = 0;

			/**
			 * Ready to play function, called when the playback is
			 * ready to start. This function is called to indicate
			 * that the demuxer is ready to process play operations.
			 * Generally the demuxer is ready to play as soon as the
			 * demuxerOpenResponse() function has been called with a
			 * success status. One case when the open operation was
			 * successful and the playback is not ready is when
			 * connected to a SkyController's RTSP server but the
			 * SkyController itself is not yet connected to a drone.
			 * Similarly, when connected to a drone's stream through
			 * a SkyController's RTSP server, if the drone is
			 * disconnected from the SkyController, this function
			 * will be called with a false value in the ready
			 * parameter.
			 * @param pdraw: PDrAW instance handle
			 * @param demuxer: demuxer handle
			 * @param ready: true if the session is ready to play,
			 *               false otherwise
			 */
			virtual void
			demuxerReadyToPlay(IPdraw *pdraw,
					   IPdraw::IDemuxer *demuxer,
					   bool ready) = 0;

			/**
			 * End of range function, called when the playback is
			 * suspended after having reached the end of the
			 * playback duration. This function is only called for
			 * replays (either local or streamed), not for live
			 * streams. The timestamp parameter is the current play
			 * time in microseconds at the moment the playback is
			 * suspended.
			 * @param pdraw: PDrAW instance handle
			 * @param demuxer: demuxer handle
			 * @param timestamp: current playback time in
			 *                   microseconds
			 */
			virtual void
			onDemuxerEndOfRange(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    uint64_t timestamp) = 0;

			/**
			 * Play response function, called when a play operation
			 * is complete (the playback has started) or has failed.
			 * The status parameter is the play operation status: 0
			 * on success, or a negative errno value in case of
			 * error. The timestamp parameter is the current play
			 * time in microseconds at the moment the playback is
			 * started. The speed parameter is the current playback
			 * speed; a negative value means playing backward.
			 * @param pdraw: PDrAW instance handle
			 * @param demuxer: demuxer handle
			 * @param status: 0 on success, negative errno value
			 *                in case of error
			 * @param timestamp: current playback time in
			 *                   microseconds
			 * @param speed: current playback speed, negative means
			 *               backward
			 */
			virtual void
			demuxerPlayResponse(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    int status,
					    uint64_t timestamp,
					    float speed) = 0;

			/**
			 * Pause response function, called when a pause
			 * operation is complete (the playback is suspended) or
			 * has failed. The status parameter is the pause
			 * operation status: 0 on success, or a negative errno
			 * value in case of error. The timestamp parameter is
			 * the current play time in microseconds at the moment
			 * the playback is paused.
			 * @param pdraw: PDrAW instance handle
			 * @param demuxer: demuxer handle
			 * @param status: 0 on success, negative errno value
			 *                in case of error
			 * @param timestamp: current playback time in
			 *                   microseconds
			 */
			virtual void
			demuxerPauseResponse(IPdraw *pdraw,
					     IPdraw::IDemuxer *demuxer,
					     int status,
					     uint64_t timestamp) = 0;

			/**
			 * Seek response function, called when a seek operation
			 * is complete or has failed. The status parameter is
			 * the seek operation status: 0 on success, or a
			 * negative errno value in case of error. The timestamp
			 * parameter is the current play time in microseconds
			 * after seeking. The speed parameter is the current
			 * playback speed; a negative value means playing
			 * backward.
			 * @param pdraw: PDrAW instance handle
			 * @param demuxer: demuxer handle
			 * @param status: 0 on success, negative errno value
			 *                in case of error
			 * @param timestamp: current playback time in
			 *                   microseconds
			 * @param speed: current playback speed, negative means
			 *               backward
			 */
			virtual void
			demuxerSeekResponse(IPdraw *pdraw,
					    IPdraw::IDemuxer *demuxer,
					    int status,
					    uint64_t timestamp,
					    float speed) = 0;
		};

		/**
		 * Destroy a demuxer.
		 * This function stops a running demuxer and frees the
		 * associated resources.
		 */
		virtual ~IDemuxer(void) {}

		/**
		 * Close a demuxer.
		 * This function closes a previously opened demuxer. The
		 * function returns before the actual closing is done. If the
		 * function returns 0, the demuxerCloseResponse() listener
		 * function will be called once the close is successful (0
		 * status) or has failed (negative errno status). If the
		 * function returns a negative errno value (immediate failure),
		 * the demuxerCloseResponse() listener function will not be
		 * called. After a successful close, the demuxer must be
		 * destroyed.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int close(void) = 0;

		/**
		 * Get the available demuxer media list.
		 * This function returns the media list. If no media are
		 * available, -ENOENT is returned. Otherwise, the mediaList is
		 * allocated (must be freed once no longer used) and mediaCount
		 * is set to the number of media.
		 * @param mediaList: pointer to an array of struct
		 *                   pdraw_demuxer_media (output, must be freed)
		 * @param mediaCount: pointer to the media count (output)
		 * @param selectedMedias: bitfield of the identifiers of the
		 *                        currently selected medias (output)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int getMediaList(struct pdraw_demuxer_media **mediaList,
					 size_t *mediaCount,
					 uint32_t *selectedMedias) = 0;

		/**
		 * Select media function.
		 * This function dynamically selects the media to use from a
		 * running demuxer. If the demuxer is opening or closing,
		 * -EPROTO is returned. The selectedMedias parameter is a
		 * bitfield of the identifiers of the chosen medias (from the
		 * pdraw_demuxer_media structure), or 0 to choose the default
		 * medias. If the bitfield is invalid, -EINVAL is returned.
		 * @param selectedMedias: bitfield of the identifiers of the
		 *                        chosen medias or 0 to choose the
		 *                        default medias
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int selectMedia(uint32_t selectedMedias) = 0;

		/**
		 * Get the single stream local stream port.
		 * This function returns the local stream (RTP) port currently
		 * in use after a succesful open operation on a single stream
		 * (RTP/AVP) or an RTSP URL. If no open operation has been done,
		 * or if an open operation has been done on a mux channel, 0 is
		 * returned. This is useful when opening a single stream with
		 * null local ports to let PDrAW open sockets on any available
		 * port.
		 * @return the stream port on success, 0 in case of error
		 */
		virtual uint16_t getSingleStreamLocalStreamPort(void) = 0;

		/**
		 * Get the single stream local control port.
		 * This function returns the local control (RTCP) port currently
		 * in use after a succesful open operation on a single stream
		 * (RTP/AVP) or an RTSP URL. If no open operation has been done,
		 * or if an open operation has been done on a mux channel, 0 is
		 * returned. This is useful when opening a single stream with
		 * null local ports to let PDrAW open sockets on any available
		 * port.
		 * @return the stream port on success, 0 in case of error
		 */
		virtual uint16_t getSingleStreamLocalControlPort(void) = 0;

		/**
		 * Get the ready to play status.
		 * This function returns true if a successful open operation has
		 * been completed and if the playback is ready to start, false
		 * otherwise. One case when a successful open operation is
		 * complete but the playback is not ready is when connected to a
		 * SkyController's RTSP server but the SkyController itself is
		 * not yet connected to a drone. The value returned by this
		 * function is identical to the ready parameter passed to the
		 * readyToPlay() listener function when it is called.
		 * @return the ready to play status on success, false in case of
		 *         error
		 */
		virtual bool isReadyToPlay(void) = 0;

		/**
		 * Get the pause status.
		 * This function returns true if the playback is currently
		 * paused, false otherwise.
		 * @return the pause status on success, false in case of error
		 */
		virtual bool isPaused(void) = 0;

		/**
		 * Play at the given speed.
		 * This function starts the playback of the video. The speed
		 * parameter is optional and defaults to 1.0. If the speed
		 * parameter is negative, the video is played backward. If the
		 * speed is greater than or equal to PDRAW_PLAY_SPEED_MAX, the
		 * speed is ignored and the video is played at the maximum speed
		 * achievable. If the speed is less than or equal to
		 * -PDRAW_PLAY_SPEED_MAX, the speed is ignored and the video is
		 * played backward at the maximum speed achievable. A 0.0 speed
		 * has the same effet as calling the pause() function. On a live
		 * stream, the speed parameter has no effect. The function
		 * returns before the actual operation is done. If the function
		 * returns 0, the demuxerPlayResponse() listener function will
		 * be called once the play is successful (0 status) or has
		 * failed (negative errno status). If the function returns a
		 * negative errno value (immediate failure), the
		 * demuxerPlayResponse() listener function will not be called.
		 * @param speed: playback speed (0.0 means pause, negative value
		 *               means play backward)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int play(float speed = 1.0f) = 0;

		/**
		 * Pause the playback.
		 * This function suspends the playback of the video. The session
		 * is not closed and the playback can be resumed using the
		 * play() function. The function returns before the actual
		 * operation is done. If the function returns 0, the
		 * demuxerPauseResponse() listener function will be called once
		 * the pause is successful (0 status) or has failed (negative
		 * errno status). If the function returns a negative errno value
		 * (immediate failure), the demuxerPauseResponse() listener
		 * function will not be called.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int pause(void) = 0;

		/**
		 * Go to previous frame in frame-by-frame playback.
		 * This function plays the previous frame while the playback is
		 * paused. If the playback is not currently paused an error is
		 * returned. Frame-by-frame is only available on local replays
		 * (MP4 records). The function returns before the actual
		 * operation is done. If the function returns 0, the
		 * demuxerSeekResponse() listener function will be called once
		 * the seek is successful (0 status) or has failed (negative
		 * errno status). If the function returns a negative errno value
		 * (immediate failure), the demuxerSeekResponse() listener
		 * function will not be called.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int previousFrame(void) = 0;

		/**
		 * Go to next frame in frame-by-frame playback.
		 * This function plays the next frame while the playback is
		 * paused. If the playback is not currently paused an error is
		 * returned. Frame-by-frame is only available on local replays
		 * (MP4 records). The function returns before the actual
		 * operation is done. If the function returns 0, the
		 * demuxerSeekResponse() listener function will be called once
		 * the seek is successful (0 status) or has failed (negative
		 * errno status). If the function returns a negative errno value
		 * (immediate failure), the demuxerSeekResponse() listener
		 * function will not be called.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int nextFrame(void) = 0;

		/**
		 * Seek forward or backward.
		 * This function seeks forward (positive delta) or backward
		 * (negative delta). The delta parameter is in microseconds.
		 * When exact is false the seek is done to the nearest
		 * synchronization sample preceeding the delta, otherwise the
		 * seek is done to the sample nearest to the delta. Seeking is
		 * only available on replays (either local or streamed), not on
		 * live streams. The function returns before the actual
		 * operation is done. If the function returns 0, the
		 * demuxerSeekResponse() listener function will be called once
		 * the seek is successful (0 status) or has failed (negative
		 * errno status). If the function returns a negative errno value
		 * (immediate failure), the demuxerSeekResponse() listener
		 * function will not be called.
		 * @param delta: time delta in microseconds (positive or
		 *               negative)
		 * @param exact: true means seek to the sample closest to the
		 *               delta, false means seek to the nearest
		 *               synchronization sample preceeding the delta
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int seek(int64_t delta, bool exact = false) = 0;

		/**
		 * Seek forward.
		 * This function seeks forward by delta microseconds. It has the
		 * same behavior as calling seek() with a positive delta. When
		 * exact is false the seek is done to the nearest
		 * synchronization sample preceeding the delta, otherwise the
		 * seek is done to the sample nearest to the delta. Seeking is
		 * only available on replays (either local or streamed), not on
		 * live streams. The function returns before the actual
		 * operation is done. If the function returns 0, the
		 * demuxerSeekResponse() listener function will be called once
		 * the seek is successful (0 status) or has failed (negative
		 * errno status). If the function returns a negative errno value
		 * (immediate failure), the demuxerSeekResponse() listener
		 * function will not be called.
		 * @param delta: positive time delta forward in microseconds
		 * @param exact: true means seek to the sample closest to the
		 *               delta, false means seek to the nearest
		 *               synchronization sample preceeding the delta
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int seekForward(uint64_t delta, bool exact = false) = 0;

		/**
		 * Seek backward.
		 * This function seeks backward by delta microseconds (postive).
		 * It has the same behavior as calling seek() with a negative
		 * delta. When exact is false the seek is done to the nearest
		 * synchronization sample preceeding the delta, otherwise the
		 * seek is done to the sample nearest to the delta. Seeking is
		 * only available on replays (either local or streamed), not on
		 * live streams. The function returns before the actual
		 * operation is done. If the function returns 0, the
		 * demuxerSeekResponse() listener function will be called once
		 * the seek is successful (0 status) or has failed (negative
		 * errno status). If the function returns a negative errno value
		 * (immediate failure), the demuxerSeekResponse() listener
		 * function will not be called.
		 * @param delta: positive time delta backward in microseconds
		 * @param exact: true means seek to the sample closest to the
		 *               delta, false means seek to the nearest
		 *               synchronization sample preceeding the delta
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int seekBack(uint64_t delta, bool exact = false) = 0;

		/**
		 * Seek to a given time.
		 * This function seeks to the given play timestamp in
		 * microseconds. When exact is false the seek is done to the
		 * nearest synchronization sample preceeding the timestamp,
		 * otherwise the seek is done to the sample nearest to the
		 * timestamp. Seeking is only available on replays (either local
		 * or streamed), not on live streams. The function returns
		 * before the actual operation is done. If the function returns
		 * 0, the demuxerSeekResponse() listener function will be called
		 * once the seek is successful (0 status) or has failed
		 * (negative errno status). If the function returns a negative
		 * errno value (immediate failure), the demuxerSeekResponse()
		 * listener function will not be called.
		 * @param timestamp: play timestamp in microseconds
		 * @param exact: true means seek to the sample closest to the
		 *               timestamp, false means seek to the nearest
		 *               synchronization sample preceeding the timestamp
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int seekTo(uint64_t timestamp, bool exact = false) = 0;

		/**
		 * Get the video chapter list.
		 * This function returns the video chapter list if available. If
		 * the video does not contain any chapter, -ENOENT is returned.
		 * Otherwise, the chapterList is allocated (must be freed
		 * once no longer used) and chapterCount is set to the number of
		 * chapters. The chapter timestamps can be used to seek to the
		 * desired chapter. This function is available on a record
		 * demuxer only; on any other type of muxer -ENOSYS is returned.
		 * @param chapterList: pointer to an array of struct
		 *                     pdraw_chapter (output, must be freed)
		 * @param chapterCount: pointer to the chapter count (output)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int getChapterList(struct pdraw_chapter **chapterList,
					   size_t *chapterCount) = 0;

		/**
		 * Get the playback duration.
		 * This function returns the playback duration in microseconds.
		 * The duration is only available on replays (either local or
		 * streamed), not on live streams.
		 * @return the duration in microseconds on success,
		 *         0 in case of error
		 */
		virtual uint64_t getDuration(void) = 0;

		/**
		 * Get the playback current time.
		 * This function returns the current playback position in
		 * microseconds. On replays (either local or streamed) this is
		 * the position between 0 and the duration; on live streams this
		 * is the time since the start of the stream session.
		 * @return the current time in microseconds on success,
		 *         0 in case of error
		 */
		virtual uint64_t getCurrentTime(void) = 0;
	};

	/**
	 * Create a demuxer on a URL (stream or local file).
	 * The URL can be either an RTSP URL (starting with "rtsp://") or a
	 * local file path (either absolute or relative).
	 * The params structure must be provided but all parameters are optional
	 * and can be left null. The function returns before the actual opening
	 * is done. If the function returns 0, the demuxerOpenResponse()
	 * listener function will be called once the open operation is
	 * successful (0 status) or has failed (negative errno status). If the
	 * function returns a negative errno value (immediate failure), the
	 * demuxerOpenResponse() listener function will not be called. Once a
	 * demuxer is no longer used, it must be closed and then destroyed (@see
	 * the IDemuxer::close() function).
	 * @param url: URL of the resource to open
	 * @param params: demuxer parameters
	 * @param listener: demuxer listener functions implementation
	 * @param retObj: demuxer object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int createDemuxer(const std::string &url,
				  const struct pdraw_demuxer_params *params,
				  IPdraw::IDemuxer::Listener *listener,
				  IPdraw::IDemuxer **retObj) = 0;

	/**
	 * Create a demuxer on a single stream.
	 * This function opens an RTP/AVP stream. No session management is done:
	 * it is the application's responsibility to handle the ports
	 * negociation with the sender. If null local ports are given as
	 * parameter, the effective port numbers used can be retrieved using the
	 * getSingleStreamLocalStreamPort() for the RTP port and
	 * getSingleStreamLocalControlPort() for the RTCP port functions.
	 * If the localAddr parameter is left empty, any local network interface
	 * will be used. The remoteAddr, remoteStreamPort and remoteControlPort
	 * parameters can be left null/empty if unknown; they will be known once
	 * the stream is being received.
	 * The params structure must be provided but all parameters are optional
	 * and can be left null. The function returns before the actual opening
	 * is done. If the function returns 0, the demuxerOpenResponse()
	 * listener function will be called once the open operation is
	 * successful (0 status) or has failed (negative errno status). If the
	 * function returns a negative errno value (immediate failure), the
	 * demuxerOpenResponse() listener function will not be called. Once a
	 * demuxer is no longer used, it must be closed and then destroyed (@see
	 * the IDemuxer::close() function).
	 * @param localAddr: local IP address (optional, can be empty)
	 * @param localStreamPort: local stream (RTP) port (optional, can be 0)
	 * @param localControlPort: local control (RTCP) port (optional,
	 *                          can be 0)
	 * @param remoteAddr: remote IP address (optional, can be empty)
	 * @param remoteStreamPort: remote stream (RTP) port (optional,
	 *                          can be 0)
	 * @param remoteControlPort: remote control (RTCP) port (optional,
	 *                           can be 0)
	 * @param params: demuxer parameters
	 * @param listener: demuxer listener functions implementation
	 * @param retObj: demuxer object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int createDemuxer(const std::string &localAddr,
				  uint16_t localStreamPort,
				  uint16_t localControlPort,
				  const std::string &remoteAddr,
				  uint16_t remoteStreamPort,
				  uint16_t remoteControlPort,
				  const struct pdraw_demuxer_params *params,
				  IPdraw::IDemuxer::Listener *listener,
				  IPdraw::IDemuxer **retObj) = 0;

	/**
	 * Create a demuxer on a stream URL through a mux channel.
	 * The URL must be an RTSP URL (starting with "rtsp://"). Mux channels
	 * are used to transfer data between a SkyController remote and a
	 * smartphone through USB; see Parrot's libmux for more information.
	 * No concurrent sessions can run on the mux channel; therefore the user
	 * must take care of limiting the number of PDrAW instances and demuxer
	 * objects running on the mux channel to only one.
	 * The params structure must be provided but all parameters are optional
	 * and can be left null. The function returns before the actual opening
	 * is done. If the function returns 0, the demuxerOpenResponse()
	 * listener function will be called once the open operation is
	 * successful (0 status) or has failed (negative errno status). If the
	 * function returns a negative errno value (immediate failure), the
	 * demuxerOpenResponse() listener function will not be called. Once a
	 * demuxer is no longer used, it must be closed and then destroyed (@see
	 * the IDemuxer::close() function).
	 * @param url: URL of the resource to open
	 * @param mux: mux instance handle
	 * @param params: demuxer parameters
	 * @param listener: demuxer listener functions implementation
	 * @param retObj: demuxer object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int createDemuxer(const std::string &url,
				  struct mux_ctx *mux,
				  const struct pdraw_demuxer_params *params,
				  IPdraw::IDemuxer::Listener *listener,
				  IPdraw::IDemuxer **retObj) = 0;


	/**
	 * Muxer API
	 * @see the createMuxer() function for the muxer object creation
	 */
	class IMuxer {
	public:
		/* Muxer listener object */
		class Listener {
		public:
			/**
			 * Muxer listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Connection state changed function, called when a
			 * muxer connection state has changed.
			 * This function is called on a stream muxer only;
			 * on any other type of muxer it is not relevant.
			 * @param pdraw: PDrAW instance handle
			 * @param muxer: muxer handle
			 * @param connectionState: connection state
			 * @param disconnectionReason: disconnection reason;
			 *                             only relevant when
			 *                             connectionState is
			 *                             DISCONNECTED.
			 */
			virtual void onMuxerConnectionStateChanged(
				IPdraw *pdraw,
				IPdraw::IMuxer *muxer,
				enum pdraw_muxer_connection_state
					connectionState,
				enum pdraw_muxer_disconnection_reason
					disconnectionReason) = 0;

			/**
			 * Unrecoverable error function, called when a
			 * previously opened muxer is no longer running.
			 * When this function is called, the muxer is no longer
			 * running; the close() function must be called and one
			 * must wait for the muxerCloseResponse() listener
			 * function to be called prior to destroying the muxer.
			 * @param pdraw: PDrAW instance handle
			 * @param muxer: muxer handle
			 * @param status: error status code
			 */
			virtual void
			onMuxerUnrecoverableError(IPdraw *pdraw,
						  IPdraw::IMuxer *muxer,
						  int status) = 0;

			/**
			 * Close response function, called when a close
			 * operation is complete or has failed. The status
			 * parameter is the close operation status: 0 on
			 * success, or a negative errno value in case of error.
			 * @param pdraw: PDrAW instance handle
			 * @param muxer: muxer handle
			 * @param status: 0 on success, negative errno value in
			 *                case of error
			 */
			virtual void muxerCloseResponse(IPdraw *pdraw,
							IPdraw::IMuxer *muxer,
							int status) = 0;
		};

		/**
		 * Destroy a muxer.
		 * This function stops a running muxer and frees the
		 * associated resources.
		 */
		virtual ~IMuxer(void) {}

		/**
		 * Close a muxer.
		 * This function closes a previously opened muxer. The
		 * function returns before the actual closing is done. If the
		 * function returns 0, the muxerCloseResponse() listener
		 * function will be called once the close is successful (0
		 * status) or has failed (negative errno status). If the
		 * function returns a negative errno value (immediate failure),
		 * the muxerCloseResponse() listener function will not be
		 * called. After a successful close, the muxer must be
		 * destroyed.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int close(void) = 0;

		/**
		 * Add a media to a muxer.
		 * This function adds a media to the muxer by its mediaId.
		 * The media idenfifiers are known when the onMediaAdded() or
		 * onMediaRemoved() general listener functions are called.
		 * The params structure is optional.
		 * @param mediaId: identifier of the media to add to the muxer
		 * @param params: muxer media parameters
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int
		addMedia(unsigned int mediaId,
			 const struct pdraw_muxer_media_params *params) = 0;

		/**
		 * Set the thumbnail of the MP4 file written by the muxer.
		 * This function is available on a record muxer only; on any
		 * other type of muxer -ENOSYS is returned.
		 * @param type: type of the thumbnail (JPEG, PNG, etc...)
		 * @param data: thumbnail data must be of length size
		 * @param size: thumbnail data size
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int setThumbnail(enum pdraw_muxer_thumbnail_type type,
					 const uint8_t *data,
					 size_t size) = 0;

		/**
		 * Add a chapter to the MP4 file written by the muxer.
		 * This function is available on a record muxer only; on any
		 * other type of muxer -ENOSYS is returned.
		 * The timestamp is expressed in microseconds and is relative to
		 * the start of the record. If no zero-timestamped chapter is
		 * set, a default chapter named 'Start' will be added as the
		 * first chapter.
		 * @param timestamp: timestamp of the chapter in microseconds
		 * @param name: name of the chapter
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int addChapter(uint64_t timestamp,
				       const char *name) = 0;

		/**
		 * Get statistics about the muxer.
		 * This function fills the stats structure with the latest muxer
		 * statistics. The structure must have been previously
		 * allocated.
		 * @param stats: muxer statistics structure to fill
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int getStats(struct pdraw_muxer_stats *stats) = 0;

		/**
		 * Set the muxer dynamic parameters.
		 * This function is available on a record muxer only; on any
		 * other type of muxer -ENOSYS is returned.
		 * @param dyn_params: dynamic parameters
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int setDynParams(
			const struct pdraw_muxer_dyn_params *dyn_params) = 0;

		/**
		 * Get the muxer dynamic parameters.
		 * This function is available on a record muxer only; on any
		 * other type of muxer -ENOSYS is returned.
		 * @param dyn_params: dynamic parameters structure to fill
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int
		getDynParams(struct pdraw_muxer_dyn_params *dyn_params) = 0;

		/**
		 * Force the muxer to write the MP4 tables.
		 * This function is available on a record muxer only; on any
		 * other type of muxer -ENOSYS is returned.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int forceSync(void) = 0;
	};

	/**
	 * Create a muxer.
	 * This function creates a muxer with a given URL. The url parameter is
	 * either an RTMP URL or a path to an MP4 file (the file will be
	 * created/overwritten). Once the muxer is created medias can be added
	 * by id using the addMedia() function. Once a muxer is no longer used,
	 * it must be destroyed. If writing to an MP4 file, the file is
	 * finalized in the destructor. The params structure must be provided
	 * but all parameters are optional and can be left null. The listener
	 * must be provided; all listener functions are called from the
	 * pomp_loop thread.
	 * @param url: destination URL
	 * @param params: muxer parameters
	 * @param listener: muxer listener functions implementation
	 * @param retObj: muxer object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int createMuxer(const std::string &url,
				const struct pdraw_muxer_params *params,
				IPdraw::IMuxer::Listener *listener,
				IPdraw::IMuxer **retObj) = 0;


	/**
	 * Video renderer API
	 * @warning all functions must be called from the application's
	 * rendering thread
	 * @see the createVideoRenderer() function for the video renderer
	 * object creation
	 */
	class IVideoRenderer {
	public:
		/* Video renderer listener object */
		class Listener {
		public:
			/**
			 * Video renderer listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Media added function, called when a media has been
			 * added internally to the renderer. Medias are raw
			 * video medias. This function is called from the
			 * pomp_loop thread.
			 * @param pdraw: PDrAW instance handle
			 * @param renderer: renderer handle
			 * @param info: pointer on the media information
			 */
			virtual void onVideoRendererMediaAdded(
				IPdraw *pdraw,
				IPdraw::IVideoRenderer *renderer,
				const struct pdraw_media_info *info) = 0;

			/**
			 * Media removed function, called when a media has been
			 * removed internally from the renderer. Medias are raw
			 * video medias. This function is called from the
			 * pomp_loop thread.
			 * @param pdraw: PDrAW instance handle
			 * @param renderer: renderer handle
			 * @param info: pointer on the media information
			 * @param restart: true if a new media should follow
			 *                 shortly (reconfiguration,
			 *                 resolution change...)
			 */
			virtual void onVideoRendererMediaRemoved(
				IPdraw *pdraw,
				IPdraw::IVideoRenderer *renderer,
				const struct pdraw_media_info *info,
				bool restart) = 0;

			/**
			 * Render ready function, called both when a new frame
			 * is ready for rendering and periodically. This
			 * function is called from the pomp_loop thread.
			 * @param pdraw: PDrAW instance handle
			 * @param renderer: renderer handle
			 */
			virtual void onVideoRenderReady(
				IPdraw *pdraw,
				IPdraw::IVideoRenderer *renderer) = 0;

			/**
			 * External texture loading function. This function is
			 * called before the rendering of the video frame in
			 * order to override the frame loading as a texture.
			 * This can be used to transform the video frames
			 * outside of PDrAW before resuming the rendering. This
			 * function is called from the rendering thread. If no
			 * implementation of this function is required by the
			 * application, -ENOSYS must be returned (before
			 * checking input values, so that implementation can be
			 * tested with all arguments null).
			 * @param pdraw: PDrAW instance handle
			 * @param renderer: renderer handle
			 * @param textureWidth: texture width in pixels
			 * @param textureHeight: texture height in pixels
			 * @param mediaInfo: media information
			 * @param frame: frame information
			 * @param frameUserdata: frame user data buffer
			 * @param frameUserdataLen: frame user data buffer size
			 *                          in bytes
			 * @return 0 on success, -ENOSYS if not implemented,
			 *         or another negative errno value in case of
			 *         error
			 */
			virtual int loadVideoTexture(
				IPdraw *pdraw,
				IPdraw::IVideoRenderer *renderer,
				unsigned int textureWidth,
				unsigned int textureHeight,
				const struct pdraw_media_info *mediaInfo,
				struct mbuf_raw_video_frame *frame,
				const void *frameUserdata,
				size_t frameUserdataLen) = 0;

			/**
			 * Overlay rendering function. This function is called
			 * after the rendering of the video frame (if one is
			 * available) in order to render an application overlay
			 * on top of the video. When no frame is available for
			 * the rendering, the frameMeta and frameExtra
			 * parameters are null. This function is called from
			 * the rendering thread. If no implementation of this
			 * function is required by the application, -ENOSYS
			 * must be returned (before checking input values, so
			 * that implementation can be tested with all arguments
			 * null).
			 * @param pdraw: PDrAW instance handle
			 * @param renderer: renderer handle
			 * @param renderPos: rendering position
			 * @param contentPos: video content position
			 * @param viewMat: 4x4 view matrix
			 * @param projMat: 4x4 projection matrix
			 * @param mediaInfo: media information (optional,
			 *                   can be null)
			 * @param frameMeta: frame metadata (optional,
			 *                   can be null)
			 * @param frameExtra: frame extra information
			 *                    (optional, can be null)
			 * @return 0 on success, -ENOSYS if not implemented,
			 *         or another negative errno value in case of
			 *         error
			 */
			virtual int renderVideoOverlay(
				IPdraw *pdraw,
				IPdraw::IVideoRenderer *renderer,
				const struct pdraw_rect *renderPos,
				const struct pdraw_rect *contentPos,
				const float *viewMat,
				const float *projMat,
				const struct pdraw_media_info *mediaInfo,
				struct vmeta_frame *frameMeta,
				const struct pdraw_video_frame_extra
					*frameExtra) = 0;
		};

		/**
		 * Destroy a video renderer.
		 * This function stops a running video renderer and frees the
		 * associated resources.
		 * @warning this function must be called from the application's
		 * rendering thread.
		 */
		virtual ~IVideoRenderer(void) {}

		/**
		 * Resize a video renderer.
		 * This function updates the rendering position and size on a
		 * running video renderer. The render_pos parameter sets the
		 * position of the rendering in the window/view; these
		 * coordinates are in pixels from the bottom-left corner (OpenGL
		 * coordinates).
		 * @warning this function must be called from the application's
		 * rendering thread.
		 * @param renderPos: rendering position and size
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int resize(const struct pdraw_rect *renderPos) = 0;

		/**
		 * Set the video renderer media identifier.
		 * This function updates the identifier of the media on which
		 * the rendering is done; if the media id is zero the first raw
		 * media encountered is used.
		 * @warning this function must be called from the application's
		 * rendering thread.
		 * @param mediaId: identifier of the raw media to render (from
		 *                 a pdraw_media_info structure); if zero the
		 *                 first raw media found is used for rendering
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int setMediaId(unsigned int mediaId) = 0;

		/**
		 * Get the video renderer media identifier.
		 * This function retrieves the identifier of the media on which
		 * the rendering is done.
		 * @warning this function must be called from the application's
		 * rendering thread.
		 * @return the identifier of the media on success,
		 *         0 if no media is being renderered or in case of error
		 */
		virtual unsigned int getMediaId(void) = 0;

		/**
		 * Set the video renderer parameters.
		 * This function updates the rendering parameters on a running
		 * video renderer. The params structure must be provided but all
		 * parameters are optional and can be left null.
		 * @warning this function must be called from the application's
		 * rendering thread.
		 * @param params: renderer parameters
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int
		setParams(const struct pdraw_video_renderer_params *params) = 0;

		/**
		 * Get the video renderer parameters.
		 * This function retrieves the rendering parameters on a running
		 * video renderer. The provided params structure is filled by
		 * the function.
		 * @warning this function must be called from the application's
		 * rendering thread.
		 * @param params: renderer parameters (output)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int
		getParams(struct pdraw_video_renderer_params *params) = 0;

		/**
		 * Render the video.
		 * This function renders the video with the current rendering
		 * position and rendering parameters, and optionally with the
		 * provided view and projection matrices. The viewMat and
		 * projMat matrices are 4x4 OpenGL matrices. For
		 * render-on-demand, consider using the onVideoRenderReady()
		 * listener function which is called both when a new frame is
		 * ready for rendering and periodically. Note that the
		 * onVideoRenderReady() listener function is called from the
		 * pomp_loop thread, not the rendering thread; the
		 * synchronization is up to the caller. If a contentPos
		 * structure is provided, it is filled with the actual position
		 * and size of the video within the rendering position; these
		 * coordinates are in pixels from the bottom-left corner (OpenGL
		 * coordinates).
		 * @warning this function must be called from the application's
		 * rendering thread.
		 * @param contentPos: video content position (output;
		 *                    optional, can be null)
		 * @param viewMat: 4x4 view matrix (optional, can be null)
		 * @param projMat: 4x4 projection matrix (optional, can be null)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int render(struct pdraw_rect *contentPos,
				   const float *viewMat = nullptr,
				   const float *projMat = nullptr) = 0;
	};

	/**
	 * Create a video renderer.
	 * This function creates a video renderer on a media of the given media
	 * id; if the media id is zero the first raw media encountered is used.
	 * Optionally an active EGL display context can be provided. Once the
	 * renderer is created, the rendering is done by calling the render()
	 * function. Once a renderer is no longer used it must be
	 * destroyed. The renderPos parameter sets the position and size
	 * of the rendering in the window/view; these coordinates are in
	 * pixels from the bottom-left corner (OpenGL coordinates). The
	 * params structure must be provided but all parameters are
	 * optional and can be left null. A valid listener must be
	 * provided and all functions must be implemented; the
	 * loadVideoTexture() and renderVideoOverlay() are optional and
	 * can return -ENOSYS if no real implementation is provided. All
	 * listener functions are called from the rendering thread,
	 * except the onVideoRenderReady() function which is called from
	 * the pomp_loop thread.
	 * @warning this function must be called from the application's
	 * rendering thread.
	 * @param mediaId: identifier of the raw media to render (from a
	 *                 pdraw_media_info structure); if zero the first
	 *                 raw media found is used for rendering
	 * @param renderPos: rendering position and size
	 * @param params: renderer parameters
	 * @param listener: renderer listener functions implementation
	 * @param retObj: renderer object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createVideoRenderer(unsigned int mediaId,
			    const struct pdraw_rect *renderPos,
			    const struct pdraw_video_renderer_params *params,
			    IPdraw::IVideoRenderer::Listener *listener,
			    IPdraw::IVideoRenderer **retObj) = 0;


	/**
	 * Audio renderer API
	 * @see the createAudioRenderer() function for the audio renderer
	 * object creation
	 */
	class IAudioRenderer {
	public:
		/* Audio renderer listener object */
		class Listener {
		public:
			/**
			 * Audio renderer listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Media added function, called when a media has been
			 * added internally to the renderer. Medias are audio
			 * medias. This function is called from the pomp_loop
			 * thread.
			 * @param pdraw: PDrAW instance handle
			 * @param renderer: renderer handle
			 * @param info: pointer on the media information
			 */
			virtual void onAudioRendererMediaAdded(
				IPdraw *pdraw,
				IPdraw::IAudioRenderer *renderer,
				const struct pdraw_media_info *info) = 0;

			/**
			 * Media removed function, called when a media has been
			 * removed internally from the renderer. Medias are
			 * audio medias. This function is called from the
			 * pomp_loop thread.
			 * @param pdraw: PDrAW instance handle
			 * @param renderer: renderer handle
			 * @param info: pointer on the media information
			 */
			virtual void onAudioRendererMediaRemoved(
				IPdraw *pdraw,
				IPdraw::IAudioRenderer *renderer,
				const struct pdraw_media_info *info) = 0;
		};

		/**
		 * Destroy an audio renderer.
		 * This function stops a running audio renderer and frees the
		 * associated resources.
		 */
		virtual ~IAudioRenderer(void) {}

		/**
		 * Set the audio renderer media identifier.
		 * This function updates the identifier of the media on which
		 * the rendering is done; if the media id is zero the first
		 * audio media encountered is used.
		 * @param mediaId: identifier of the audio media to render (from
		 *                 a pdraw_media_info structure); if zero the
		 *                 first audio media found is used for rendering
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int setMediaId(unsigned int mediaId) = 0;

		/**
		 * Get the audio renderer media identifier.
		 * This function retrieves the identifier of the media on which
		 * the rendering is done.
		 * @return the identifier of the media on success,
		 *         0 if no media is being renderered or in case of error
		 */
		virtual unsigned int getMediaId(void) = 0;

		/**
		 * Set the audio renderer parameters.
		 * This function updates the rendering parameters on a running
		 * audio renderer. The params structure must be provided but all
		 * parameters are optional and can be left null.
		 * @param params: renderer parameters
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int
		setParams(const struct pdraw_audio_renderer_params *params) = 0;

		/**
		 * Get the audio renderer parameters.
		 * This function retrieves the rendering parameters on a running
		 * audio renderer. The provided params structure is filled by
		 * the function.
		 * @param params: renderer parameters (output)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int
		getParams(struct pdraw_audio_renderer_params *params) = 0;
	};

	/**
	 * Create an audio renderer.
	 * This function creates an audio renderer on a media of the given media
	 * id; if the media id is zero the first audio media encountered is
	 * used. Once the renderer is created, the rendering is done internally.
	 * Once a renderer is no longer used it must be destroyed. A valid
	 * listener must be provided and all functions must be implemented. All
	 * listener functions are called from the pomp_loop thread.
	 * @param mediaId: identifier of the audio media to render (from a
	 *                 pdraw_media_info structure); if zero the first
	 *                 audio media found is used for rendering
	 * @param params: renderer parameters
	 * @param listener: renderer listener functions implementation
	 * @param retObj: renderer object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createAudioRenderer(unsigned int mediaId,
			    const struct pdraw_audio_renderer_params *params,
			    IPdraw::IAudioRenderer::Listener *listener,
			    IPdraw::IAudioRenderer **retObj) = 0;


	/**
	 * Video IPC source API
	 * @see the createVipcSource() function for the video IPC source
	 * object creation
	 */
	class IVipcSource {
	public:
		/* Video IPC source listener object */
		class Listener {
		public:
			/**
			 * Video IPC source listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Ready to play function, called when the video IPC is
			 * ready to start receiving frames (the ready parameter
			 * is true), or when the video IPC cannot receive frames
			 * any more (end of stream; the ready parameter is
			 * false). When the ready parameter is false, the
			 * eosReason parameter indicates the reason why the end
			 * of stream was received on the video IPC.
			 * @param pdraw: PDrAW instance handle
			 * @param source: video IPC source handle
			 * @param ready: true if the session is ready to play,
			 *               false otherwise
			 * @param eosReason: end of stream reason if the ready
			 *                   parameter is false
			 */
			virtual void
			vipcSourceReadyToPlay(IPdraw *pdraw,
					      IPdraw::IVipcSource *source,
					      bool ready,
					      enum pdraw_vipc_source_eos_reason
						      eosReason) = 0;

			/**
			 * Framerate changed function, called when the video IPC
			 * framerate has changed (new status). The return value
			 * is a boolean indicating whether the framerate change
			 * must be ignored or not. If 'false', the current media
			 * is destroyed and a new media is created.
			 * @param pdraw: PDrAW instance handle
			 * @param source: video IPC source handle
			 * @param prevFramerate the previous framerate,
			 * @param newFramerate the new framerate.
			 * @return true to ignore the framerate change, false to
			 * re-create a media.
			 */
			virtual bool vipcSourceFramerateChanged(
				IPdraw *pdraw,
				IPdraw::IVipcSource *source,
				const struct vdef_frac *prevFramerate,
				const struct vdef_frac *newFramerate) = 0;

			/**
			 * Configured function, called when a video IPC has
			 * been configured (initially or reconfigured) or when
			 * a configuration has failed. The status parameter is
			 * the configuration operation status: 0 on success,
			 * or a negative errno value in case of error. The info
			 * and crop parameters are the current video IPC
			 * configuration.
			 * @param pdraw: PDrAW instance handle
			 * @param source: video IPC source handle
			 * @param status: 0 on success, negative errno value
			 *                in case of error
			 * @param info: current video format info
			 * @param crop: current crop configuration
			 */
			virtual void vipcSourceConfigured(
				IPdraw *pdraw,
				IPdraw::IVipcSource *source,
				int status,
				const struct vdef_format_info *info,
				const struct vdef_rectf *crop) = 0;

			/**
			 * Frame ready function, called when a video frame has
			 * been received and before it is propagated downstream
			 * in the pipeline. This can be used to associate
			 * metadata with the frame.
			 * @param pdraw: PDrAW instance handle
			 * @param source: video IPC source handle
			 * @param frame: frame information
			 */
			virtual void vipcSourceFrameReady(
				IPdraw *pdraw,
				IPdraw::IVipcSource *source,
				struct mbuf_raw_video_frame *frame) = 0;

			/**
			 * End-of-stream function, called when the video IPC has
			 * received an end-of-stream from the server. The return
			 * value is a boolean indicating whether the
			 * end-of-stream must be ignored or not. If 'false',
			 * the current media is destroyed. In all cases, a flush
			 * is triggered.
			 * @param pdraw: PDrAW instance handle
			 * @param source: video IPC source handle
			 * @param eosReason end of stream reason
			 * @return true to ignore the end-of-stream, false to
			 * destroy the media.
			 */
			virtual bool
			vipcSourceEndOfStream(IPdraw *pdraw,
					      IPdraw::IVipcSource *source,
					      enum pdraw_vipc_source_eos_reason
						      eosReason) = 0;
		};

		/**
		 * Destroy a video IPC source.
		 * This function stops a running video IPC source and frees
		 * the associated resources.
		 */
		virtual ~IVipcSource(void) {}

		/**
		 * Get the ready to play status.
		 * This function returns true if the video IPC is ready to
		 * start, false otherwise. The value returned by this function
		 * is identical to the ready parameter passed to the
		 * readyToPlay() listener function when it is called.
		 * @return the ready to play status on success, false in case of
		 *         error
		 */
		virtual bool isReadyToPlay(void) = 0;

		/**
		 * Get the pause status.
		 * This function returns true if the video IPC is currently
		 * paused, false otherwise.
		 * @return the pause status on success, false in case of error
		 */
		virtual bool isPaused(void) = 0;

		/**
		 * Start receiving frames on the video IPC.
		 * This function starts the video IPC if it is ready to play.
		 * Otherwise an error is returned. Receiving frames can be
		 * halted by calling the pause() function.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int play(void) = 0;

		/**
		 * Stop receiving frames on the video IPC.
		 * This function halts the video IPC to stop receiving
		 * frames. Receiving frames can be resumed by calling the
		 * play() function.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int pause(void) = 0;

		/**
		 * Configure the video IPC.
		 * This function can be used to dynamically reconfigure a video
		 * IPC. The resolution and crop can be specified; if either
		 * parameter is null or values are 0, it is ignored.
		 * The function returns before the actual operation is done. If
		 * the function returns 0, the vipcSourceConfigured() listener
		 * function will be called once the configuration is successful
		 * (0 status) or has failed (negative errno status). If the
		 * function returns a negative errno value (immediate failure),
		 * the vipcSourceConfigured() listener function will not be
		 * called.
		 * @param resolution: new video IPC resolution to apply
		 *                    (optional, can be null)
		 * @param crop: new video IPC crop to apply
		 *              (optional, can be null)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int configure(const struct vdef_dim *resolution,
				      const struct vdef_rectf *crop) = 0;

		/**
		 * Insert a grey frame.
		 * This function can be used to insert a grey frame into the
		 * video IPC source at a given timestamp. This can be used for
		 * example to add a first sample on a VIPC that does not send
		 * frame continuously. The function can be called several times
		 * to insert several grey frames but the frame timestamp (usec)
		 * must be strictly monotonic.
		 * @param tsUs: the grey frame timestamp in microseconds
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int insertGreyFrame(uint64_t tsUs) = 0;

		/**
		 * Set the session metadata of the video IPC source.
		 * This function updates the session metadata on a running video
		 * IPC source, and propgates this structure to all elements
		 * downstream in the pipeline. The structure is copied
		 * internally and ownership stays with the caller.
		 * @param meta: new session metadata
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int
		setSessionMetadata(const struct vmeta_session *meta) = 0;

		/**
		 * Get the session metadata of the video IPC source.
		 * This function retrieves the session metadata on a running
		 * video IPC source. The provided meta structure is
		 * filled by the function.
		 * @param meta: session metadata (output)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int getSessionMetadata(struct vmeta_session *meta) = 0;
	};


	/**
	 * Create a video IPC source.
	 * This function creates a video IPC source. Once a video source is no
	 * longer used, it must be destroyed. The Video IPC address must be
	 * provided. The listener must be provided; all listener functions are
	 * called from the pomp_loop thread.
	 *
	 * @param params: video IPC source parameters
	 * @param listener: video IPC source listener functions implementation
	 * @param retObj: video IPC source object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createVipcSource(const struct pdraw_vipc_source_params *params,
			 IPdraw::IVipcSource::Listener *listener,
			 IPdraw::IVipcSource **retObj) = 0;


	/**
	 * Coded video source API
	 * @see the createCodedVideoSource() function for the coded video source
	 * object creation
	 */
	class ICodedVideoSource {
	public:
		/* Video source listener object */
		class Listener {
		public:
			/**
			 * Coded video source listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Coded video source flushed function, called to signal
			 * that flushing is complete after the coded video
			 * source flush() function has been called.
			 * @param pdraw: PDrAW instance handle
			 * @param source: coded video source handle
			 */
			virtual void onCodedVideoSourceFlushed(
				IPdraw *pdraw,
				IPdraw::ICodedVideoSource *source) = 0;
		};

		/**
		 * Destroy a coded video source.
		 * This function stops a running coded video source and frees
		 * the associated resources.
		 */
		virtual ~ICodedVideoSource(void) {}

		/**
		 * Get the coded video source frame queue.
		 * This function returns the frame queue to use in order to
		 * push frames to a running coded video source. Frames are
		 * pushed into the queue by using the
		 * mbuf_coded_video_frame_queue_push() function.
		 * @return a pointer on a mbuf_coded_video_frame_queue object on
		 *          success, nullptr in case of error
		 */
		virtual struct mbuf_coded_video_frame_queue *getQueue(void) = 0;

		/**
		 * Coded video source flush function, to be called when flushing
		 * is required. When this function is called, all frames
		 * previously pushed to the queue will be returned; once the
		 * flushing is done, the onCodedVideoSourceFlushed() listener
		 * function will be called.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int flush(void) = 0;

		/**
		 * Set the session metadata of the coded video source.
		 * This function updates the session metadata on a running coded
		 * video source, and propgates this structure to all elements
		 * downstream in the pipeline. The structure is copied
		 * internally and ownership stays with the caller.
		 * @param meta: new session metadata
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int
		setSessionMetadata(const struct vmeta_session *meta) = 0;

		/**
		 * Get the session metadata of the coded video source.
		 * This function retrieves the session metadata on a running
		 * coded video source. The provided meta structure is
		 * filled by the function.
		 * @param meta: session metadata (output)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int getSessionMetadata(struct vmeta_session *meta) = 0;
	};


	/**
	 * Raw video source API
	 * @see the createRawVideoSource() function for the raw video source
	 * object creation
	 */
	class IRawVideoSource {
	public:
		/* Video source listener object */
		class Listener {
		public:
			/**
			 * Raw video source listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Raw video source flushed function, called to signal
			 * that flushing is complete after the raw video
			 * source flush() function has been called.
			 * @param pdraw: PDrAW instance handle
			 * @param source: raw video source handle
			 */
			virtual void onRawVideoSourceFlushed(
				IPdraw *pdraw,
				IPdraw::IRawVideoSource *source) = 0;
		};

		/**
		 * Destroy a raw video source.
		 * This function stops a running raw video source and frees
		 * the associated resources.
		 */
		virtual ~IRawVideoSource(void) {}

		/**
		 * Get the raw video source frame queue.
		 * This function returns the frame queue to use in order to
		 * push frames to a running raw video source. Frames are
		 * pushed into the queue by using the
		 * mbuf_raw_video_frame_queue_push() function.
		 * @return a pointer on a mbuf_raw_video_frame_queue object on
		 *          success, nullptr in case of error
		 */
		virtual struct mbuf_raw_video_frame_queue *getQueue(void) = 0;

		/**
		 * Raw video source flush function, to be called when flushing
		 * is required. When this function is called, all frames
		 * previously pushed to the queue will be returned; once the
		 * flushing is done, the onRawVideoSourceFlushed() listener
		 * function will be called.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int flush(void) = 0;

		/**
		 * Set the session metadata of the raw video source.
		 * This function updates the session metadata on a running raw
		 * video source, and propgates this structure to all elements
		 * downstream in the pipeline. The structure is copied
		 * internally and ownership stays with the caller.
		 * @param meta: new session metadata
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int
		setSessionMetadata(const struct vmeta_session *meta) = 0;

		/**
		 * Get the session metadata of the raw video source.
		 * This function retrieves the session metadata on a running
		 * raw video source. The provided meta structure is
		 * filled by the function.
		 * @param meta: session metadata (output)
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int getSessionMetadata(struct vmeta_session *meta) = 0;
	};


	/**
	 * Create a coded video source.
	 * This function creates a video source to push frames to. Once the
	 * source is created, video frames are to be pushed into the frame queue
	 * returned by the getQueue() function. Once a video source is no longer
	 * used, it must be destroyed.
	 * The params structure must be provided and must be filled.
	 * The listener must be provided and the onCodedVideoSourceFlushed()
	 * function is required to be implemented; all listener functions are
	 * called from the pomp_loop thread.
	 *
	 * @param params: video source parameters
	 * @param listener: video source listener functions implementation
	 * @param retObj: video source object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createCodedVideoSource(const struct pdraw_video_source_params *params,
			       IPdraw::ICodedVideoSource::Listener *listener,
			       IPdraw::ICodedVideoSource **retObj) = 0;

	/**
	 * Create a raw video source.
	 * This function creates a video source to push frames to. Once the
	 * source is created, video frames are to be pushed into the frame queue
	 * returned by the getQueue() function. Once a video source is no longer
	 * used, it must be destroyed.
	 * The params structure must be provided and must be filled.
	 * The listener must be provided and the onRawVideoSourceFlushed()
	 * function is required to be implemented; all listener functions are
	 * called from the pomp_loop thread.
	 *
	 * @param params: video source parameters
	 * @param listener: video source listener functions implementation
	 * @param retObj: video source object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createRawVideoSource(const struct pdraw_video_source_params *params,
			     IPdraw::IRawVideoSource::Listener *listener,
			     IPdraw::IRawVideoSource **retObj) = 0;


	/**
	 * Coded video sink API
	 * @see the createCodedVideoSink() function for the coded video sink
	 * object creation
	 */
	class ICodedVideoSink {
	public:
		/* Video sink listener object */
		class Listener {
		public:
			/**
			 * Coded video sink listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Coded video sink flush function, called when flushing
			 * is required. When this function is called, the
			 * application must flush the sink queue by calling
			 * mbuf_coded_video_frame_queue_flush() and must return
			 * all frames outside of the queue by calling
			 * mbuf_coded_frame_unref(); once the flushing is done,
			 * the queueFlushed() function must be called.
			 * @param pdraw: PDrAW instance handle
			 * @param sink: coded video sink handle
			 */
			virtual void onCodedVideoSinkFlush(
				IPdraw *pdraw,
				IPdraw::ICodedVideoSink *sink) = 0;

			/**
			 * Session metadata update function, called when the
			 * session metadata have been updated.
			 * @param pdraw: PDrAW instance handle
			 * @param sink: coded video sink handle
			 * @param meta: new session metadata
			 */
			virtual void onCodedVideoSinkSessionMetaUpdate(
				IPdraw *pdraw,
				IPdraw::ICodedVideoSink *sink,
				const struct vmeta_session *meta) = 0;
		};

		/**
		 * Destroy a coded video sink.
		 * This function stops a running coded video sink and frees the
		 * associated resources.
		 */
		virtual ~ICodedVideoSink(void) {}

		/**
		 * Resynchronize a coded video sink.
		 * This function schedules the output of a synchronization frame
		 * (IDR) for a running coded video sink.
		 * It can be used for example in case of unrecoverable video
		 * decoder errors to restart decoding. After a coded video sink
		 * creation, the first frame that is output is always a
		 * synchronization frame; therefore it is not necessary to call
		 * this function immediately after a video sink creation.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int resync(void) = 0;

		/**
		 * Get the coded video sink frame queue.
		 * This function returns the frame queue to use in order to
		 * retrieve frames from a running coded video sink. Frames are
		 * retrieved from the queue by using the
		 * mbuf_coded_video_frame_queue_pop() function.
		 * @return a pointer on a mbuf_coded_video_frame_queue object on
		 *          success, nullptr in case of error
		 */
		virtual struct mbuf_coded_video_frame_queue *getQueue(void) = 0;

		/**
		 * Signal that a coded video sink has been flushed.
		 * This function is used to signal that flushing is complete.
		 * When the onCodedVideoSinkFlush() video sink listener function
		 * is called, the application must flush the sink queue by
		 * calling mbuf_coded_video_frame_queue_flush() and must return
		 * all frames outside of the queue by calling
		 * mbuf_coded_video_frame_unref(); once the flushing is
		 * complete, this function must be called.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int queueFlushed(void) = 0;
	};


	/**
	 * Raw video sink API
	 * @see the createRawVideoSink() function for the raw video sink
	 * object creation
	 */
	class IRawVideoSink {
	public:
		/* Video sink listener object */
		class Listener {
		public:
			/**
			 * Raw video sink listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Raw video sink flush function, called when flushing
			 * is required. When this function is called, the
			 * application must flush the sink queue by calling
			 * mbuf_raw_video_frame_queue_flush() and must return
			 * all frames outside of the queue by calling
			 * mbuf_raw_frame_unref(); once the flushing is done,
			 * the queueFlushed() function must be called.
			 * @param pdraw: PDrAW instance handle
			 * @param sink: raw video sink handle
			 */
			virtual void
			onRawVideoSinkFlush(IPdraw *pdraw,
					    IPdraw::IRawVideoSink *sink) = 0;

			/**
			 * Session metadata update function, called when the
			 * session metadata have been updated.
			 * @param pdraw: PDrAW instance handle
			 * @param sink: raw video sink handle
			 * @param meta: new session metadata
			 */
			virtual void onRawVideoSinkSessionMetaUpdate(
				IPdraw *pdraw,
				IPdraw::IRawVideoSink *sink,
				const struct vmeta_session *meta) = 0;
		};

		/**
		 * Destroy a raw video sink.
		 * This function stops a running raw video sink and frees the
		 * associated resources.
		 */
		virtual ~IRawVideoSink(void) {}

		/**
		 * Get the raw video sink frame queue.
		 * This function returns the frame queue to use in order to
		 * retrieve frames from a running raw video sink. Frames are
		 * retrieved from the queue by using the
		 * mbuf_raw_video_frame_queue_pop() function.
		 * @return a pointer on a mbuf_raw_video_frame_queue object on
		 *          success, nullptr in case of error
		 */
		virtual struct mbuf_raw_video_frame_queue *getQueue(void) = 0;

		/**
		 * Signal that a raw video sink has been flushed.
		 * This function is used to signal that flushing is complete.
		 * When the onRawVideoSinkFlush() video sink listener function
		 * is called, the application must flush the sink queue by
		 * calling mbuf_raw_video_frame_queue_flush() and must return
		 * all frames outside of the queue by calling
		 * mbuf_raw_video_frame_unref(); once the flushing is
		 * complete, this function must be called.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int queueFlushed(void) = 0;
	};


	/**
	 * Create a coded video sink.
	 * This function creates a video sink on a media of the given mediaId.
	 * The media idenfifiers are known when the onMediaAdded() or
	 * onMediaRemoved() general listener functions are called. Once the
	 * sink is created, video frames are retrieved by getting them from
	 * the frame queue returned by the getQueue() function. Once a video
	 * sink is no longer used, it must be destroyed.
	 * The params structure must be provided but all parameters are optional
	 * and can be left null. The listener must be provided and the
	 * onCodedVideoSinkFlush() function is required to be implemented; all
	 * listener functions are called from the pomp_loop thread. When the
	 * onCodedVideoSinkFlush() function is called, the application must
	 * flush the sink queue by calling mbuf_coded_video_frame_queue_flush()
	 * and must return all frames outside of the queue by calling
	 * mbuf_coded_video_frame_unref(); once the flushing is complete, the
	 * queueFlushed() function must be called.
	 *
	 * @note mediaId must refer to a coded video media.
	 *
	 * @param mediaId: identifier of the media on which to create the sink
	 * @param params: video sink parameters
	 * @param listener: video sink listener functions implementation
	 * @param retObj: video sink object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createCodedVideoSink(unsigned int mediaId,
			     const struct pdraw_video_sink_params *params,
			     IPdraw::ICodedVideoSink::Listener *listener,
			     IPdraw::ICodedVideoSink **retObj) = 0;

	/**
	 * Create a raw video sink.
	 * This function creates a video sink on a media of the given mediaId.
	 * The media idenfifiers are known when the onMediaAdded() or
	 * onMediaRemoved() general listener functions are called. Once the
	 * sink is created, video frames are retrieved by getting them from
	 * the frame queue returned by the getQueue() function. Once a video
	 * sink is no longer used, it must be destroyed.
	 * The params structure must be provided but all parameters are optional
	 * and can be left null. The listener must be provided and the
	 * onRawVideoSinkFlush() function is required to be implemented; all
	 * listener functions are called from the pomp_loop thread. When the
	 * onRawVideoSinkFlush() function is called, the application must
	 * flush the sink queue by calling mbuf_raw_video_frame_queue_flush()
	 * and must return all frames outside of the queue by calling
	 * mbuf_raw_video_frame_unref(); once the flushing is complete, the
	 * queueFlushed() function must be called.
	 *
	 * @note mediaId must refer to a raw video media.
	 *
	 * @param mediaId: identifier of the media on which to create the sink
	 * @param params: video sink parameters
	 * @param listener: video sink listener functions implementation
	 * @param retObj: video sink object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createRawVideoSink(unsigned int mediaId,
			   const struct pdraw_video_sink_params *params,
			   IPdraw::IRawVideoSink::Listener *listener,
			   IPdraw::IRawVideoSink **retObj) = 0;


	/**
	 * ALSA source API
	 * @see the createAlsaSource() function for the ALSA source
	 * object creation
	 */
	class IAlsaSource {
	public:
		/* Alsa source listener object */
		class Listener {
		public:
			/**
			 * Alsa source listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Ready to play function, called when the ALSA source
			 * is ready to start receiving frames (the ready
			 * parameter is true), or when the ALSA source cannot
			 * receive frames any more (end of stream; the ready
			 * parameter is false). When the ready parameter is
			 * false, the eosReason parameter indicates the reason
			 * why the end of stream was received on the ALSA
			 * source.
			 * @param pdraw: PDrAW instance handle
			 * @param source: ALSA source handle
			 * @param ready: true if the session is ready to play,
			 *               false otherwise
			 * @param eosReason: end of stream reason if the ready
			 *                   parameter is false
			 */
			virtual void
			alsaSourceReadyToPlay(IPdraw *pdraw,
					      IPdraw::IAlsaSource *source,
					      bool ready,
					      enum pdraw_alsa_source_eos_reason
						      eosReason) = 0;

			/**
			 * Frame ready function, called when a video frame has
			 * been received and before it is propagated downstream
			 * in the pipeline. This can be used to associate
			 * metadata with the frame.
			 * @param pdraw: PDrAW instance handle
			 * @param source: ALSA source handle
			 * @param frame: frame information
			 */
			virtual void alsaSourceFrameReady(
				IPdraw *pdraw,
				IPdraw::IAlsaSource *source,
				struct mbuf_audio_frame *frame) = 0;
		};

		/**
		 * Destroy an ALSA source.
		 * This function stops a running ALSA source and frees
		 * the associated resources.
		 */
		virtual ~IAlsaSource(void) {}

		/**
		 * Get the ready to play status.
		 * This function returns true if the ALSA source is ready to
		 * start, false otherwise. The value returned by this function
		 * is identical to the ready parameter passed to the
		 * readyToPlay() listener function when it is called.
		 * @return the ready to play status on success, false in case of
		 *         error
		 */
		virtual bool isReadyToPlay(void) = 0;

		/**
		 * Get the pause status.
		 * This function returns true if the ALSA source is currently
		 * paused, false otherwise.
		 * @return the pause status on success, false in case of error
		 */
		virtual bool isPaused(void) = 0;

		/**
		 * Start receiving frames on the ALSA source.
		 * This function starts the ALSA source if it is ready to play.
		 * Otherwise an error is returned. Receiving frames can be
		 * halted by calling the pause() function.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int play(void) = 0;

		/**
		 * Stop receiving frames on the ALSA source.
		 * This function halts the ALSA source to stop receiving
		 * frames. Receiving frames can be resumed by calling the
		 * play() function.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int pause(void) = 0;
	};


	/**
	 * Create an ALSA source.
	 * This function creates an ALSA source. Once an ALSA source
	 * is no longer used, it must be destroyed. The hardware address must be
	 * provided. The listener must be provided; all listener functions are
	 * called from the pomp_loop thread.
	 *
	 * @param params: ALSA source parameters
	 * @param listener: ALSA source listener functions implementation
	 * @param retObj: ALSA source object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createAlsaSource(const struct pdraw_alsa_source_params *params,
			 IPdraw::IAlsaSource::Listener *listener,
			 IPdraw::IAlsaSource **retObj) = 0;


	/**
	 * Audio source API
	 * @see the createAudioSource() function for the audio source
	 * object creation
	 */
	class IAudioSource {
	public:
		/* Audio source listener object */
		class Listener {
		public:
			/**
			 * Audio source listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Audio source flushed function, called to signal
			 * that flushing is complete after the audio source
			 * flush() function has been called.
			 * @param pdraw: PDrAW instance handle
			 * @param source: audio source handle
			 */
			virtual void
			onAudioSourceFlushed(IPdraw *pdraw,
					     IPdraw::IAudioSource *source) = 0;
		};

		/**
		 * Destroy an audio source.
		 * This function stops a running audio source and frees
		 * the associated resources.
		 */
		virtual ~IAudioSource(void) {}

		/**
		 * Get the audio source frame queue.
		 * This function returns the frame queue to use in order to
		 * push frames to a running audio source. Frames are pushed
		 * into the queue by using the mbuf_audio_frame_queue_push()
		 * function.
		 * @return a pointer on a mbuf_audio_frame_queue object on
		 *         success, nullptr in case of error
		 */
		virtual struct mbuf_audio_frame_queue *getQueue(void) = 0;

		/**
		 * Audio source flush function, to be called when flushing
		 * is required. When this function is called, all frames
		 * previously pushed to the queue will be returned; once the
		 * flushing is done, the onAudioSourceFlushed() listener
		 * function will be called.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int flush(void) = 0;
	};


	/**
	 * Create an audio source.
	 * This function creates an audio source to push frames to. Once the
	 * source is created, audio frames are to be pushed into the frame queue
	 * returned by the getQueue() function. Once an audio source is no
	 * longer used, it must be destroyed. The params structure must be
	 * provided and must be filled. The listener must be provided and the
	 * onAudioSourceFlushed() function is required to be implemented; all
	 * listener functions are called from the pomp_loop thread.
	 *
	 * @param params: audio source parameters
	 * @param listener: audio source listener functions implementation
	 * @param retObj: audio source object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createAudioSource(const struct pdraw_audio_source_params *params,
			  IPdraw::IAudioSource::Listener *listener,
			  IPdraw::IAudioSource **retObj) = 0;


	/**
	 * Audio sink API
	 * @see the createAudioSink() function for the audio sink
	 * object creation
	 */
	class IAudioSink {
	public:
		/* Video sink listener object */
		class Listener {
		public:
			/**
			 * Audio sink listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Audio sink flush function, called when flushing
			 * is required. When this function is called, the
			 * application must flush the sink queue by calling
			 * mbuf_audio_frame_queue_flush() and must return
			 * all frames outside of the queue by calling
			 * mbuf_raw_frame_unref(); once the flushing is done,
			 * the queueFlushed() function must be called.
			 * @param pdraw: PDrAW instance handle
			 * @param sink: audio sink handle
			 */
			virtual void
			onAudioSinkFlush(IPdraw *pdraw,
					 IPdraw::IAudioSink *sink) = 0;
		};

		/**
		 * Destroy an audio sink.
		 * This function stops a running audio sink and frees the
		 * associated resources.
		 */
		virtual ~IAudioSink(void) {}

		/**
		 * Get the audio sink frame queue.
		 * This function returns the frame queue to use in order to
		 * retrieve frames from a running audio sink. Frames are
		 * retrieved from the queue by using the
		 * mbuf_audio_frame_queue_pop() function.
		 * @return a pointer on a mbuf_audio_frame_queue object on
		 *         success, nullptr in case of error
		 */
		virtual struct mbuf_audio_frame_queue *getQueue(void) = 0;

		/**
		 * Signal that an audio sink has been flushed.
		 * This function is used to signal that flushing is complete.
		 * When the onAudioSinkFlush() audio sink listener function
		 * is called, the application must flush the sink queue by
		 * calling mbuf_audio_frame_queue_flush() and must return
		 * all frames outside of the queue by calling
		 * mbuf_audio_frame_unref(); once the flushing is complete,
		 * this function must be called.
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int queueFlushed(void) = 0;
	};


	/**
	 * Create an audio sink.
	 * This function creates an audio sink on a media of the given mediaId.
	 * The media idenfifiers are known when the onMediaAdded() or
	 * onMediaRemoved() general listener functions are called. Once the
	 * sink is created, audio frames are retrieved by getting them from
	 * the frame queue returned by the getQueue() function. Once an audio
	 * sink is no longer used, it must be destroyed.
	 * The listener must be provided and the onAudioSinkFlush() function is
	 * required to be implemented; all listener functions are called from
	 * the pomp_loop thread. When the onAudioSinkFlush() function is called,
	 * the application must flush the sink queue by calling
	 * mbuf_audio_frame_queue_flush() and must return all frames outside of
	 * the queue by calling mbuf_audio_frame_unref(); once the flushing is
	 * complete, the queueFlushed() function must be called.
	 *
	 * @note mediaId must refer to an audio media.
	 *
	 * @param mediaId: identifier of the media on which to create the sink
	 * @param listener: audio sink listener functions implementation
	 * @param retObj: audio sink object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int createAudioSink(unsigned int mediaId,
				    IPdraw::IAudioSink::Listener *listener,
				    IPdraw::IAudioSink **retObj) = 0;


	/**
	 * Video encoder API
	 */
	class IVideoEncoder {
	public:
		/* Video encoder listener object */
		class Listener {
		public:
			/**
			 * Video encoder listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Frame output function, called when a video frame is
			 * output from the encoder. This can be used to extract
			 * frame info and ancillary data from the frame. This
			 * function is usually called from the pomp_loop thread,
			 * but it may depend on the video encoder
			 * implementation.
			 * @warning: this function is called with a Listener
			 * mutex locked, therefore no API function must be
			 * called from this callback.
			 * @param pdraw: PDrAW instance handle
			 * @param encoder: video encoder handle
			 * @param frame: frame information
			 */
			virtual void videoEncoderFrameOutput(
				IPdraw *pdraw,
				IPdraw::IVideoEncoder *encoder,
				struct mbuf_coded_video_frame *frame) = 0;

			/**
			 * Frame pre-release function, called before a video
			 * frame is released. This can be used to extract frame
			 * info and ancillary data from the frame. This function
			 * is called from the thread that unrefs the frame; it
			 * can be any thread.
			 * @warning: this function is called with a Listener
			 * mutex locked, therefore no API function must be
			 * called from this callback.
			 * @param pdraw: PDrAW instance handle
			 * @param encoder: video encoder handle
			 * @param frame: frame information
			 */
			virtual void videoEncoderFramePreRelease(
				IPdraw *pdraw,
				IPdraw::IVideoEncoder *encoder,
				struct mbuf_coded_video_frame *frame) = 0;
		};

		/**
		 * Destroy a video encoder.
		 * This function stops a running video encoder and frees the
		 * associated resources.
		 */
		virtual ~IVideoEncoder(void) {}

		/**
		 * Configure the video encoder.
		 * This function can be used to dynamically reconfigure a video
		 * encoder.
		 * @param config: new video encoder dynamic configuration
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int configure(const struct venc_dyn_config *config) = 0;

		/**
		 * Get the video encoder dynamic configuration.
		 * This function fills the config structure with the dynamic
		 * configuration. The structure must have been previously
		 * allocated.
		 * @param config: video encoder configuration structure to fill
		 * @return 0 on success, negative errno value in case of error
		 */
		virtual int getConfig(struct venc_dyn_config *config) = 0;
	};


	/**
	 * Create a video encoder.
	 * This function creates a video encoder on a media of the given
	 * mediaId. The media idenfifiers are known when the onMediaAdded() or
	 * onMediaRemoved() general listener functions are called.
	 * Once a video encoder is no longer used, it must be destroyed.
	 * The params structure must be provided but some parameters are
	 * optional and can be left null. The input sub-structure in the params
	 * structure is ignored. The listener must be provided but all listener
	 * functions are optional; the videoEncoderFrameOutput() function is
	 * usually called from the pomp_loop thread, but it may depend on the
	 * video encoder implementation. The videoEncoderFramePreRelease()
	 * function is called from the thread that unrefs the frame; it can be
	 * called from any thread.
	 *
	 * @note mediaId must refer to a raw video media.
	 *
	 * @param mediaId: identifier of the media on which to create the
	 *                 video encoder
	 * @param params: video encoder parameters
	 * @param listener: video encoder listener functions implementation
	 * @param retObj: video encoder object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createVideoEncoder(unsigned int mediaId,
			   const struct venc_config *params,
			   IPdraw::IVideoEncoder::Listener *listener,
			   IPdraw::IVideoEncoder **retObj) = 0;


	/**
	 * Video Scaler API
	 */

	class IVideoScaler {
	public:
		/* Video scaler listener object */
		class Listener {
		public:
			/**
			 * Video scaler listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Frame output function, called when a video frame is
			 * output from the scaler. This can be used to extract
			 * frame info and ancillary data from the frame. This
			 * function is usually called from the pomp_loop thread,
			 * but it may depend on the video scaler
			 * implementation.
			 * @param pdraw: PDrAW instance handle
			 * @param scaler: video scaler handle
			 * @param frame: frame information
			 */
			virtual void videoScalerFrameOutput(
				IPdraw *pdraw,
				IPdraw::IVideoScaler *scaler,
				struct mbuf_raw_video_frame *frame) = 0;
		};

		/**
		 * Destroy a video scaler.
		 * This function stops a running video scaler and frees the
		 * associated resources.
		 */
		virtual ~IVideoScaler(void) {}
	};

	/**
	 * Create a video scaler.
	 * This function creates a video scaler on a media of the given
	 * mediaId. The media idenfifiers are known when the onMediaAdded() or
	 * onMediaRemoved() general listener functions are called.
	 * Once a video scaler is no longer used, it must be destroyed.
	 * The params structure must be provided but some parameters are
	 * optional and can be left null. The input sub-structure in the params
	 * structure is ignored. The listener must be provided but all listener
	 * functions are optional; the videoScalerFrameOutput() function is
	 * usually called from the pomp_loop thread, but it may depend on the
	 * video scaler implementation.
	 *
	 * @note mediaId must refer to a raw video media.
	 *
	 * @param mediaId: identifier of the media on which to create the sink
	 * @param params: video scaler parameters
	 * @param listener: video scaler listener functions implementation
	 * @param retObj: video scaler object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int createVideoScaler(unsigned int mediaId,
				      const struct vscale_config *params,
				      IPdraw::IVideoScaler::Listener *listener,
				      IPdraw::IVideoScaler **retObj) = 0;


	/**
	 * Audio encoder API
	 */
	class IAudioEncoder {
	public:
		/* Audio encoder listener object */
		class Listener {
		public:
			/**
			 * Audio encoder listener object destructor.
			 */
			virtual ~Listener(void) {}

			/**
			 * Frame output function, called when an audio frame is
			 * output from the encoder. This can be used to extract
			 * frame info and ancillary data from the frame. This
			 * function is usually called from the pomp_loop thread,
			 * but it may depend on the audio encoder
			 * implementation.
			 * @warning: this function is called with a Listener
			 * mutex locked, therefore no API function must be
			 * called from this callback.
			 * @param pdraw: PDrAW instance handle
			 * @param encoder: audio encoder handle
			 * @param frame: frame information
			 */
			virtual void audioEncoderFrameOutput(
				IPdraw *pdraw,
				IPdraw::IAudioEncoder *encoder,
				struct mbuf_audio_frame *frame) = 0;

			/**
			 * Frame pre-release function, called before an audio
			 * frame is released. This can be used to extract frame
			 * info and ancillary data from the frame. This function
			 * is called from the thread that unrefs the frame; it
			 * can be any thread.
			 * @warning: this function is called with a Listener
			 * mutex locked, therefore no API function must be
			 * called from this callback.
			 * @param pdraw: PDrAW instance handle
			 * @param encoder: audio encoder handle
			 * @param frame: frame information
			 */
			virtual void audioEncoderFramePreRelease(
				IPdraw *pdraw,
				IPdraw::IAudioEncoder *encoder,
				struct mbuf_audio_frame *frame) = 0;
		};

		/**
		 * Destroy an audio encoder.
		 * This function stops a running audio encoder and frees the
		 * associated resources.
		 */
		virtual ~IAudioEncoder(void) {}
	};


	/**
	 * Create an audio encoder.
	 * This function creates an audio encoder on a media of the given
	 * mediaId. The media idenfifiers are known when the onMediaAdded() or
	 * onMediaRemoved() general listener functions are called.
	 * Once an audio encoder is no longer used, it must be destroyed.
	 * The params structure must be provided but some parameters are
	 * optional and can be left null. The input sub-structure in the params
	 * structure is ignored. The listener must be provided but all listener
	 * functions are optional; the audioEncoderFrameOutput() function is
	 * usually called from the pomp_loop thread, but it may depend on the
	 * audio encoder implementation. The audioEncoderFramePreRelease()
	 * function is called from the thread that unrefs the frame; it can be
	 * called from any thread.
	 *
	 * @note mediaId must refer to an audio media.
	 *
	 * @param mediaId: identifier of the media on which to create the sink
	 * @param params: audio encoder parameters
	 * @param listener: audio encoder listener functions implementation
	 * @param retObj: audio encoder object pointer (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	createAudioEncoder(unsigned int mediaId,
			   const struct aenc_config *params,
			   IPdraw::IAudioEncoder::Listener *listener,
			   IPdraw::IAudioEncoder **retObj) = 0;


	/**
	 * Settings API
	 */

	/**
	 * Get the PDrAW instance friendly name.
	 * This function fills the friendlyName string with the friendly name.
	 * The string must have been previously allocated. The friendly name is
	 * generally either the device's friendly name (e.g. "Bob's phone") or
	 * the application's name (e.g. "MyDroneControllerApp"). It is used for
	 * example in metadata exchanged with a streaming server.
	 * @param friendlyName: pointer to the string to write to (output)
	 */
	virtual void getFriendlyNameSetting(std::string *friendlyName) = 0;

	/**
	 * Set the PDrAW instance friendly name.
	 * The friendlyName string is copied internally. The friendly name is
	 * generally either the device's friendly name (e.g. "Bob's phone") or
	 * the application's name (e.g. "MyDroneControllerApp"). It is used for
	 * example in metadata exchanged with a streaming server. Setting the
	 * friendly name value is optional.
	 * @param friendlyName: friendly name string
	 */
	virtual void
	setFriendlyNameSetting(const std::string &friendlyName) = 0;

	/**
	 * Get the PDrAW instance serial number.
	 * This function fills the serialNumber string with the serial number.
	 * The string must have been previously allocated. The serial number is
	 * generally the unique serial number of the device on which PDrAW is
	 * running. It is used for example in metadata exchanged with a
	 * streaming server.
	 * @param serialNumber: pointer to the string to write to (output)
	 */
	virtual void getSerialNumberSetting(std::string *serialNumber) = 0;

	/**
	 * Set the PDrAW instance serial number.
	 * The serialNumber string is copied internally. The serial number is
	 * generally the unique serial number of the device on which PDrAW is
	 * running. It is used for example in metadata exchanged with a
	 * streaming server. Setting the serial number value is recommended as
	 * it is used as a unique identifier in the streaming protocols.
	 * @param serialNumber: serial number string
	 */
	virtual void
	setSerialNumberSetting(const std::string &serialNumber) = 0;

	/**
	 * Get the PDrAW instance software version.
	 * This function fills the softwareVersion string with the software
	 * version. The string must have been previously allocated. The software
	 * version is generally the version number of the application running
	 * PDrAW (e.g. "MyApp v1.2.3"). It is used for example in metadata
	 * exchanged with a streaming server.
	 * @param softwareVersion: pointer to the string to write to (output)
	 */
	virtual void
	getSoftwareVersionSetting(std::string *softwareVersion) = 0;

	/**
	 * Set the PDrAW instance software version.
	 * The softwareVersion string is copied internally. The software version
	 * is generally the version number of the application running PDrAW
	 * (e.g. "MyApp v1.2.3"). It is used for example in metadata exchanged
	 * with a streaming server. Setting the software version value is
	 * optional.
	 * @param softwareVersion: software version string
	 */
	virtual void
	setSoftwareVersionSetting(const std::string &softwareVersion) = 0;


	/**
	 * Debug API
	 */

	/**
	 * Dump the current pipeline as a directed graph using the DOT file
	 * format.
	 * @param fileName: DOT file to write to
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int dumpPipeline(const std::string &fileName) = 0;
};


/**
 * Create a PDrAW instance.
 * A running pomp_loop must be provided; all functions (with the exception of
 * rendering functions) must be called from the pomp_loop thread. All listener
 * functions are called from the pomp_loop thread; a valid listener must be
 * provided. The instance handle is returned through the retObj parameter.
 * When no longer needed, the instance must be deleted to free the resources.
 * The stop() function must be called and one must wait for the stopResponse()
 * listener function to be called prior to destroying the instance.
 * @param loop: pomp_loop to use
 * @param listener: listener functions implementation
 * @param retObj: PDrAW instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int createPdraw(struct pomp_loop *loop,
			  IPdraw::Listener *listener,
			  IPdraw **retObj);


/**
 * Helpers
 */

/**
 * ToString function for enum pdraw_demuxer_autodecoding_mode.
 * @param val: pipeline mode value to convert
 * @return a string description of the pipeline mode
 */
PDRAW_API const char *
pdrawDemuxerAutodecodingModeStr(enum pdraw_demuxer_autodecoding_mode val);


/**
 * FromString function for enum pdraw_demuxer_autodecoding_mode.
 * @param val: string to convert
 * @return pipeline mode converted from the string description
 */
PDRAW_API enum pdraw_demuxer_autodecoding_mode
pdrawDemuxerAutodecodingModeFromStr(const char *val);


/**
 * ToString function for enum pdraw_playback_type.
 * @param val: playback type value to convert
 * @return a string description of the playback type
 */
PDRAW_API const char *pdrawPlaybackTypeStr(enum pdraw_playback_type val);


/**
 * FromString function for enum pdraw_playback_type.
 * @param val: string to convert
 * @return playback type converted from the string description
 */
PDRAW_API enum pdraw_playback_type pdrawPlaybackTypeFromStr(const char *val);


/**
 * ToString function for enum pdraw_media_type.
 * @param val: media type value to convert
 * @return a string description of the media type
 */
PDRAW_API const char *pdrawMediaTypeStr(enum pdraw_media_type val);


/**
 * FromString function for enum pdraw_media_type.
 * @param val: string to convert
 * @return media type converted from the string description
 */
PDRAW_API enum pdraw_media_type pdrawMediaTypeFromStr(const char *val);


/**
 * ToString function for enum pdraw_video_type.
 * @param val: video type value to convert
 * @return a string description of the video type
 */
PDRAW_API const char *pdrawVideoTypeStr(enum pdraw_video_type val);


/**
 * FromString function for enum pdraw_video_type.
 * @param val: string to convert
 * @return video type converted from the string description
 */
PDRAW_API enum pdraw_video_type pdrawVideoTypeFromStr(const char *val);


/**
 * ToString function for enum pdraw_histogram_channel.
 * @param val: histogram channel value to convert
 * @return a string description of the histogram channel
 */
PDRAW_API const char *
pdrawHistogramChannelStr(enum pdraw_histogram_channel val);


/**
 * FromString function for enum pdraw_histogram_channel.
 * @param val: string to convert
 * @return histogram channel converted from the string description
 */
PDRAW_API enum pdraw_histogram_channel
pdrawHistogramChannelFromStr(const char *val);


/**
 * ToString function for enum pdraw_video_renderer_scheduling_mode.
 * @param val: video renderer scheduling mode value to convert
 * @return a string description of the video renderer scheduling mode
 */
PDRAW_API const char *pdrawVideoRendererSchedulingModeStr(
	enum pdraw_video_renderer_scheduling_mode val);


/**
 * FromString function for enum pdraw_video_renderer_scheduling_mode.
 * @param val: string to convert
 * @return video renderer scheduling mode converted from the string description
 */
PDRAW_API enum pdraw_video_renderer_scheduling_mode
pdrawVideoRendererSchedulingModeFromStr(const char *val);


/**
 * ToString function for enum pdraw_video_renderer_fill_mode.
 * @param val: video renderer fill mode value to convert
 * @return a string description of the video renderer fill mode
 */
PDRAW_API const char *
pdrawVideoRendererFillModeStr(enum pdraw_video_renderer_fill_mode val);


/**
 * FromString function for enum pdraw_video_renderer_fill_mode.
 * @param val: string to convert
 * @return video renderer fill mode converted from the string description
 */
PDRAW_API enum pdraw_video_renderer_fill_mode
pdrawVideoRendererFillModeFromStr(const char *val);


/**
 * ToString function for enum pdraw_video_renderer_transition_flag.
 * @param val: video renderer transition flag value to convert
 * @return a string description of the video renderer transition flag
 */
PDRAW_API const char *pdrawVideoRendererTransitionFlagStr(
	enum pdraw_video_renderer_transition_flag val);


/**
 * FromString function for enum pdraw_video_renderer_transition_flag.
 * @param val: string to convert
 * @return video renderer transition flag converted from the string description
 */
PDRAW_API enum pdraw_video_renderer_transition_flag
pdrawVideoRendererTransitionFlagFromStr(const char *val);


/**
 * ToString function for enum pdraw_vipc_source_eos_reason.
 * @param val: video IPC source end of stream reason value to convert
 * @return a string description of the video IPC source end of stream reason
 */
PDRAW_API const char *
pdrawVipcSourceEosReasonStr(enum pdraw_vipc_source_eos_reason val);


/**
 * FromString function for enum pdraw_vipc_source_eos_reason.
 * @param val: string to convert
 * @return video IPC source end of stream reason converted from the string
 * description
 */
PDRAW_API enum pdraw_vipc_source_eos_reason
pdrawVipcSourceEosReasonFromStr(const char *val);


/**
 * Convert a video frame metadata structure to a JSON object.
 * The JSON object must have been previously created. The function appends
 * new elements in this object.
 * If `metadata` is not null, its own json dump will be added to the frame dump.
 * @param frame: pointer to a video frame structure
 * @param metadata: optional pointer to the frame metadata
 * @param jobj: pointer to a JSON object to fill (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdrawVideoFrameToJson(const struct pdraw_video_frame *frame,
				    struct vmeta_frame *metadata,
				    struct json_object *jobj);


/**
 * Convert a video frame metadata structure to a JSON string.
 * This function fills the str array with the null-terminated JSON string.
 * The string must have been previously allocated. The function writes
 * up to len characters.
 * If `metadata` is not null, its own json dump will be added to the frame dump.
 * @param frame: pointer to a video frame structure
 * @param metadata: optional pointer to the frame metadata
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdrawVideoFrameToJsonStr(const struct pdraw_video_frame *frame,
				       struct vmeta_frame *metadata,
				       char *str,
				       unsigned int len);


/**
 * Duplicate a media_info structure.
 * @param src: pointer to the media_info structure to duplicate
 * @return a pointer to the newly allocated structure or nullptr on error.
 */
PDRAW_API struct pdraw_media_info *
pdrawMediaInfoDup(const struct pdraw_media_info *src);


/**
 * Free a media_info structure.
 * @param media_info: pointer to the media_info structure to free
 */
PDRAW_API void pdrawMediaInfoFree(struct pdraw_media_info *media_info);


/**
 * Get the capabilities of an ALSA source.
 * This function retrieves the capabilities of the audio capture device.
 * The provided caps structure is filled by the function.
 * @param address: address of the audio capture device
 * @param caps: ALSA source capabilities (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdrawAlsaSourceGetCapabilities(const std::string &address,
			       struct pdraw_alsa_source_caps *caps);


} /* namespace Pdraw */

#endif /* !_PDRAW_HPP_ */
