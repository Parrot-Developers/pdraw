/**
 * Parrot Drones Awesome Video Viewer Library
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
		 * Open response function, called when an open operation is
		 * complete or has failed. The status parameter is the open
		 * operation status: 0 on success, or a negative errno value
		 * in case of error.
		 * @param pdraw: PDrAW instance handle
		 * @param status: 0 on success, negative errno value in case
		 *                of error
		 */
		virtual void openResponse(IPdraw *pdraw, int status) = 0;

		/**
		 * Close response function, called when a close operation is
		 * complete or has failed. The status parameter is the close
		 * operation status: 0 on success, or a negative errno value
		 * in case of error.
		 * @param pdraw: PDrAW instance handle
		 * @param status: 0 on success, negative errno value in case
		 *                of error
		 */
		virtual void closeResponse(IPdraw *pdraw, int status) = 0;

		/**
		 * Unrecoverable error function, called when a previously
		 * opened session is no longer running. This happens only on
		 * streaming sessions, for example if the server has ended the
		 * stream. When this function is called, the PDrAW session is
		 * no longer running; the close() function must be called and
		 * one must wait for the closeResponse() listener function to
		 * be called prior to destroying the instance.
		 * @param pdraw: PDrAW instance handle
		 */
		virtual void onUnrecoverableError(IPdraw *pdraw) = 0;

		/**
		 * Demuxer media selection function, called with a list of
		 * video medias found from which the application must choose
		 *  one to process in the pipeline. Generally, only 1
		 * media will be available but there are cases when multiple
		 * medias can be available (e.g. local or streamed replay of an
		 * Anafi Thermal video). The return value of the function must
		 * be the identifier of the chosen media (from the
		 * pdraw_demuxer_media structure), or 0 to choose the default
		 * media. If the return value is -ENOSYS, the callback is
		 * considered not implemented and the default media is chosen.
		 * If the return value is another negative errno or an invalid
		 * media identifier the openResponse() function will be called
		 * if an open operation is in progress, or the
		 * onUnrecoverableError() function otherwise.
		 * @param pdraw: PDrAW instance handle
		 * @param medias: array of demuxer media
		 * @param count: demuxer media array element count
		 * @return the identifier of the chosen media, 0 or -ENOSYS to
		 *         choose the default media, or another negative errno
		 *         value in case of error
		 */
		virtual int
		selectDemuxerMedia(IPdraw *pdraw,
				   const struct pdraw_demuxer_media *medias,
				   size_t count) = 0;

		/**
		 * Media added function, called when a media has been added
		 * internally in the PDrAW pipeline. Medias are for example
		 * YUV or H.264 video medias. The info structure gives the
		 * media identifier that can be used to create a video sink
		 * on this media.
		 * @param pdraw: PDrAW instance handle
		 * @param info: pointer on the media information
		 */
		virtual void
		onMediaAdded(IPdraw *pdraw,
			     const struct pdraw_media_info *info) = 0;

		/**
		 * Media removed function, called when a media has been removed
		 * internally from the PDrAW pipeline. Medias are for example
		 * YUV or H.264 video medias. When a media is removed, any
		 * video sink created on this media must then be stopped.
		 * @param pdraw: PDrAW instance handle
		 * @param info: pointer on the media information
		 */
		virtual void
		onMediaRemoved(IPdraw *pdraw,
			       const struct pdraw_media_info *info) = 0;

		/**
		 * Ready to play function, called when the playback is ready
		 * to start. This function is called to indicate that the
		 * PDrAW session is ready to process play operations. Generally
		 * the session is ready to play as soon as the openResponse()
		 * function has been called with a success status. One case
		 * when the open operation was successful and the playback is
		 * not ready is when connected to a SkyController's RTSP server
		 * but the SkyController itself is not yet connected to a drone.
		 * Similarly, when connected to a drone's stream through a
		 * SkyController's RTSP server, if the drone is disconnected
		 * from the SkyController, this function will be called with a
		 * false value in the ready parameter.
		 * @param pdraw: PDrAW instance handle
		 * @param ready: true if the session is ready to play,
		 *               false otherwise
		 */
		virtual void readyToPlay(IPdraw *pdraw, bool ready) = 0;

		/**
		 * End of range function, called when the playback is suspended
		 * after having reached the end of the playback duration. This
		 * function is only called for replays (either local or
		 * streamed), not for live streams. The timestamp parameter is
		 * the current play time in microseconds at the moment the
		 * playback is suspended.
		 * @param pdraw: PDrAW instance handle
		 * @param timestamp: current playback time in microseconds
		 */
		virtual void onEndOfRange(IPdraw *pdraw,
					  uint64_t timestamp) = 0;

		/**
		 * Play response function, called when a play operation is
		 * complete (the playback has started) or has failed. The
		 * status parameter is the play operation status: 0 on success,
		 * or a negative errno value in case of error. The timestamp
		 * parameter is the current play time in microseconds at the
		 * moment the playback is started. The speed parameter is the
		 * current playback speed; a negative value means playing
		 * backward.
		 * @param pdraw: PDrAW instance handle
		 * @param status: 0 on success, negative errno value
		 *                in case of error
		 * @param timestamp: current playback time in microseconds
		 * @param speed: current playback speed, negative means backward
		 */
		virtual void playResponse(IPdraw *pdraw,
					  int status,
					  uint64_t timestamp,
					  float speed) = 0;

		/**
		 * Pause response function, called when a pause operation is
		 * complete (the playback is suspended) or has failed. The
		 * status parameter is the pause operation status: 0 on success,
		 * or a negative errno value in case of error. The timestamp
		 * parameter is the current play time in microseconds at the
		 * moment the playback is paused.
		 * @param pdraw: PDrAW instance handle
		 * @param status: 0 on success, negative errno value
		 *                in case of error
		 * @param timestamp: current playback time in microseconds
		 */
		virtual void pauseResponse(IPdraw *pdraw,
					   int status,
					   uint64_t timestamp) = 0;

		/**
		 * Seek response function, called when a seek operation is
		 * complete or has failed. The status parameter is the seek
		 * operation status: 0 on success, or a negative errno value in
		 * case of error. The timestamp parameter is the current play
		 * time in microseconds after seeking. The speed parameter is
		 * the current playback speed; a negative value means playing
		 * backward.
		 * @param pdraw: PDrAW instance handle
		 * @param status: 0 on success, negative errno value
		 *                in case of error
		 * @param timestamp: current playback time in microseconds
		 * @param speed: current playback speed, negative means backward
		 */
		virtual void seekResponse(IPdraw *pdraw,
					  int status,
					  uint64_t timestamp,
					  float speed) = 0;

		/**
		 * Socket creation function, called immediately after a socket
		 * creation with its file descriptor as parameter.
		 * @param pdraw: PDrAW instance handle
		 * @param fd: socket file descriptor
		 */
		virtual void onSocketCreated(IPdraw *pdraw, int fd) = 0;
	};


	/* Video renderer listener object */
	class VideoRendererListener {
	public:
		/**
		 * Video renderer listener object destructor.
		 */
		virtual ~VideoRendererListener(void) {}

		/**
		 * Render ready function, called both when a new frame is ready
		 * for rendering and periodically. This function is called from
		 * the pomp_loop thread.
		 * @param pdraw: PDrAW instance handle
		 * @param renderer: renderer handle
		 */
		virtual void
		onVideoRenderReady(IPdraw *pdraw,
				   struct pdraw_video_renderer *renderer) = 0;

		/**
		 * External texture loading function. This function is called
		 * before the rendering of the video frame in order to override
		 * the frame loading as a texture. This can be used to transform
		 * the video frames outside of PDrAW before resuming the
		 * rendering. This function is called from the rendering thread.
		 * If no implementation of this function is required by the
		 * application, -ENOSYS must be returned (before checking
		 * input values, so that implementation can be tested
		 * with all arguments null).
		 * @param pdraw: PDrAW instance handle
		 * @param renderer: renderer handle
		 * @param textureWidth: texture width in pixels
		 * @param textureHeight: texture height in pixels
		 * @param sessionInfo: session information
		 * @param sessionMeta: session metadata
		 * @param frame: frame information
		 * @param frameUserdata: frame user data buffer
		 * @param frameUserdataLen: frame user data buffer size in bytes
		 * @return 0 on success, -ENOSYS if not implemented,
		 *         or another negative errno value in case of error
		 */
		virtual int
		loadVideoTexture(IPdraw *pdraw,
				 struct pdraw_video_renderer *renderer,
				 unsigned int textureWidth,
				 unsigned int textureHeight,
				 const struct pdraw_session_info *sessionInfo,
				 const struct vmeta_session *sessionMeta,
				 const struct pdraw_video_frame *frame,
				 const void *frameUserdata,
				 size_t frameUserdataLen) = 0;

		/**
		 * Overlay rendering function. This function is called after the
		 * rendering of the video frame in order to render an
		 * application overlay on top of the video. When HMD distorsion
		 * correction is enabled in the renderer, it is applied after
		 * the overlay rendering. This function is called from the
		 * rendering thread.
		 * If no implementation of this function is required by the
		 * application, -ENOSYS must be returned (before checking
		 * input values, so that implementation can be tested
		 * with all arguments null).
		 * @param pdraw: PDrAW instance handle
		 * @param renderer: renderer handle
		 * @param renderPos: rendering position
		 * @param contentPos: video content position
		 * @param viewMat: 4x4 view matrix
		 * @param projMat: 4x4 projection matrix
		 * @param sessionInfo: session information
		 * @param sessionMeta: session metadata
		 * @param frameMeta: frame metadata
		 * @param frameExtra: frame extra information
		 * @return 0 on success, -ENOSYS if not implemented,
		 *         or another negative errno value in case of error
		 */
		virtual int renderVideoOverlay(
			IPdraw *pdraw,
			struct pdraw_video_renderer *renderer,
			const struct pdraw_rect *renderPos,
			const struct pdraw_rect *contentPos,
			const float *viewMat,
			const float *projMat,
			const struct pdraw_session_info *sessionInfo,
			const struct vmeta_session *sessionMeta,
			const struct vmeta_frame *frameMeta,
			const struct pdraw_video_frame_extra *frameExtra) = 0;
	};


	/* Video sink listener object */
	class VideoSinkListener {
	public:
		/**
		 * Video sink listener object destructor.
		 */
		virtual ~VideoSinkListener(void) {}

		/**
		 * Video sink flush function, called when flushing is required.
		 * When this function is called, the application must flush the
		 * sink queue by calling vbuf_queue_flush() and must return all
		 * buffers outside of the queue by calling vbuf_unref(); once
		 * the flushing is done, the videoSinkQueueFlushed()
		 * function must be called.
		 * @param pdraw: PDrAW instance handle
		 * @param sink: video sink handle
		 */
		virtual void
		onVideoSinkFlush(IPdraw *pdraw,
				 struct pdraw_video_sink *sink) = 0;
	};


	/**
	 * Instance management API
	 */

	/**
	 * Destroy a PDrAW instance.
	 * This function frees all resources associated with a PDrAW instance.
	 * If a successful open operation was made, the close() function must
	 * be called and one must wait for the closeResponse() listener function
	 * to be called prior to destroying the PDrAW instance.
	 */
	virtual ~IPdraw(void) {}


	/**
	 * Session management API
	 */

	/**
	 * Open a session on a URL (stream or local file).
	 * The URL can be either an RTSP URL (starting with "rtsp://") or a
	 * local file path (either absolute or relative).
	 * The function returns before the actual opening is done. If the
	 * function returns 0, the openResponse() listener function will be
	 * called once the open operation is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the openResponse() listener function will
	 * not be called. A failed open operation can be retried, i.e. the open
	 * functions can be called again on the same PDrAW instance.
	 * @param url: URL of the resource to open
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int open(const std::string &url) = 0;

	/**
	 * Open a session on a single stream.
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
	 * The function returns before the actual opening is done. If the
	 * function returns 0, the openResponse() listener function will be
	 * called once the open operation is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the openResponse() listener function will
	 * not be called. A failed open operation can be retried, i.e. the open
	 * functions can be called again on the same PDrAW instance.
	 * @param localAddr: local IP address (optional, can be empty)
	 * @param localStreamPort: local stream (RTP) port (optional, can be 0)
	 * @param localControlPort: local control (RTCP) port (optional,
	 *                          can be 0)
	 * @param remoteAddr: remote IP address (optional, can be empty)
	 * @param remoteStreamPort: remote stream (RTP) port (optional,
	 *                          can be 0)
	 * @param remoteControlPort: remote control (RTCP) port (optional,
	 *                           can be 0)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int open(const std::string &localAddr,
			 uint16_t localStreamPort,
			 uint16_t localControlPort,
			 const std::string &remoteAddr,
			 uint16_t remoteStreamPort,
			 uint16_t remoteControlPort) = 0;

	/**
	 * Open a session on a stream URL through a mux channel.
	 * The URL must be an RTSP URL (starting with "rtsp://"). Mux channels
	 * are used to transfer data between a SkyController remote and a
	 * smartphone through USB; see Parrot's libmux for more information.
	 * No concurrent sessions can run on the mux channel; therefore the user
	 * must take care of limiting the number of PDrAW instances running on
	 * the mux channel to only one.
	 * The function returns before the actual opening is done. If the
	 * function returns 0, the openResponse() listener function will be
	 * called once the open operation is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the openResponse() listener function will
	 * not be called. A failed open operation can be retried, i.e. the open
	 * functions can be called again on the same PDrAW instance.
	 * @param url: URL of the resource to open
	 * @param mux: mux instance handle
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int open(const std::string &url, struct mux_ctx *mux) = 0;

	/**
	 * Close a session.
	 * This function closes a previously opened session (either record
	 * or stream).
	 * The function returns before the actual closing is done. If the
	 * function returns 0, the closeResponse() listener function will be
	 * called once the close is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the closeResponse() listener function
	 * will not be called. After a successful close, the PDrAW instance
	 * must be destroyed.
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int close(void) = 0;

	/**
	 * Get the single stream local stream port.
	 * This function returns the local stream (RTP) port currently in use
	 * after a succesful open operation on a single stream (RTP/AVP) or an
	 * RTSP URL. If no open operation has been done, or if an open operation
	 * has been done on a mux channel, 0 is returned. This is useful when
	 * opening a single stream with null local ports to let PDrAW open
	 * sockets on any available port.
	 * @return the stream port on success, 0 in case of error
	 */
	virtual uint16_t getSingleStreamLocalStreamPort(void) = 0;

	/**
	 * Get the single stream local control port.
	 * This function returns the local control (RTCP) port currently in use
	 * after a succesful open operation on a single stream (RTP/AVP) or an
	 * RTSP URL. If no open operation has been done, or if an open operation
	 * has been done on a mux channel, 0 is returned. This is useful when
	 * opening a single stream with null local ports to let PDrAW open
	 * sockets on any available port.
	 * @return the stream port on success, 0 in case of error
	 */
	virtual uint16_t getSingleStreamLocalControlPort(void) = 0;


	/**
	 * Playback control API
	 */

	/**
	 * Get the ready to play status.
	 * This function returns true if a successful open operation has been
	 * completed and if the playback is ready to start, false otherwise. One
	 * case when a successful open operation is complete but the playback is
	 * not ready is when connected to a SkyController's RTSP server but the
	 * SkyController itself is not yet connected to a drone.
	 * The value returned by this function is identical to the ready
	 * parameter passed to the readyToPlay() listener function when it is
	 * called.
	 * @return the ready to play status on success, false in case of error
	 */
	virtual bool isReadyToPlay(void) = 0;

	/**
	 * Get the pause status.
	 * This function returns true if the playback is currently paused,
	 * false otherwise.
	 * @return the pause status on success, false in case of error
	 */
	virtual bool isPaused(void) = 0;

	/**
	 * Play at the given speed.
	 * This function starts the playback of the video. The speed parameter
	 * is optional and defaults to 1.0. If the speed parameter is negative,
	 * the video is played backward. If the speed is greater than or equal
	 * to PDRAW_PLAY_SPEED_MAX, the speed is ignored and the video is played
	 * at the maximum speed achievable. If the speed is less than or equal
	 * to -PDRAW_PLAY_SPEED_MAX, the speed is ignored and the video is
	 * played backward at the maximum speed achievable. A 0.0 speed has the
	 * same effet as calling the pause() function. On a live stream, the
	 * speed parameter has no effect.
	 * The function returns before the actual operation is done. If the
	 * function returns 0, the playResponse() listener function will be
	 * called once the play is successful (0 status) or has failed (negative
	 * errno status). If the function returns a negative errno value
	 * (immediate failure), the playResponse() listener function will not be
	 * called.
	 * @param speed: playback speed (0.0 means pause, negative value means
	 *               play backward)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int play(float speed = 1.0f) = 0;

	/**
	 * Pause the playback.
	 * This function suspends the playback of the video. The session is not
	 * closed and the playback can be resumed using the play() function.
	 * The function returns before the actual operation is done. If the
	 * function returns 0, the pauseResponse() listener function will be
	 * called once the pause is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the pauseResponse() listener function will
	 * not be called.
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int pause(void) = 0;

	/**
	 * Go to previous frame in frame-by-frame playback.
	 * This function plays the previous frame while the playback is paused.
	 * If the playback is not currently paused an error is returned.
	 * Frame-by-frame is only available on local replays (MP4 records).
	 * The function returns before the actual operation is done. If the
	 * function returns 0, the seekResponse() listener function will be
	 * called once the pause is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the seekResponse() listener function will
	 * not be called.
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int previousFrame(void) = 0;

	/**
	 * Go to next frame in frame-by-frame playback.
	 * This function plays the next frame while the playback is paused.
	 * If the playback is not currently paused an error is returned.
	 * Frame-by-frame is only available on local replays (MP4 records).
	 * The function returns before the actual operation is done. If the
	 * function returns 0, the seekResponse() listener function will be
	 * called once the pause is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the seekResponse() listener function will
	 * not be called.
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int nextFrame(void) = 0;

	/**
	 * Seek forward or backward.
	 * This function seeks forward (positive delta) or backward (negative
	 * delta). The delta parameter is in microseconds. When exact is false
	 * the seek is done to the nearest synchronization sample preceeding the
	 * delta, otherwise the seek is done to the sample nearest to the delta.
	 * Seeking is only available on replays (either local or streamed), not
	 * on live streams.
	 * The function returns before the actual operation is done. If the
	 * function returns 0, the seekResponse() listener function will be
	 * called once the pause is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the seekResponse() listener function will
	 * not be called.
	 * @param delta: time delta in microseconds (positive or negative)
	 * @param exact: true means seek to the sample closest to the delta,
	 *               false means seek to the nearest synchronization sample
	 *               preceeding the delta
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int seek(int64_t delta, bool exact = false) = 0;

	/**
	 * Seek forward.
	 * This function seeks forward by delta microseconds. It has the same
	 * behavior as calling seek() with a positive delta. When exact is
	 * false the seek is done to the nearest synchronization sample
	 * preceeding the delta, otherwise the seek is done to the sample
	 * nearest to the delta. Seeking is only available on replays (either
	 * local or streamed), not on live streams.
	 * The function returns before the actual operation is done. If the
	 * function returns 0, the seekResponse() listener function will be
	 * called once the pause is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the seekResponse() listener function will
	 * not be called.
	 * @param delta: positive time delta forward in microseconds
	 * @param exact: true means seek to the sample closest to the delta,
	 *               false means seek to the nearest synchronization sample
	 *               preceeding the delta
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int seekForward(uint64_t delta, bool exact = false) = 0;

	/**
	 * Seek backward.
	 * This function seeks backward by delta microseconds (postive). It has
	 * the same behavior as calling seek() with a negative delta. When
	 * exact is false the seek is done to the nearest synchronization sample
	 * preceeding the delta, otherwise the seek is done to the sample
	 * nearest to the delta. Seeking is only available on replays (either
	 * local or streamed), not on live streams.
	 * The function returns before the actual operation is done. If the
	 * function returns 0, the seekResponse() listener function will be
	 * called once the pause is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the seekResponse() listener function will
	 * not be called.
	 * @param delta: positive time delta backward in microseconds
	 * @param exact: true means seek to the sample closest to the delta,
	 *               false means seek to the nearest synchronization sample
	 *               preceeding the delta
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int seekBack(uint64_t delta, bool exact = false) = 0;

	/**
	 * Seek to a given time.
	 * This function seeks to the given play timestamp in microseconds. When
	 * exact is false the seek is done to the nearest synchronization sample
	 * preceeding the timestamp, otherwise the seek is done to the sample
	 * nearest to the timestamp. Seeking is only available on replays
	 * (either local or streamed), not on live streams.
	 * The function returns before the actual operation is done. If the
	 * function returns 0, the seekResponse() listener function will be
	 * called once the pause is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the seekResponse() listener function will
	 * not be called.
	 * @param timestamp: play timestamp in microseconds
	 * @param exact: true means seek to the sample closest to the timestamp,
	 *               false means seek to the nearest synchronization sample
	 *               preceeding the timestamp
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int seekTo(uint64_t timestamp, bool exact = false) = 0;

	/**
	 * Get the playback duration.
	 * This function returns the playback duration in microseconds. The
	 * duration is only available on replays (either local or streamed),
	 * not on live streams.
	 * @return the duration in microseconds on success, 0 in case of error
	 */
	virtual uint64_t getDuration(void) = 0;

	/**
	 * Get the playback current time.
	 * This function returns the current playback position in microseconds.
	 * On replays (either local or streamed) this is the position between 0
	 * and the duration; on live streams this is the time since the start of
	 * the stream session.
	 * @return the current time in microseconds on success,
	 *         0 in case of error
	 */
	virtual uint64_t getCurrentTime(void) = 0;


	/**
	 * Video renderer API
	 * @warning: all functions must be called from the application's
	 * rendering thread
	 */

	/**
	 * Start a video renderer.
	 * This function creates a video renderer. Optionally an active EGL
	 * display context can be provided. Once the renderer is created, the
	 * rendering is done by calling the renderVideo() function. Once a
	 * renderer is no longer used it must be destroyed by calling the
	 * stopVideoRenderer() function.
	 * The renderPos parameter sets the position and size of the rendering
	 * in the window/view; these coordinates are in pixels from the
	 * bottom-left corner (OpenGL coordinates). The params structure must
	 * be provided but all parameters are optional and can be left null.
	 * A valid listener must be provided and all functions must be
	 * implemented; the loadVideoTexture() and renderVideoOverlay() are
	 * optional and can return -ENOSYS if no real implementation is
	 * provided. All listener functions are called from the rendering
	 * thread, except the onVideoRenderReady() function which is called
	 * from the pomp_loop thread.
	 * @warning: this function must be called from the application's
	 * rendering thread.
	 * @param renderPos: rendering position and size
	 * @param params: renderer parameters
	 * @param listener: renderer listener functions implementation
	 * @param retObj: renderer handle (output)
	 * @param eglDisplay: EGL display context
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	startVideoRenderer(const struct pdraw_rect *renderPos,
			   const struct pdraw_video_renderer_params *params,
			   IPdraw::VideoRendererListener *listener,
			   struct pdraw_video_renderer **retObj,
			   struct egl_display *eglDisplay = NULL) = 0;

	/**
	 * Stop a video renderer.
	 * This function stops a running video renderer and frees the associated
	 * resources.
	 * @warning: this function must be called from the application's
	 * rendering thread.
	 * @param renderer: renderer handle
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	stopVideoRenderer(struct pdraw_video_renderer *renderer) = 0;

	/**
	 * Resize a video renderer.
	 * This function updates the rendering position and size on a running
	 * video renderer. The render_pos parameter sets the position of the
	 * rendering in the window/view; these coordinates are in pixels from
	 * the bottom-left corner (OpenGL coordinates).
	 * @warning: this function must be called from the application's
	 * rendering thread.
	 * @param renderer: renderer handle
	 * @param renderPos: rendering position and size
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int resizeVideoRenderer(struct pdraw_video_renderer *renderer,
					const struct pdraw_rect *renderPos) = 0;

	/**
	 * Set the video renderer parameters.
	 * This function updates the rendering parameters on a running video
	 * renderer. The params structure must be provided but all parameters
	 * are optional and can be left null.
	 * @warning: this function must be called from the application's
	 * rendering thread.
	 * @param renderer: renderer handle
	 * @param params: renderer parameters
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int setVideoRendererParams(
		struct pdraw_video_renderer *renderer,
		const struct pdraw_video_renderer_params *params) = 0;

	/**
	 * Get the video renderer parameters.
	 * This function retrieves the rendering parameters on a running video
	 * renderer. The provided params structure is filled by the function.
	 * @warning: this function must be called from the application's
	 * rendering thread.
	 * @param renderer: renderer handle
	 * @param params: renderer parameters (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int
	getVideoRendererParams(struct pdraw_video_renderer *renderer,
			       struct pdraw_video_renderer_params *params) = 0;

	/**
	 * Render the video.
	 * This function renders the video with the current rendering position
	 * and rendering parameters, and optionally with the provided view and
	 * projection matrices. The viewMat and projMat matrices are 4x4 OpenGL
	 * matrices.
	 * For render-on-demand, consider using the onVideoRenderReady()
	 * listener function which is called both when a new frame is ready for
	 * rendering and periodically. Note that the onVideoRenderReady()
	 * listener function is called from the pomp_loop thread, not the
	 * rendering thread; the synchronization is up to the caller.
	 * If a contentPos structure is provided, it is filled with the actual
	 * position and size of the video within the rendering position; these
	 * coordinates are in pixels from the bottom-left corner (OpenGL
	 * coordinates).
	 * @warning: this function must be called from the application's
	 * rendering thread.
	 * @param renderer: renderer handle
	 * @param contentPos: video content position (output;
	 *                    optional, can be null)
	 * @param viewMat: 4x4 view matrix (optional, can be null)
	 * @param projMat: 4x4 projection matrix (optional, can be null)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int renderVideo(struct pdraw_video_renderer *renderer,
				struct pdraw_rect *contentPos,
				const float *viewMat = NULL,
				const float *projMat = NULL) = 0;


	/**
	 * Video sink API
	 */

	/**
	 * Start a video sink.
	 * This function creates a video sink on a media of the given mediaId.
	 * The media idenfifiers are known when the onMediaAdded() or
	 * onMediaRemoved() general listener functions are called. Once the
	 * sink is created, video frames are retrieved by getting buffers from
	 * the buffer queue returned by the getVideoSinkQueue() function.
	 * Once a video sink is no longer used, it must be destroyed by calling
	 * the stopVideoSink() function.
	 * The params structure must be provided but all parameters are optional
	 * and can be left null.
	 * The listener must be provided and the onVideoSinkFlush() function
	 * is required to be implemented; all listener functions are called
	 * from the pomp_loop thread. When the onVideoSinkFlush() function is
	 * called, the application must flush the sink queue by calling
	 * vbuf_queue_flush() and must return all buffers outside of the queue
	 * by calling vbuf_unref(); once the flushing is complete, the
	 * videoSinkQueueFlushed() function must be called.
	 * @param mediaId: identifier of the media on which to create the sink
	 * @param params: video sink parameters
	 * @param listener: video sink listener functions implementation
	 * @param retObj: video sink handle (output)
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int startVideoSink(unsigned int mediaId,
				   const struct pdraw_video_sink_params *params,
				   IPdraw::VideoSinkListener *listener,
				   struct pdraw_video_sink **retObj) = 0;

	/**
	 * Stop a video sink.
	 * This function stops a running video sink and frees the associated
	 * resources. A video sink must not be stopped unless all buffers
	 * outside of the queue have been returned by calling vbuf_unref().
	 * Once a video sink is stopped the queue returned by
	 * getVideoSinkQueue() must no longer be used.
	 * @param sink: video sink handle
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int stopVideoSink(struct pdraw_video_sink *sink) = 0;

	/**
	 * Resynchronize a video sink.
	 * This function schedules the output of a synchronization frame (IDR)
	 * for a running video sink. This is only useful for video medias of
	 * type PDRAW_VIDEO_MEDIA_FORMAT_H264. It can be used for example in
	 * case of unrecoverable video decoder errors to restart decoding.
	 * After a video sink creation, the first frame that is output is always
	 * a synchronization frame; therefore it is not necessary to call this
	 * function immediately after a video sink creation.
	 * @param sink: video sink handle
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int resyncVideoSink(struct pdraw_video_sink *sink) = 0;

	/**
	 * Get the video sink buffer queue.
	 * This function returns the buffer queue to use in order to retrieve
	 * frames from a running video sink. Buffers are retrieved from the
	 * queue by using the vbuf_queue_pop() function.
	 * @param sink: video sink handle
	 * @return a pointer on a vbuf_queue object on success,
	 *         NULL in case of error
	 */
	virtual struct vbuf_queue *
	getVideoSinkQueue(struct pdraw_video_sink *sink) = 0;

	/**
	 * Signal that a video sink has been flushed.
	 * This function is used to signal that flushing is complete. When
	 * the onVideoSinkFlush() video sink listener function is called, the
	 * application must flush the sink queue by calling vbuf_queue_flush()
	 * and must return all buffers outside of the queue by calling
	 * vbuf_unref(); once the flushing is complete, this function must
	 * be called.
	 * @param sink: video sink handle
	 * @return 0 on success, negative errno value in case of error
	 */
	virtual int videoSinkQueueFlushed(struct pdraw_video_sink *sink) = 0;


	/**
	 * Metadata API
	 */

	/**
	 * Get the session type.
	 * This function returns the session type, which is either live or
	 * replay, for an opened PDrAW session.
	 * @return the session type, or PDRAW_SESSION_TYPE_UNKNOWN
	 *         in case of error
	 */
	virtual enum pdraw_session_type getSessionType(void) = 0;

	/**
	 * Get the PDrAW instance friendly name.
	 * This function fills the friendlyName string with the friendly name.
	 * The string must have been previously allocated. The friendly name is
	 * generally either the device's friendly name (e.g. "Bob's phone") or
	 * the application's name (e.g. "MyDroneControllerApp"). It is used in
	 * metadata exchanged with a streaming server.
	 * @param friendlyName: pointer to the string to write to (output)
	 */
	virtual void getSelfFriendlyName(std::string *friendlyName) = 0;

	/**
	 * Set the PDrAW instance friendly name.
	 * The friendlyName string is copied internally. The friendly name is
	 * generally either the device's friendly name (e.g. "Bob's phone") or
	 * the application's name (e.g. "MyDroneControllerApp"). It is used in
	 * metadata exchanged with a streaming server. Setting the friendly
	 * name value is optional.
	 * @param friendlyName: friendly name string
	 */
	virtual void setSelfFriendlyName(const std::string &friendlyName) = 0;

	/**
	 * Get the PDrAW instance serial number.
	 * This function fills the serialNumber string with the serial number.
	 * The string must have been previously allocated. The serial number is
	 * generally the unique serial number of the device on which PDrAW is
	 * running. It is used in metadata exchanged with a streaming server.
	 * @param serialNumber: pointer to the string to write to (output)
	 */
	virtual void getSelfSerialNumber(std::string *serialNumber) = 0;

	/**
	 * Set the PDrAW instance serial number.
	 * The serialNumber string is copied internally. The serial number is
	 * generally the unique serial number of the device on which PDrAW is
	 * running. It is used in metadata exchanged with a streaming server.
	 * Setting the serial number value is recommended as it is used as a
	 * unique identifier in the streaming protocols.
	 * @param serialNumber: serial number string
	 */
	virtual void setSelfSerialNumber(const std::string &serialNumber) = 0;

	/**
	 * Get the PDrAW instance software version.
	 * This function fills the softwareVersion string with the software
	 * version. The string must have been previously allocated. The software
	 * version is generally the version number of the application running
	 * PDrAW (e.g. "MyApp v1.2.3"). It is used in metadata exchanged with a
	 * streaming server.
	 * @param softwareVersion: pointer to the string to write to (output)
	 */
	virtual void getSelfSoftwareVersion(std::string *softwareVersion) = 0;

	/**
	 * Set the PDrAW instance software version.
	 * The softwareVersion string is copied internally. The software version
	 * is generally the version number of the application running PDrAW
	 * (e.g. "MyApp v1.2.3"). It is used in metadata exchanged with a
	 * streaming server. Setting the software version value is optional.
	 * @param softwareVersion: software version string
	 */
	virtual void
	setSelfSoftwareVersion(const std::string &softwareVersion) = 0;

	/**
	 * Get the pilot status.
	 * This function returns true if the PDrAW session corresponds to the
	 * drone's pilot, false otherwise. This value must be set by the
	 * application by using the setSelfPilot() function. The default value
	 * is true. This value is mostly used as metadata for the video
	 * renderer listener functions in the pdraw_session_info structure.
	 * @return the pilot status on success, false in case of error
	 */
	virtual bool isSelfPilot(void) = 0;

	/**
	 * Set the pilot status.
	 * The application should set isPilot to true if the PDrAW session
	 * corresponds to the drone's pilot, false otherwise. The default value
	 * is true. This value is mostly used as metadata for the video renderer
	 * listener functions in the pdraw_session_info structure.
	 * @param isPilot: pilot status
	 */
	virtual void setSelfPilot(bool isPilot) = 0;

	/**
	 * Get the peer session metadata.
	 * This function returns the drone's session (i.e. untimed) metadata on
	 * a stream or record. For the session metadata to be known, a session
	 * must be opened. The session structure is filled by the function and
	 * must have been previously allocated.
	 * @param session: pointer to a session metadata structure (output)
	 */
	virtual void getPeerSessionMetadata(struct vmeta_session *session) = 0;

	/**
	 * Get the peer drone model.
	 * This function returns the drone model of a stream or record if it is
	 * known. For the drone model to be known, a session must be opened.
	 * @return the drone model, or PDRAW_DRONE_MODEL_UNKNOWN
	 *         in case of error
	 */
	virtual enum pdraw_drone_model getPeerDroneModel(void) = 0;


	/**
	 * Settings API
	 */

	/**
	 * Get the pipeline mode setting.
	 * This function returns the pipeline mode of a PDrAW instance. The
	 * pipeline mode controls whether to decode the selected video media
	 * (for full processing up to the rendering), or to disable video
	 * decoding (e.g. when no rendering is required, only an H.264 video
	 * sink).
	 * @return the pipeline mode, or PDRAW_PIPELINE_MODE_DECODE_ALL
	 *         in case of error
	 */
	virtual enum pdraw_pipeline_mode getPipelineModeSetting(void) = 0;

	/**
	 * Set the pipeline mode setting.
	 * This function sets the pipeline mode of a PDrAW instance. This
	 * function can be called only prior to any open operation. The pipeline
	 * mode controls whether to decode the selected video media (for full
	 * processing up to the rendering), or to disable video decoding (e.g.
	 * when no rendering is required, only an H.264 video sink).
	 * @param mode: pipeline mode
	 */
	virtual void setPipelineModeSetting(enum pdraw_pipeline_mode mode) = 0;

	/**
	 * Get the display screen settings.
	 * This function returns the display screen settings through the xdpi,
	 * ydpi and deviceMargin* parameters. This is only useful if HMD
	 * distortion correction is enabled in the video rendering. The xdpi
	 * and ydpi are pixel densities in dots per inches. The device margins
	 * are in millimeters.
	 * @param xdpi: horizontal pixel density (output)
	 * @param ydpi: vertical pixel density (output)
	 * @param deviceMarginTop: top device margin (output)
	 * @param deviceMarginBottom: bottom device margin (output)
	 * @param deviceMarginLeft: left device margin (output)
	 * @param deviceMarginRight: right device margin (output)
	 */
	virtual void getDisplayScreenSettings(float *xdpi,
					      float *ydpi,
					      float *deviceMarginTop,
					      float *deviceMarginBottom,
					      float *deviceMarginLeft,
					      float *deviceMarginRight) = 0;

	/**
	 * Set the display screen settings.
	 * This function sets the display screen settings. This is only useful
	 * if HMD distortion correction is enabled in the video rendering.
	 * The xdpi and ydpi are pixel densities in dots per inches. The device
	 * margins are in millimeters.
	 * @param xdpi: horizontal pixel density
	 * @param ydpi: vertical pixel density
	 * @param deviceMarginTop: top device margin
	 * @param deviceMarginBottom: bottom device margin
	 * @param deviceMarginLeft: left device margin
	 * @param deviceMarginRight: right device margin
	 */
	virtual void setDisplayScreenSettings(float xdpi,
					      float ydpi,
					      float deviceMarginTop,
					      float deviceMarginBottom,
					      float deviceMarginLeft,
					      float deviceMarginRight) = 0;

	/**
	 * Get the HMD model setting.
	 * This function returns the head-mounted display (HMD) model. This is
	 * only useful if HMD distortion correction is enabled in the video
	 * rendering.
	 * @return the HMD model, or PDRAW_HMD_MODEL_UNKNOWN in case of error
	 */
	virtual enum pdraw_hmd_model getHmdModelSetting(void) = 0;

	/**
	 * Set the HMD model setting.
	 * This function sets the head-mounted display (HMD) model. This is
	 * only useful if HMD distortion correction is enabled in the video
	 * rendering.
	 * @param hmdModel: HMD model
	 */
	virtual void setHmdModelSetting(enum pdraw_hmd_model hmdModel) = 0;


	/**
	 * Platform-specific API
	 */

	/**
	 * Set the Android JVM pointer.
	 * This function sets the JVM pointer for internal calls to the Android
	 * SDK API. This is only useful on Android and is ignored on other
	 * platforms. If the JVM pointer is not provided on Android platforms,
	 * some features may not be available.
	 * @param jvm: JVM pointer
	 */
	virtual void setAndroidJvm(void *jvm) = 0;
};


/**
 * Create a PDrAW instance.
 * A running pomp_loop must be provided; all functions (with the exception of
 * rendering functions) must be called from the pomp_loop thread. All listener
 * functions are called from the pomp_loop thread; a valid listener must be
 * provided. The instance handle is returned through the retObj parameter.
 * When no longer needed, the instance must be deleted to free the resources.
 * If a successful open operation was made, the close() function must be called
 * and one must wait for the closeResponse() listener function to be called
 * prior to destroying the instance.
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
 * ToString function for enum pdraw_drone_model.
 * @param val: drone model value to convert
 * @return a string description of the drone model
 */
PDRAW_API const char *pdrawDroneModelStr(enum pdraw_drone_model val);


/**
 * ToString function for enum pdraw_hmd_model.
 * @param val: HMD model value to convert
 * @return a string description of the HMD model
 */
PDRAW_API const char *pdrawHmdModelStr(enum pdraw_hmd_model val);


/**
 * ToString function for enum pdraw_pipeline_mode.
 * @param val: pipeline mode value to convert
 * @return a string description of the pipeline mode
 */
PDRAW_API const char *pdrawPipelineModeStr(enum pdraw_pipeline_mode val);


/**
 * ToString function for enum pdraw_session_type.
 * @param val: session type value to convert
 * @return a string description of the session type
 */
PDRAW_API const char *pdrawSessionTypeStr(enum pdraw_session_type val);


/**
 * ToString function for enum pdraw_media_type.
 * @param val: media type value to convert
 * @return a string description of the media type
 */
PDRAW_API const char *pdrawMediaTypeStr(enum pdraw_media_type val);


/**
 * ToString function for enum pdraw_video_media_format.
 * @param val: video media format value to convert
 * @return a string description of the video media format
 */
PDRAW_API const char *
pdrawVideoMediaFormatStr(enum pdraw_video_media_format val);


/**
 * ToString function for enum pdraw_yuv_format.
 * @param val: YUV format value to convert
 * @return a string description of the YUV format
 */
PDRAW_API const char *pdrawYuvFormatStr(enum pdraw_yuv_format val);


/**
 * ToString function for enum pdraw_h264_format.
 * @param val: H.264 format value to convert
 * @return a string description of the H.264 format
 */
PDRAW_API const char *pdrawH264FormatStr(enum pdraw_h264_format val);


/**
 * ToString function for enum pdraw_video_type.
 * @param val: video type value to convert
 * @return a string description of the video type
 */
PDRAW_API const char *pdrawVideoTypeStr(enum pdraw_video_type val);


/**
 * ToString function for enum pdraw_histogram_channel.
 * @param val: histogram channel value to convert
 * @return a string description of the histogram channel
 */
PDRAW_API const char *
pdrawHistogramChannelStr(enum pdraw_histogram_channel val);


/**
 * ToString function for enum pdraw_video_renderer_fill_mode.
 * @param val: video renderer fill mode value to convert
 * @return a string description of the video renderer fill mode
 */
PDRAW_API const char *
pdrawVideoRendererFillModeStr(enum pdraw_video_renderer_fill_mode val);


/**
 * ToString function for enum pdraw_video_renderer_transition_flag.
 * @param val: video renderer transition flag value to convert
 * @return a string description of the video renderer transition flag
 */
PDRAW_API const char *pdrawVideoRendererTransitionFlagStr(
	enum pdraw_video_renderer_transition_flag val);


/**
 * Convert a video frame metadata structure to a JSON object.
 * The JSON object must have been previously created. The function appends
 * new elements in this object.
 * @param frame: pointer to a video frame structure
 * @param jobj: pointer to a JSON object to fill (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdrawVideoFrameToJson(const struct pdraw_video_frame *frame,
				    struct json_object *jobj);


/**
 * Convert a video frame metadata structure to a JSON string.
 * This function fills the str array with the null-terminated JSON string.
 * The string must have been previously allocated. The function writes
 * up to len characters.
 * @param frame: pointer to a video frame structure
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdrawVideoFrameToJsonStr(const struct pdraw_video_frame *frame,
				       char *str,
				       unsigned int len);


/**
 * Pack an YUV video frame structure that points to non-contiguous planes into
 * a contiguous buffer.
 * This function changes the the output buffer capacity if necessary, copies
 * the input video frame into the output buffer and fills the output video
 * frame structure that will point the the output buffer.
 * @param in_frame: pointer to a non-contiguous video frame structure
 * @param out_frame: pointer to an output video frame structure
 * @param out_buf: pointer to an output buffer
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdrawPackYuvFrame(const struct pdraw_video_frame *in_frame,
				struct pdraw_video_frame *out_frame,
				struct vbuf_buffer *out_buf);

} /* namespace Pdraw */

#endif /* !_PDRAW_HPP_ */
