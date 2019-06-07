/**
 * Parrot Drones Awesome Video Viewer
 * Qt PDrAW object
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

#ifndef _QPDRAW_HPP_
#define _QPDRAW_HPP_

#include <QObject>

#include <pdraw/pdraw_defs.h>

namespace QPdraw {

/* Forward declarations */
namespace Internal {
class QPdrawPriv;
}


class QPdraw : public QObject {
	Q_OBJECT

public:
	explicit QPdraw(QObject *parent = nullptr);

	~QPdraw();

	/**
	 * Start a QPdraw instance.
	 * This function must be called after the object creation prior to
	 * calling any other function.
	 * @return 0 on success, negative errno value in case of error
	 */
	int start(void);

	/**
	 * Stop a QPdraw instance.
	 * This function must be called prior to destroying the instance.
	 * @return 0 on success, negative errno value in case of error
	 */
	int stop(void);

	/**
	 * Get the internal PDrAW instance pointer.
	 * This function returns a pointer to the internal PDrAW instance.
	 * @return a pointer on the internal PDrAW instance on success,
	 *         null in case of error
	 */
	intptr_t getInternal(void);

	/**
	 * Get the internal event loop.
	 * This function returns a pointer to the event loop used internally.
	 * @return a pointer on the internal loop on success,
	 *         null in case of error
	 */
	struct pomp_loop *getLoop(void);

signals:
	/**
	 * Stop response signal, called when a stop operation is complete or
	 * has failed. The status parameter is the stop operation status: 0 on
	 * success, or a negative errno value in case of error.
	 * @param status: 0 on success, negative errno value in case of error
	 */
	void stopResponse(int status);

	/**
	 * Media added signal, called when a media has been added internally in
	 * the PDrAW pipeline. Medias are for example YUV or H.264 video medias.
	 * The info structure gives the media identifier that can be used to
	 * create a video sink on this media.
	 * @param info: information on the media
	 */
	void onMediaAdded(struct pdraw_media_info info);

	/**
	 * Media removed signal, called when a media has been removed internally
	 * from the PDrAW pipeline. Medias are for example YUV or H.264 video
	 * medias. When a media is removed, any video sink created on this media
	 * must then be stopped.
	 * @param info: information on the media
	 */
	void onMediaRemoved(struct pdraw_media_info info);

	/**
	 * Socket creation signal, called immediately after a socket creation
	 * with its file descriptor as parameter.
	 * @param fd: socket file descriptor
	 */
	void onSocketCreated(int fd);

private:
	Internal::QPdrawPriv *mPriv;
};

} /* namespace QPdraw */

#endif /* !_QPDRAW_HPP_ */
