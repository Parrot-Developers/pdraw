/**
 * Parrot Drones Audio and Video Vector
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

#include "qpdraw_priv.hpp"

#define ULOG_TAG qpdraw
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


namespace QPdraw {
namespace Internal {


QPdrawPriv::QPdrawPriv(QPdraw *parent) : mParent(parent)
{
	int res;
	IPdrawBackend *pdraw = nullptr;

	res = createPdrawBackend(this, &pdraw);
	if (res < 0) {
		ULOG_ERRNO("createPdrawBackend", -res);
		return;
	}

	mPdraw.reset(pdraw);
}


int QPdrawPriv::start()
{
	return mPdraw->start();
}


int QPdrawPriv::stop()
{
	return mPdraw->stop();
}


intptr_t QPdrawPriv::getInternal()
{
	return reinterpret_cast<intptr_t>(mPdraw.get());
}


struct pomp_loop *QPdrawPriv::getLoop()
{
	return mPdraw->getLoop();
}


void QPdrawPriv::stopResponse(IPdraw *pdraw, int status)
{
	PDRAW_UNUSED(pdraw);

	emit mParent->stopResponse(status);
}


void QPdrawPriv::onMediaAdded(IPdraw *pdraw,
			      const struct pdraw_media_info *info,
			      void *elementUserData)
{
	PDRAW_UNUSED(pdraw);

	struct pdraw_media_info info_copy = *info;
	emit mParent->onMediaAdded(info_copy, elementUserData);
}


void QPdrawPriv::onMediaRemoved(IPdraw *pdraw,
				const struct pdraw_media_info *info,
				void *elementUserData)
{
	PDRAW_UNUSED(pdraw);

	struct pdraw_media_info info_copy = *info;
	emit mParent->onMediaRemoved(info_copy, elementUserData);
}


void QPdrawPriv::onSocketCreated(IPdraw *pdraw, int fd)
{
	PDRAW_UNUSED(pdraw);

	emit mParent->onSocketCreated(fd);
}

} /* namespace Internal */


QPdraw::QPdraw(QObject *parent) :
		QObject(parent), mPriv(make_unique<Internal::QPdrawPriv>(this))
{
	qRegisterMetaType<pdraw_media_info>("pdraw_media_info");
}


/* The destructor must be defined here because QPdrawPriv is an
 * incomplete type in the header (PIMPL idiom) */
QPdraw::~QPdraw() = default;


int QPdraw::start()
{
	return mPriv->start();
}


int QPdraw::stop()
{
	return mPriv->stop();
}


intptr_t QPdraw::getInternal()
{
	return mPriv->getInternal();
}


struct pomp_loop *QPdraw::getLoop()
{
	return mPriv->getLoop();
}

} /* namespace QPdraw */
