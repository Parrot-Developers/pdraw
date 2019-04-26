/**
 * Parrot Drones Awesome Video Viewer Library
 * Session metadata
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

#include "pdraw_metadata_session.hpp"

#include <math.h>
#include <string.h>

#include <string>

#define ULOG_TAG pdraw_metasess
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_metasess);

#ifdef __APPLE__
#	ifndef sincosf
#		define sincosf __sincosf
#	endif
#endif

namespace Pdraw {


SessionSelfMetadata::SessionSelfMetadata(void)
{
	int res;
	pthread_mutexattr_t attr;
	bool attr_created = false;

	mIsPilot = true;

	res = pthread_mutexattr_init(&attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_init", res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_settype", res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, &attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		goto error;
	}

	pthread_mutexattr_destroy(&attr);
	return;

error:
	if (attr_created)
		pthread_mutexattr_destroy(&attr);
}


SessionSelfMetadata::~SessionSelfMetadata(void)
{
	pthread_mutex_destroy(&mMutex);
}


void SessionSelfMetadata::lock(void)
{
	pthread_mutex_lock(&mMutex);
}


void SessionSelfMetadata::unlock(void)
{
	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::getFriendlyName(std::string *friendlyName)
{
	if (friendlyName == NULL)
		return;

	pthread_mutex_lock(&mMutex);
	*friendlyName = mFriendlyName;
	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::setFriendlyName(const std::string &friendlyName)
{
	pthread_mutex_lock(&mMutex);
	mFriendlyName = friendlyName;
	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::getSerialNumber(std::string *serialNumber)
{
	if (serialNumber == NULL)
		return;

	pthread_mutex_lock(&mMutex);
	*serialNumber = mSerialNumber;
	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::setSerialNumber(const std::string &serialNumber)
{
	pthread_mutex_lock(&mMutex);
	mSerialNumber = serialNumber;
	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::getSoftwareVersion(std::string *softwareVersion)
{
	if (softwareVersion == NULL)
		return;

	pthread_mutex_lock(&mMutex);
	*softwareVersion = mSoftwareVersion;
	pthread_mutex_unlock(&mMutex);
}


void SessionSelfMetadata::setSoftwareVersion(const std::string &softwareVersion)
{
	pthread_mutex_lock(&mMutex);
	mSoftwareVersion = softwareVersion;
	pthread_mutex_unlock(&mMutex);
}


bool SessionSelfMetadata::isPilot(void)
{
	pthread_mutex_lock(&mMutex);
	bool ret = mIsPilot;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void SessionSelfMetadata::setPilot(bool isPilot)
{
	pthread_mutex_lock(&mMutex);
	mIsPilot = isPilot;
	pthread_mutex_unlock(&mMutex);
}


SessionPeerMetadata::SessionPeerMetadata(void)
{
	int res;
	pthread_mutexattr_t attr;
	bool attr_created = false;

	memset(&mMeta, 0, sizeof(mMeta));
	mDroneModel = PDRAW_DRONE_MODEL_UNKNOWN;

	res = pthread_mutexattr_init(&attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_init", res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_settype", res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, &attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		goto error;
	}

	pthread_mutexattr_destroy(&attr);
	return;

error:
	if (attr_created)
		pthread_mutexattr_destroy(&attr);
}


SessionPeerMetadata::~SessionPeerMetadata(void)
{
	pthread_mutex_destroy(&mMutex);
}


void SessionPeerMetadata::lock(void)
{
	pthread_mutex_lock(&mMutex);
}


void SessionPeerMetadata::unlock(void)
{
	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::get(struct vmeta_session *meta)
{
	if (!meta)
		return;

	pthread_mutex_lock(&mMutex);
	*meta = mMeta;
	pthread_mutex_unlock(&mMutex);
}


void SessionPeerMetadata::set(const struct vmeta_session *meta)
{
	if (!meta)
		return;

	pthread_mutex_lock(&mMutex);

	mMeta = *meta;

	std::string modelId(mMeta.model_id);
	if (!modelId.compare("0901"))
		mDroneModel = PDRAW_DRONE_MODEL_BEBOP;
	else if (!modelId.compare("090c"))
		mDroneModel = PDRAW_DRONE_MODEL_BEBOP2;
	else if (!modelId.compare("090e"))
		mDroneModel = PDRAW_DRONE_MODEL_DISCO;
	else if (!modelId.compare("0916"))
		mDroneModel = PDRAW_DRONE_MODEL_BLUEGRASS;
	else if (!modelId.compare("0914"))
		mDroneModel = PDRAW_DRONE_MODEL_ANAFI;
	else if (!modelId.compare("0919"))
		mDroneModel = PDRAW_DRONE_MODEL_ANAFI_THERMAL;
	if (mDroneModel == PDRAW_DRONE_MODEL_UNKNOWN) {
		std::string model(mMeta.model);
		if (!model.compare("Bebop"))
			mDroneModel = PDRAW_DRONE_MODEL_BEBOP;
		else if (!model.compare("Bebop 2"))
			mDroneModel = PDRAW_DRONE_MODEL_BEBOP2;
		else if (!model.compare("Disco"))
			mDroneModel = PDRAW_DRONE_MODEL_DISCO;
		else if (!model.compare("Bluegrass"))
			mDroneModel = PDRAW_DRONE_MODEL_BLUEGRASS;
		else if (!model.compare("ANAFI"))
			mDroneModel = PDRAW_DRONE_MODEL_ANAFI;
		else if (!model.compare("Anafi"))
			mDroneModel = PDRAW_DRONE_MODEL_ANAFI;
		else if (!model.compare("AnafiThermal"))
			mDroneModel = PDRAW_DRONE_MODEL_ANAFI_THERMAL;
	}
	if (mDroneModel == PDRAW_DRONE_MODEL_UNKNOWN) {
		std::string name(mMeta.friendly_name);
		if (!name.compare("Parrot Bebop"))
			mDroneModel = PDRAW_DRONE_MODEL_BEBOP;
		else if (!name.compare("Parrot Bebop 2"))
			mDroneModel = PDRAW_DRONE_MODEL_BEBOP2;
		else if (!name.compare("Parrot Disco"))
			mDroneModel = PDRAW_DRONE_MODEL_DISCO;
	}

	pthread_mutex_unlock(&mMutex);
}

} /* namespace Pdraw */
