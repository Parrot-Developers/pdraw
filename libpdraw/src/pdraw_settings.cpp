/**
 * Parrot Drones Awesome Video Viewer Library
 * User settings
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

#define ULOG_TAG pdraw_settings
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_settings.hpp"

namespace Pdraw {


Settings::Settings(void)
{
	int res;
	pthread_mutexattr_t attr;
	bool attr_created = false;

	mPipelineMode = PDRAW_PIPELINE_MODE_DECODE_ALL;
	mDisplayXdpi = SETTINGS_DISPLAY_XDPI;
	mDisplayYdpi = SETTINGS_DISPLAY_YDPI;
	mDisplayDeviceMarginTop = SETTINGS_DISPLAY_DEVICE_MARGIN;
	mDisplayDeviceMarginBottom = SETTINGS_DISPLAY_DEVICE_MARGIN;
	mDisplayDeviceMarginLeft = SETTINGS_DISPLAY_DEVICE_MARGIN;
	mDisplayDeviceMarginRight = SETTINGS_DISPLAY_DEVICE_MARGIN;
	mHmdModel = PDRAW_HMD_MODEL_UNKNOWN;

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


Settings::~Settings(void)
{
	pthread_mutex_destroy(&mMutex);
}


void Settings::lock(void)
{
	pthread_mutex_lock(&mMutex);
}


void Settings::unlock(void)
{
	pthread_mutex_unlock(&mMutex);
}


void Settings::getFriendlyName(std::string *friendlyName)
{
	if (friendlyName == nullptr)
		return;

	pthread_mutex_lock(&mMutex);
	*friendlyName = mFriendlyName;
	pthread_mutex_unlock(&mMutex);
}


void Settings::setFriendlyName(const std::string &friendlyName)
{
	pthread_mutex_lock(&mMutex);
	mFriendlyName = friendlyName;
	pthread_mutex_unlock(&mMutex);
}


void Settings::getSerialNumber(std::string *serialNumber)
{
	if (serialNumber == nullptr)
		return;

	pthread_mutex_lock(&mMutex);
	*serialNumber = mSerialNumber;
	pthread_mutex_unlock(&mMutex);
}


void Settings::setSerialNumber(const std::string &serialNumber)
{
	pthread_mutex_lock(&mMutex);
	mSerialNumber = serialNumber;
	pthread_mutex_unlock(&mMutex);
}


void Settings::getSoftwareVersion(std::string *softwareVersion)
{
	if (softwareVersion == nullptr)
		return;

	pthread_mutex_lock(&mMutex);
	*softwareVersion = mSoftwareVersion;
	pthread_mutex_unlock(&mMutex);
}


void Settings::setSoftwareVersion(const std::string &softwareVersion)
{
	pthread_mutex_lock(&mMutex);
	mSoftwareVersion = softwareVersion;
	pthread_mutex_unlock(&mMutex);
}


enum pdraw_pipeline_mode Settings::getPipelineMode(void)
{
	pthread_mutex_lock(&mMutex);
	enum pdraw_pipeline_mode ret = mPipelineMode;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void Settings::setPipelineMode(enum pdraw_pipeline_mode mode)
{
	pthread_mutex_lock(&mMutex);
	mPipelineMode = mode;
	pthread_mutex_unlock(&mMutex);
}


void Settings::getDisplayScreenSettings(float *xdpi,
					float *ydpi,
					float *deviceMarginTop,
					float *deviceMarginBottom,
					float *deviceMarginLeft,
					float *deviceMarginRight)
{
	pthread_mutex_lock(&mMutex);
	if (xdpi)
		*xdpi = mDisplayXdpi;
	if (ydpi)
		*ydpi = mDisplayYdpi;
	if (deviceMarginTop)
		*deviceMarginTop = mDisplayDeviceMarginTop;
	if (deviceMarginTop)
		*deviceMarginBottom = mDisplayDeviceMarginBottom;
	if (deviceMarginTop)
		*deviceMarginLeft = mDisplayDeviceMarginLeft;
	if (deviceMarginTop)
		*deviceMarginRight = mDisplayDeviceMarginRight;
	pthread_mutex_unlock(&mMutex);
}


void Settings::setDisplayScreenSettings(float xdpi,
					float ydpi,
					float deviceMarginTop,
					float deviceMarginBottom,
					float deviceMarginLeft,
					float deviceMarginRight)
{
	pthread_mutex_lock(&mMutex);
	mDisplayXdpi = xdpi;
	mDisplayYdpi = ydpi;
	mDisplayDeviceMarginTop = deviceMarginTop;
	mDisplayDeviceMarginBottom = deviceMarginBottom;
	mDisplayDeviceMarginLeft = deviceMarginLeft;
	mDisplayDeviceMarginRight = deviceMarginRight;
	pthread_mutex_unlock(&mMutex);
}


enum pdraw_hmd_model Settings::getHmdModelSetting(void)
{
	pthread_mutex_lock(&mMutex);
	enum pdraw_hmd_model ret = mHmdModel;
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void Settings::setHmdModelSetting(enum pdraw_hmd_model hmdModel)
{
	pthread_mutex_lock(&mMutex);
	mHmdModel = hmdModel;
	pthread_mutex_unlock(&mMutex);
}

} /* namespace Pdraw */
