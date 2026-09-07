/**
 * Parrot Drones Audio and Video Vector library
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

#include "pdraw_settings.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


void Settings::getFriendlyName(std::string *friendlyName)
{
	if (friendlyName == nullptr)
		return;

	std::scoped_lock lock(mMutex);
	*friendlyName = mFriendlyName;
}


void Settings::setFriendlyName(const std::string &friendlyName)
{
	std::scoped_lock lock(mMutex);
	mFriendlyName = friendlyName;
}


void Settings::getSerialNumber(std::string *serialNumber)
{
	if (serialNumber == nullptr)
		return;

	std::scoped_lock lock(mMutex);
	*serialNumber = mSerialNumber;
}


void Settings::setSerialNumber(const std::string &serialNumber)
{
	std::scoped_lock lock(mMutex);
	mSerialNumber = serialNumber;
}


void Settings::getSoftwareVersion(std::string *softwareVersion)
{
	if (softwareVersion == nullptr)
		return;

	std::scoped_lock lock(mMutex);
	*softwareVersion = mSoftwareVersion;
}


void Settings::setSoftwareVersion(const std::string &softwareVersion)
{
	std::scoped_lock lock(mMutex);
	mSoftwareVersion = softwareVersion;
}

} /* namespace Pdraw */
