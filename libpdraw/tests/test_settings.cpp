/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Settings (Tier A)
 *
 * Copyright (c) 2026 Parrot Drones SAS
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

#include "pdraw_settings.hpp"
#include "test_common.h"

#include <atomic>
#include <string>
#include <thread>

using Pdraw::Settings;


static void testSettingsDefaultsAreEmpty()
{
	Settings s;
	std::string val;

	s.getFriendlyName(&val);
	CU_ASSERT(val.empty());

	s.getSerialNumber(&val);
	CU_ASSERT(val.empty());

	s.getSoftwareVersion(&val);
	CU_ASSERT(val.empty());
}


static void testSettingsSetGetFriendlyName()
{
	Settings s;
	std::string val;

	s.setFriendlyName("MyDrone");
	s.getFriendlyName(&val);
	CU_ASSERT_STRING_EQUAL(val.c_str(), "MyDrone");
}


static void testSettingsSetGetSerialNumber()
{
	Settings s;
	std::string val;

	s.setSerialNumber("SN-12345");
	s.getSerialNumber(&val);
	CU_ASSERT_STRING_EQUAL(val.c_str(), "SN-12345");
}


static void testSettingsSetGetSoftwareVersion()
{
	Settings s;
	std::string val;

	s.setSoftwareVersion("7.0.1");
	s.getSoftwareVersion(&val);
	CU_ASSERT_STRING_EQUAL(val.c_str(), "7.0.1");
}


static void testSettingsGetNullPointerSafe()
{
	/* Passing nullptr to getters must not crash. */
	Settings s;
	s.setFriendlyName("X");
	s.getFriendlyName(nullptr);
	s.getSerialNumber(nullptr);
	s.getSoftwareVersion(nullptr);
	CU_PASS("null pointer getters did not crash");
}


static void testSettingsConcurrentReadWrite()
{
	Settings s;
	s.setFriendlyName("initial");

	auto writer = [&s]() {
		for (int i = 0; i < 1000; i++)
			s.setFriendlyName(i % 2 ? "even" : "odd");
	};
	auto reader = [&s]() {
		std::string v;
		for (int i = 0; i < 1000; i++)
			s.getFriendlyName(&v);
	};

	std::thread t1(writer);
	std::thread t2(reader);
	t1.join();
	t2.join();

	/* Just reaching here without ASAN/TSAN errors is the pass condition. */
	CU_PASS("concurrent read/write did not crash or deadlock");
}


CU_TestInfo g_pdraw_test_settings[] = {
	{FN("testSettingsDefaultsAreEmpty"), testSettingsDefaultsAreEmpty},
	{FN("testSettingsSetGetFriendlyName"), testSettingsSetGetFriendlyName},
	{FN("testSettingsSetGetSerialNumber"), testSettingsSetGetSerialNumber},
	{FN("testSettingsSetGetSoftwareVersion"),
	 testSettingsSetGetSoftwareVersion},
	{FN("testSettingsGetNullPointerSafe"), testSettingsGetNullPointerSafe},
	{FN("testSettingsConcurrentReadWrite"),
	 testSettingsConcurrentReadWrite},
	CU_TEST_INFO_NULL,
};
