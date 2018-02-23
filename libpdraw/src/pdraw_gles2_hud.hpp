/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 HUD rendering
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PDRAW_GLES2_HUD_HPP_
#define _PDRAW_GLES2_HUD_HPP_

#ifdef USE_GLES2

#include <math.h>
#include "pdraw_gles2_common.hpp"
#include "pdraw_metadata_videoframe.hpp"

namespace Pdraw {


#define GLES2_HUD_TEX_UNIT_COUNT 3

#define GLES2_HUD_DEFAULT_CENTRAL_ZONE_SIZE         (0.25f)
#define GLES2_HUD_DEFAULT_HEADING_ZONE_V_OFFSET     (-0.80f)
#define GLES2_HUD_DEFAULT_ROLL_ZONE_V_OFFSET        (0.50f)
#define GLES2_HUD_DEFAULT_VU_METER_ZONE_H_OFFSET    (-0.60f)
#define GLES2_HUD_DEFAULT_VU_METER_V_INTERVAL       (-0.30f)
#define GLES2_HUD_DEFAULT_RIGHT_ZONE_H_OFFSET       (0.65f)
#define GLES2_HUD_DEFAULT_RADAR_ZONE_H_OFFSET       (0.45f)
#define GLES2_HUD_DEFAULT_RADAR_ZONE_V_OFFSET       (-0.65f)
#define GLES2_HUD_DEFAULT_TEXT_SIZE                 (0.15f)
#define GLES2_HUD_DEFAULT_SMALL_ICON_SIZE           (0.040f)
#define GLES2_HUD_DEFAULT_MEDIUM_ICON_SIZE          (0.050f)
#define GLES2_HUD_DEFAULT_SCALE                     (1.00f)

#define GLES2_HUD_HMD_SCALE                         (1.00f)
#define GLES2_HUD_HMD_CENTRAL_ZONE_SIZE             (0.20f)
#define GLES2_HUD_HMD_HEADING_ZONE_V_OFFSET         (-0.80f)
#define GLES2_HUD_HMD_ROLL_ZONE_V_OFFSET            (0.50f)
#define GLES2_HUD_HMD_VU_METER_ZONE_H_OFFSET        (-0.50f)
#define GLES2_HUD_HMD_VU_METER_V_INTERVAL           (-0.25f)
#define GLES2_HUD_HMD_RIGHT_ZONE_H_OFFSET           (0.55f)
#define GLES2_HUD_HMD_RADAR_ZONE_H_OFFSET           (0.35f)
#define GLES2_HUD_HMD_RADAR_ZONE_V_OFFSET           (-0.65f)
#define GLES2_HUD_HMD_TEXT_SIZE                     (0.14f)
#define GLES2_HUD_HMD_SMALL_ICON_SIZE               (0.037f)
#define GLES2_HUD_HMD_MEDIUM_ICON_SIZE              (0.047f)
#define GLES2_HUD_HMD_SCALE                         (1.00f)

#define GLES2_HUD_DEFAULT_HFOV                      (78.)
#define GLES2_HUD_DEFAULT_VFOV                      (49.)


enum gles2_hud_text_align {
	GLES2_HUD_TEXT_ALIGN_LEFT = 0,
	GLES2_HUD_TEXT_ALIGN_TOP = 0,
	GLES2_HUD_TEXT_ALIGN_CENTER = 1,
	GLES2_HUD_TEXT_ALIGN_MIDDLE = 1,
	GLES2_HUD_TEXT_ALIGN_RIGHT = 2,
	GLES2_HUD_TEXT_ALIGN_BOTTOM = 2,
};


class Session;
class VideoMedia;


class Gles2Hud {
public:
	Gles2Hud(Session *session,
		VideoMedia *media,
		unsigned int firstTexUnit);

	~Gles2Hud(
		void);

	static int getTexUnitCount(
		void) {
		return GLES2_HUD_TEX_UNIT_COUNT;
	}

	int renderHud(
		unsigned int videoWidth,
		unsigned int videoHeight,
		unsigned int windowWidth,
		unsigned int windowHeight,
		const struct vmeta_frame_v2 *metadata,
		bool hmdDistorsionCorrection,
		bool headtracking);

	void setVideoMedia(
		VideoMedia *media);

private:
	int loadTextureFromBuffer(
		const uint8_t *buffer,
		int width,
		int height,
		int texUnit);

	void drawIcon(
		int index,
		float x,
		float y,
		float size,
		float scaleW,
		float scaleH,
		const float color[4]);

	void drawLogo(
		float x,
		float y,
		float size,
		float scaleW,
		float scaleH,
		const float color[4]);

	void getTextDimensions(
		const char *str,
		float size,
		float scaleW,
		float scaleH,
		float *width,
		float *height);

	void drawText(
		const char *str,
		float x,
		float y,
		float size,
		float scaleW,
		float scaleH,
		enum gles2_hud_text_align halign,
		enum gles2_hud_text_align valign,
		const float color[4]);

	void drawLine(
		float x1,
		float y1,
		float x2,
		float y2,
		const float color[4],
		float lineWidth);

	void drawLineZ(
		float x1,
		float y1,
		float z1,
		float x2,
		float y2,
		float z2,
		const float color[4],
		float lineWidth);

	void drawRect(
		float x1,
		float y1,
		float x2,
		float y2,
		const float color[4],
		float lineWidth);

	void drawEllipse(
		float cx,
		float cy,
		float rx,
		float ry,
		int numSegments,
		const float color[4],
		float lineWidth);

	void drawEllipseZ(
		float cx,
		float cy,
		float rx,
		float ry,
		float z,
		int numSegments,
		const float color[4],
		float lineWidth);

	void drawEllipseFilled(
		float cx,
		float cy,
		float rx,
		float ry,
		int numSegments,
		const float color[4]);

	void drawArc(
		float cx,
		float cy,
		float rx,
		float ry,
		float startAngle,
		float spanAngle,
		int numSegments,
		const float color[4],
		float lineWidth);

	void drawArcZ(
		float cx,
		float cy,
		float rx,
		float ry,
		float z,
		float startAngle,
		float spanAngle,
		int numSegments,
		const float color[4],
		float lineWidth);

	void drawVuMeter(
		float x,
		float y,
		float r,
		float value,
		float minVal,
		float maxVal,
		float criticalMin,
		float criticalMax,
		const float color[4],
		const float criticalColor[4],
		float lineWidth);

	int initCockpit(
		void);

	void drawCockpit(
		const float color[4],
		const float color2[4],
		float lineWidth,
		Eigen::Matrix4f &xformMat);

	void drawArtificialHorizon(
		const struct vmeta_euler *drone,
		const struct vmeta_euler *frame,
		const float color[4]);

	void drawRoll(
		float droneRoll,
		const float color[4]);

	void drawHeading(
		float droneYaw,
		float speedHorizRho,
		float speedPsi,
		const float color[4]);

	void drawAltitude(
		double altitude,
		float groundDistance,
		float downSpeed,
		const float color[4]);

	void drawSpeed(
		float horizontalSpeed,
		const float color[4]);

	void drawControllerRadar(
		double distance,
		double bearing,
		float controllerYaw,
		float droneYaw,
		float controllerRadarAngle,
		const float color[4]);

	void drawRecordTimeline(
		uint64_t currentTime,
		uint64_t duration,
		const float color[4]);

	void drawRecordingStatus(
		uint64_t recordingDuration,
		const float color[4]);

	void drawFlightPathVector(
		const struct vmeta_euler *frame,
		float speedTheta,
		float speedPsi,
		const float color[4]);

	void drawPositionPin(
		const struct vmeta_euler *frame,
		double bearing,
		double elevation,
		const float color[4]);

	Session *mSession;
	VideoMedia *mMedia;
	unsigned int mFirstTexUnit;
	float mAspectRatio;
	float mVideoAspectRatio;
	GLint mProgram[2];
	GLint mPositionHandle;
	GLint mTransformMatrixHandle;
	GLint mColorHandle;
	GLuint mLogoTexture;
	unsigned int mLogoTexUnit;
	GLuint mIconsTexture;
	unsigned int mIconsTexUnit;
	GLuint mTextTexture;
	unsigned int mTextTexUnit;
	GLint mTexUniformSampler;
	GLint mTexPositionHandle;
	GLint mTexTexcoordHandle;
	GLint mTexTransformMatrixHandle;
	GLint mTexColorHandle;
	float mHudCentralZoneSize;
	float mHudHeadingZoneVOffset;
	float mHudRollZoneVOffset;
	float mHudVuMeterZoneHOffset;
	float mHudVuMeterVInterval;
	float mHudRightZoneHOffset;
	float mHudRadarZoneHOffset;
	float mHudRadarZoneVOffset;
	float mTextSize;
	float mSmallIconSize;
	float mMediumIconSize;
	float mHudScale;
	float mHfov;
	float mVfov;
	float mScaleW;
	float mScaleH;
	float mRatioW;
	float mRatioH;
	float *mCockpitSphereVertices;
	unsigned int mCockpitSphereVerticesCount;
};

} /* namespace Pdraw */

#endif /* USE_GLES2 */

#endif /* !_PDRAW_GLES2_HUD_HPP_ */
