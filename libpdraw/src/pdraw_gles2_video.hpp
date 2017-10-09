/**
 * @file pdraw_gles2_video.hpp
 * @brief Parrot Drones Awesome Video Viewer Library - OpenGL ES 2.0 video rendering
 * @date 23/11/2016
 * @author aurelien.barre@akaaba.net
 *
 * Copyright (c) 2016 Aurelien Barre <aurelien.barre@akaaba.net>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 * 
 *   * Neither the name of the copyright holder nor the names of the
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PDRAW_GLES2_VIDEO_HPP_
#define _PDRAW_GLES2_VIDEO_HPP_

#ifdef USE_GLES2

#if defined(BCM_VIDEOCORE) || defined(ANDROID_NDK)
    #include <GLES2/gl2.h>
#else
    #define GLFW_INCLUDE_ES2
    #include <GLFW/glfw3.h>
#endif

#include "pdraw_metadata_videoframe.hpp"


#define GLES2_VIDEO_TEX_UNIT_COUNT 3
#define GLES2_VIDEO_FBO_TEX_UNIT_COUNT 1
#define GLES2_VIDEO_PADDING_FBO_WIDTH 32


namespace Pdraw
{


typedef enum
{
    GLES2_VIDEO_COLOR_CONVERSION_NONE = 0,
    GLES2_VIDEO_COLOR_CONVERSION_YUV420PLANAR_TO_RGB,
    GLES2_VIDEO_COLOR_CONVERSION_YUV420SEMIPLANAR_TO_RGB,
    GLES2_VIDEO_COLOR_CONVERSION_MAX,

} gles2_video_color_conversion_t;


class Session;
class VideoMedia;


class Gles2Video
{
public:

    Gles2Video(Session *session, VideoMedia *media, unsigned int firstTexUnit);

    ~Gles2Video();

    static int getTexUnitCount() { return GLES2_VIDEO_TEX_UNIT_COUNT + GLES2_VIDEO_FBO_TEX_UNIT_COUNT; }

    GLuint* getTextures();

    int allocTextures(unsigned int videoWidth, unsigned int videoHeight);

    int loadFrame(uint8_t *framePlane[3], unsigned int frameStride[3],
                  unsigned int frameWidth, unsigned int frameHeight,
                  gles2_video_color_conversion_t colorConversion);

    int renderFrame(uint8_t *framePlane[3], unsigned int frameStride[3],
                    unsigned int frameWidth, unsigned int frameHeight,
                    unsigned int sarWidth, unsigned int sarHeight,
                    unsigned int windowWidth, unsigned int windowHeight,
                    unsigned int windowX, unsigned int windowY,
                    gles2_video_color_conversion_t colorConversion,
                    const struct vmeta_frame_v2 *metadata,
                    bool headtracking, GLuint fbo);

    void setVideoMedia(VideoMedia *media);

private:

    Session *mSession;
    VideoMedia *mMedia;
    unsigned int mFirstTexUnit;
    GLint mProgram[GLES2_VIDEO_COLOR_CONVERSION_MAX];
    GLint mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_MAX];
    GLuint mTextures[3];
    GLint mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_MAX][3];
    GLint mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_MAX];
    GLint mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_MAX];
    GLuint mPaddingFbo;
    GLuint mPaddingFboTexture;
    unsigned int mPaddingWidth;
    unsigned int mPaddingHeight;
};

}

#endif /* USE_GLES2 */

#endif /* !_PDRAW_GLES2_VIDEO_HPP_ */
