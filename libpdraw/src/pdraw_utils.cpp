/**
 * @file pdraw_utils.cpp
 * @brief Parrot Drones Awesome Video Viewer Library - utilities
 * @date 05/11/2016
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

#include <math.h>
#include <string.h>
#include <h264/h264.h>

#include "pdraw_utils.hpp"

#define ULOG_TAG libpdraw
#include <ulog.h>

#define UTILS_H264_EXTENDED_SAR 255


static const unsigned int pdraw_h264Sar[17][2] =
{
    { 1, 1 },
    { 1, 1 },
    { 12, 11 },
    { 10, 11 },
    { 16, 11 },
    { 40, 33 },
    { 24, 11 },
    { 20, 11 },
    { 32, 11 },
    { 80, 33 },
    { 18, 11 },
    { 15, 11 },
    { 64, 33 },
    { 160, 99 },
    { 4, 3 },
    { 3, 2 },
    { 2, 1 },
};


void pdraw_quatConj(const struct vmeta_quaternion *qSrc, struct vmeta_quaternion *qDst)
{
    if ((!qSrc) || (!qDst))
        return;

    qDst->w = qSrc->w;
    qDst->x = -qSrc->x;
    qDst->y = -qSrc->y;
    qDst->z = -qSrc->z;
}


void pdraw_quatMult(const struct vmeta_quaternion *qA, const struct vmeta_quaternion *qB, struct vmeta_quaternion *qDst)
{
    if ((!qA) || (!qB) || (!qDst))
        return;

    struct vmeta_quaternion tmp;

    tmp.x = qA->x * qB->w + qA->y * qB->z - qA->z * qB->y + qA->w * qB->x;
    tmp.y = -qA->x * qB->z + qA->y * qB->w + qA->z * qB->x + qA->w * qB->y;
    tmp.z = qA->x * qB->y - qA->y * qB->x + qA->z * qB->w + qA->w * qB->z;
    tmp.w = -qA->x * qB->x - qA->y * qB->y - qA->z * qB->z + qA->w * qB->w;

    *qDst = tmp;
}


void pdraw_euler2quat(const struct vmeta_euler *euler, struct vmeta_quaternion *quat)
{
    if ((!euler) || (!quat))
        return;

    float c1, c2, c3, s1, s2, s3, psi, theta, phi;
    float qw, qx, qy, qz, n;
    phi = euler->phi;
    theta = euler->theta;
    psi = euler->psi;
    c1 = cosf(phi / 2);
    s1 = sinf(phi / 2);
    c2 = cosf(theta / 2);
    s2 = sinf(theta / 2);
    c3 = cosf(psi / 2);
    s3 = sinf(psi / 2);
    qw = c1 * c2 * c3 + s1 * s2 * s3;
    qx = s1 * c2 * c3 - c1 * s2 * s3;
    qy = c1 * s2 * c3 + s1 * c2 * s3;
    qz = c1 * c2 * s3 - s1 * s2 * c3;
    n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    if (n != 0.)
    {
        qw /= n;
        qx /= n;
        qy /= n;
        qz /= n;
    }

    quat->w = qw;
    quat->x = qx;
    quat->y = qy;
    quat->z = qz;
}


void pdraw_quat2euler(const struct vmeta_quaternion *quat, struct vmeta_euler *euler)
{
    if ((!quat) || (!euler))
        return;

    float psi, theta, phi;
    float qw, qx, qy, qz;
    qw = quat->w;
    qx = quat->x;
    qy = quat->y;
    qz = quat->z;
    phi = atan2f(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
    theta = asinf(2 * (qw * qy - qz * qx));
    psi = atan2f(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    euler->phi = phi;
    euler->theta = theta;
    euler->psi = psi;
}



float pdraw_wrapToPi(float angle)
{
    if (angle >= 0)
        angle = fmod(angle, 2.f * M_PI);
    else
        angle = (2.f * M_PI - 1 - fmod(-angle - 1, 2. * M_PI));

    if (angle < -M_PI)
        angle += 2.f * M_PI;
    else if (angle > M_PI)
        angle -= 2.f * M_PI;

    return angle;
}


void pdraw_coordsDistanceAndBearing(double latitude1, double longitude1,
                                    double latitude2, double longitude2,
                                    double *distance, double *bearing)
{
    /* http://www.igismap.com/haversine-formula-calculate-geographic-distance-earth/ */
    /* http://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/ */
    double a, c, d, x, y, b;
    double r = 6371000.; // earth radius
    double lat1 = latitude1 * M_PI / 180.;
    double lon1 = longitude1 * M_PI / 180.;
    double lat2 = latitude2 * M_PI / 180.;
    double lon2 = longitude2 * M_PI / 180.;
    a = sin((lat2 - lat1) / 2) * sin((lat2 - lat1) / 2)
        + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2) * sin((lon2 - lon1) / 2);
    c = 2 * atan2(sqrt(a), sqrt(1. - a));
    d = r * c;
    x = cos(lat2) * sin(lon2 - lon1);
    y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
    b = atan2(x, y);
    if (distance) *distance = d;
    if (bearing) *bearing = b;
}


void pdraw_parseLocationString(char *locationStr, struct vmeta_location *location)
{
    if ((!locationStr) || (!location))
        return;

    memset(location, 0, sizeof(*location));

    /* ISO 6709 Annex H string expression */
    char *v = locationStr, *v2 = NULL;
    if (v)
    {
        location->latitude = strtod(v, &v2);
        v = v2;
    }
    if (v)
    {
        location->longitude = strtod(v, &v2);
        v = v2;
    }
    if (v)
    {
        location->altitude = strtod(v, &v2);
        v = v2;
    }

    if ((location->latitude != 500.) && (location->longitude != 500.))
    {
        location->valid = 1;
    }
}


void pdraw_friendlyTimeFromUs(uint64_t time, unsigned int *hrs, unsigned int *min, unsigned int *sec, unsigned int *msec)
{
    unsigned int _hrs = (unsigned int)((time + 500) / 1000 / 60 / 60) / 1000;
    unsigned int _min = (unsigned int)((time + 500) / 1000 / 60 - _hrs * 60000) / 1000;
    unsigned int _sec = (unsigned int)((time + 500) / 1000 - _hrs * 60 * 60000 - _min * 60000) / 1000;
    unsigned int _msec = (unsigned int)((time + 500) / 1000 - _hrs * 60 * 60000 - _min * 60000 - _sec * 1000);
    if (hrs)
        *hrs = _hrs;
    if (min)
        *min = _min;
    if (sec)
        *sec = _sec;
    if (msec)
        *msec = _msec;
}


int pdraw_videoDimensionsFromH264Sps(uint8_t *pSps, unsigned int spsSize,
    unsigned int *width, unsigned int *height,
    unsigned int *cropLeft, unsigned int *cropRight,
    unsigned int *cropTop, unsigned int *cropBottom,
    unsigned int *sarWidth, unsigned int *sarHeight)
{
    struct h264_sps sps;
    int ret = h264_parse_sps(pSps, spsSize, &sps);
    if (ret != 0)
    {
        ULOGE("Utils: h264_parse_sps() failed: %d(%s)", ret, strerror(-ret));
        return -1;
    }

    struct h264_sps_derived sps_derived;
    ret = h264_get_sps_derived(&sps, &sps_derived);
    if (ret != 0)
    {
        ULOGE("Utils: h264_get_sps_derived() failed: %d(%s)", ret, strerror(-ret));
        return -1;
    }

    unsigned int _width = sps_derived.PicWidthInSamplesLuma;
    unsigned int _height = sps_derived.FrameHeightInMbs * 16;
    unsigned int _cropLeft = 0, _cropRight = 0, _cropTop = 0, _cropBottom = 0;
    if (sps.frame_cropping_flag)
    {
        _cropLeft = sps.frame_crop_left_offset * sps_derived.CropUnitX;
        _cropRight = sps.frame_crop_right_offset * sps_derived.CropUnitX;
        _cropTop = sps.frame_crop_top_offset * sps_derived.CropUnitY;
        _cropBottom = sps.frame_crop_bottom_offset * sps_derived.CropUnitY;
    }

    unsigned int _sarWidth = 1, _sarHeight = 1;
    if (sps.vui.aspect_ratio_info_present_flag)
    {
        if (sps.vui.aspect_ratio_idc == UTILS_H264_EXTENDED_SAR)
        {
            _sarWidth = sps.vui.sar_width;
            _sarHeight = sps.vui.sar_height;
        }
        else if (sps.vui.aspect_ratio_idc <= 16)
        {
            _sarWidth = pdraw_h264Sar[sps.vui.aspect_ratio_idc][0];
            _sarHeight = pdraw_h264Sar[sps.vui.aspect_ratio_idc][1];
        }
    }

    if (width)
        *width = _width;
    if (height)
        *height = _height;
    if (cropLeft)
        *cropLeft = _cropLeft;
    if (cropRight)
        *cropRight = _cropRight;
    if (cropTop)
        *cropTop = _cropTop;
    if (cropBottom)
        *cropBottom = _cropBottom;
    if (sarWidth)
        *sarWidth = _sarWidth;
    if (sarHeight)
        *sarHeight = _sarHeight;

    return 0;
}
