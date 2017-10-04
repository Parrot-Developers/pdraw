/**
 * @file pdraw_linux.h
 * @brief Parrot Drones Awesome Video Viewer Linux Application
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

#ifndef _PDRAW_LINUX_H_
#define _PDRAW_LINUX_H_

#include <pthread.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARCommands/ARCommands.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARDiscovery/ARDISCOVERY_AvahiDiscovery.h>
#include <pdraw/pdraw.h>
#include <json-c/json.h>
#include <curl/curl.h>
#ifdef BUILD_SDL
#include <SDL.h>
#endif /* BUILD_SDL */


#define PDRAW_WINDOW_WIDTH      (1280)
#define PDRAW_WINDOW_HEIGHT     (720)

#define PDRAW_HMD_DEFAULT_XDPI           (250.0f)
#define PDRAW_HMD_DEFAULT_YDPI           (250.0f)
#define PDRAW_HMD_DEFAULT_DEVICE_MARGIN  (0.0f)
#define PDRAW_HMD_DEFAULT_IPD            (63.0f)
#define PDRAW_HMD_DEFAULT_SCALE          (0.75f)
#define PDRAW_HMD_DEFAULT_PAN_H          (0.0f)
#define PDRAW_HMD_DEFAULT_PAN_V          (0.0f)

#define PDRAW_ARSDK_DISCOVERY_PORT 44444
#define PDRAW_ARSDK_D2C_PORT 43210

#define PDRAW_ARSDK_CD_NONACK_ID 10
#define PDRAW_ARSDK_CD_ACK_ID 11
#define PDRAW_ARSDK_CD_EMERGENCY_ID 12
#define PDRAW_ARSDK_DC_NAVDATA_ID 127
#define PDRAW_ARSDK_DC_EVENT_ID 126

#define PDRAW_ARSDK_VIDEO_DST_STREAM_PORT 55004
#define PDRAW_ARSDK_VIDEO_DST_CONTROL_PORT 55005

#define PDRAW_CAMERA_ORIENTATION_MIN_INTERVAL 25000


struct arcmd_reader_data;


struct ardiscovery_browser_device
{
    char *serviceName;
    char *serviceType;
    char *ipAddr;
    uint16_t port;
    eARDISCOVERY_PRODUCT product;

    struct ardiscovery_browser_device *prev;
    struct ardiscovery_browser_device *next;
};


struct arcmd_reader_data
{
    struct pdraw_app *app;
    int readerBufferId;
};


struct pdraw_app
{
    struct pdraw *pdraw;
    int run;

    char url[500];
    char ipAddr[16];
    char ifaceAddr[16];
    int playRecord;
    int arsdkBrowse;
    int arsdkConnect;
    eARDISCOVERY_PRODUCT arsdkProduct;
    int arsdkStartStream;
    int scRestream;
    int receiveStream;
    int hmd;
    enum pdraw_hmd_model hmdModel;
    int headtracking;
    struct pdraw_euler headOrientation;
    uint64_t lastCameraOrientationTime;

    ARDISCOVERY_AvahiDiscovery_BrowserData_t *ardiscoveryBrowserData;
    ARNETWORKAL_Manager_t *arnetworkalManager;
    ARNETWORK_Manager_t *arnetworkManager;
    ARCOMMANDS_Decoder_t *arcmdDecoder;
    pthread_mutex_t ardiscoveryBrowserMutex;
    pthread_t ardiscoveryBrowserThread;
    int ardiscoveryBrowserThreadLaunched;
    pthread_t arnetworkRxThread;
    int arnetworkRxThreadLaunched;
    pthread_t arnetworkTxThread;
    int arnetworkTxThreadLaunched;
    pthread_t *arcmdReaderThreads;
    int *arcmdReaderThreadsLaunched;
    struct arcmd_reader_data *arcmdThreadData;
    int arsdkDiscoveryPort;
    int arsdkD2CPort;
    int arsdkC2DPort;

    struct ardiscovery_browser_device *ardiscoveryDeviceList;
    int ardiscoveryDeviceCount;

    int srcStreamPort;
    int srcControlPort;
    int dstStreamPort;
    int dstControlPort;
    int qosMode;
    int disconnected;

#ifdef BUILD_SDL
    SDL_Surface *surface;
    int windowWidth;
    int windowHeight;
    int sdlFlags;
#endif /* BUILD_SDL */
};


int startUi(struct pdraw_app *app);
void stopUi(struct pdraw_app *app);

int startPdraw(struct pdraw_app *app);
void stopPdraw(struct pdraw_app *app);

int startArdiscoveryBrowse(struct pdraw_app *app);
void stopArdiscoveryBrowse(struct pdraw_app *app);
eARDISCOVERY_ERROR ardiscoveryBrowserCallback(void* userdata, uint8_t state, const char* serviceName, const char* serviceType, const char* ipAddr, uint16_t port);
int ardiscoveryConnect(struct pdraw_app *app);
eARDISCOVERY_ERROR ardiscoveryConnectionSendJsonCallback(uint8_t *dataTx, uint32_t *dataTxSize, void *customData);
eARDISCOVERY_ERROR ardiscoveryConnectionReceiveJsonCallback(uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);
int startArnetwork(struct pdraw_app *app);
void stopArnetwork(struct pdraw_app *app);
void arnetworkOnDisconnectCallback(ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData);
eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause);
void *arnetworkCmdReaderRun(void* data);
int startArcommand(struct pdraw_app *app);
void stopArcommand(struct pdraw_app *app);

int sendDateAndTime(struct pdraw_app *app);
int sendAllStates(struct pdraw_app *app);
int sendStreamingVideoEnable(struct pdraw_app *app);
int sendCameraOrientation(struct pdraw_app *app, float pan, float tilt);

void skyControllerSkyControllerStateBatteryChangedCallback(uint8_t percent, void *custom);
void skyControllerSkyControllerStateGpsPositionChangedCallback(double latitude, double longitude, double altitude, float heading, void *custom);
void skyControllerSkyControllerStateAttitudeChangedCallback(float q0, float q1, float q2, float q3, void *custom);

int skyControllerRestreamConnect(struct pdraw_app *app);


#endif /* !_PDRAW_LINUX_H_ */
