/**
 * @file pdraw_linux.c
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

#include "pdraw_linux.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>
#include <math.h>

#define ULOG_TAG pdraw_app
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_app);


enum args_id {
    ARGS_ID_HMD = 256,
    ARGS_ID_HEADTRACK,
};


static const char short_options[] = "hu:f:bk:K:i:m:s:c:S:C:n:";


static const struct option long_options[] =
{
    { "help"            , no_argument        , NULL, 'h' },
    { "url"             , required_argument  , NULL, 'u' },
    { "file"            , required_argument  , NULL, 'f' },
    { "arsdk-browse"    , no_argument        , NULL, 'b' },
    { "arsdk"           , required_argument  , NULL, 'k' },
    { "arsdk-start"     , required_argument  , NULL, 'K' },
    { "ip"              , required_argument  , NULL, 'i' },
    { "miface"          , required_argument  , NULL, 'm' },
    { "srcstrmp"        , required_argument  , NULL, 's' },
    { "srcctrlp"        , required_argument  , NULL, 'c' },
    { "dststrmp"        , required_argument  , NULL, 'S' },
    { "dstctrlp"        , required_argument  , NULL, 'C' },
    { "screstream"      , required_argument  , NULL, 'n' },
    { "hmd"             , required_argument  , NULL, ARGS_ID_HMD },
    { "headtrack"       , no_argument        , NULL, ARGS_ID_HEADTRACK },
    { 0, 0, 0, 0 }
};


static ARNETWORK_IOBufferParam_t c2dParams[] =
{
    /* Non-acknowledged commands. */
    {
        .ID = PDRAW_ARSDK_CD_NONACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 2,
        .dataCopyMaxSize = 128,
        .isOverwriting = 1,
    },
    /* Acknowledged commands. */
    {
        .ID = PDRAW_ARSDK_CD_ACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Emergency commands. */
    {
        .ID = PDRAW_ARSDK_CD_EMERGENCY_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 10,
        .ackTimeoutMs = 100,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 1,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
};

static const size_t c2dParamsCount = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);


static ARNETWORK_IOBufferParam_t d2cParams[] =
{
    {
        .ID = PDRAW_ARSDK_DC_NAVDATA_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    {
        .ID = PDRAW_ARSDK_DC_EVENT_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
};

static const size_t d2cParamsCount = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);


static int commandBufferIds[] = {
    PDRAW_ARSDK_DC_NAVDATA_ID,
    PDRAW_ARSDK_DC_EVENT_ID,
};

static const size_t commandBufferIdsCount = sizeof(commandBufferIds) / sizeof(int);


static int stopping = 0;


static void sighandler(int signum)
{
    printf("Stopping PDrAW...\n");
    ULOGI("Stopping...");
    stopping = 1;
    signal(SIGINT, SIG_DFL);
}


static void welcome()
{
    printf("  ___ ___       ___      __ \n");
    printf(" | _ \\   \\ _ _ /_\\ \\    / / \n");
    printf(" |  _/ |) | '_/ _ \\ \\/\\/ /  \n");
    printf(" |_| |___/|_|/_/ \\_\\_/\\_/   \n");
    printf("\nParrot Drones Awesome Video Viewer\n\n");
}


static void usage(int argc, char *argv[])
{
    printf("Usage: %s [options]\n"
            "Options:\n"
            "-h | --help                        Print this message\n"
            "-u | --url <url>                   Stream from a URL\n"
            "-f | --file <file_name>            Offline MP4 file playing\n"
            "-b | --arsdk-browse                Browse for ARSDK devices (discovery)\n"
            "-k | --arsdk <ip_address>          ARSDK connection to drone with its IP address\n"
            "-K | --arsdk-start <ip_address>    ARSDK connection to drone with its IP address (connect only, do not process the stream)\n"
            "-i | --ip <ip_address>             Direct RTP/AVP H.264 reception with an IP address (ports must also be configured)\n"
            "-m | --miface <ip_address>         Multicast interface address (only in case of multicast reception)\n"
            "-s | --srcstrmp <port>             Source stream port for direct RTP/AVP reception\n"
            "-c | --srcctrlp <port>             Source control port for direct RTP/AVP reception\n"
            "-S | --dststrmp <port>             Destination stream port for direct RTP/AVP reception\n"
            "-C | --dstctrlp <port>             Destination control port for direct RTP/AVP reception\n"
            "-n | --screstream <ip_address>     Connexion to a RTP restream from a SkyController\n"
            "     --hmd <model>                 HMD distorsion correction with model id (0=Parrot Cockpit Glasses)\n"
            "     --headtrack                   Enable headtracking\n"
            "\n",
            argv[0]);
}


static void summary(struct pdraw_app* app, int afterBrowse)
{
    if ((app->arsdkBrowse) && (!strlen(app->ipAddr)) && (!afterBrowse))
    {
        printf("Browse for ARSDK devices (discovery)\n\n");
    }
    else if (app->scRestream)
    {
        printf("Connection to a restream from SkyController (address %s)\n\n", app->ipAddr);
    }
    else if (app->arsdkConnect)
    {
        printf("ARSDK connection to address %s", app->ipAddr);
        if (!app->receiveStream) printf(" (start stream only)");
        printf("\n\n");
    }
    else if ((app->receiveStream) && (strlen(app->url)))
    {
        printf("Streaming from URL '%s'\n\n", app->url);
    }
    else if (app->receiveStream)
    {
        printf("Direct RTP/AVP H.264 reception from address %s\n", app->ipAddr);
        printf("source ports: %d, %d | destination ports: %d, %d\n\n",
               app->srcStreamPort, app->srcControlPort, app->dstStreamPort, app->dstControlPort);
    }
    else if (app->playRecord)
    {
        printf("Offline playing of MP4 file '%s'\n\n", app->url);
    }
    else
    {
        printf("Nothing to do...\n\n");
    }
}


int main(int argc, char *argv[])
{
    int failed = 0;
    int idx, c, mouseDown = 0, isRecording = 0;
    int mouseDownX = 0, mouseDownY = 0;
    pdraw_euler_t mouseDownHeadOrientation;
    uint64_t lastRenderTime = 0;
    struct pdraw_app *app;

    welcome();

    if (argc < 2)
    {
        usage(argc, argv);
        exit(EXIT_FAILURE);
    }

    app = (struct pdraw_app*)malloc(sizeof(struct pdraw_app));
    if (app != NULL)
    {
        /* Initialize configuration */
        memset(app, 0, sizeof(struct pdraw_app));
        app->arsdkDiscoveryPort = PDRAW_ARSDK_DISCOVERY_PORT;
        app->arsdkD2CPort = PDRAW_ARSDK_D2C_PORT;
        app->arsdkC2DPort = 0; // Will be read from json
        app->dstStreamPort = PDRAW_ARSDK_VIDEO_DST_STREAM_PORT;
        app->dstControlPort = PDRAW_ARSDK_VIDEO_DST_CONTROL_PORT;
        app->run = 1;
#ifdef BUILD_SDL
        app->windowWidth = PDRAW_WINDOW_WIDTH;
        app->windowHeight = PDRAW_WINDOW_HEIGHT;
#endif /* BUILD_SDL */
    }
    else
    {
        failed = 1;
        ULOGE("pdraw app alloc error!");
    }

    if (!failed)
    {
        /* Command-line parameters */
        while ((c = getopt_long(argc, argv, short_options, long_options, &idx)) != -1)
        {
            switch (c)
            {
                case 0:
                    break;

                case 'h':
                    usage(argc, argv);
                    free(app);
                    exit(EXIT_SUCCESS);
                    break;

                case 'n':
                    strncpy(app->ipAddr, optarg, sizeof(app->ipAddr));
                    app->scRestream = 1;
                    app->receiveStream = 1;
                    break;

                case 'b':
                    app->arsdkBrowse = 1;
                    break;

                case 'k':
                    strncpy(app->ipAddr, optarg, sizeof(app->ipAddr));
                    app->arsdkBrowse = 1;
                    app->arsdkConnect = 1;
                    app->arsdkStartStream = 1;
                    app->receiveStream = 1;
                    break;

                case 'K':
                    strncpy(app->ipAddr, optarg, sizeof(app->ipAddr));
                    app->arsdkBrowse = 1;
                    app->arsdkConnect = 1;
                    app->arsdkStartStream = 1;
                    break;

                case 'i':
                    strncpy(app->ipAddr, optarg, sizeof(app->ipAddr));
                    app->receiveStream = 1;
                    break;

                case 'm':
                    strncpy(app->ifaceAddr, optarg, sizeof(app->ifaceAddr));
                    break;

                case 'f':
                    strncpy(app->url, optarg, sizeof(app->url));
                    app->playRecord = 1;
                    break;

                case 'u':
                    strncpy(app->url, optarg, sizeof(app->url));
                    app->receiveStream = 1;
                    break;

                case 's':
                    sscanf(optarg, "%d", &app->srcStreamPort);
                    break;

                case 'c':
                    sscanf(optarg, "%d", &app->srcControlPort);
                    break;

                case 'S':
                    sscanf(optarg, "%d", &app->dstStreamPort);
                    break;

                case 'C':
                    sscanf(optarg, "%d", &app->dstControlPort);
                    break;

                case ARGS_ID_HMD:
                    sscanf(optarg, "%d", (int*)&app->hmdModel);
                    app->hmd = 1;
                    break;

                case ARGS_ID_HEADTRACK:
                    app->headtracking = 1;
                    break;

                default:
                    usage(argc, argv);
                    free(app);
                    exit(EXIT_FAILURE);
                    break;
            }
        }
    }

    if ((!app->arsdkBrowse) && ((!strlen(app->ipAddr))
            && ((!app->url) || (!strlen(app->url)))))
    {
        failed = 1;
        fprintf(stderr, "Invalid address or file name\n\n");
        usage(argc, argv);
        ULOGE("invalid address or file name!");
    }

    if (!failed)
    {
        summary(app, 0);

        printf("Starting PDrAW...\n");
        ULOGI("Starting...");

        signal(SIGINT, sighandler);
    }

    if (!failed)
    {
        failed = startUi(app);
    }

    if ((!failed) && (app->arsdkBrowse))
    {
        failed = startArdiscoveryBrowse(app);

        int selected = 0;

        while ((!failed) && (!stopping) && (!selected))
        {
            if (!strlen(app->ipAddr))
            {
#ifdef BUILD_SDL
                SDL_Event event;
                while ((!failed) && (!stopping) && (!selected) && (SDL_PollEvent(&event)))
                {
                    switch(event.type)
                    {
                        case SDL_QUIT:
                            stopping = 1;
                            break;
                        case SDL_VIDEORESIZE:
                        {
                            app->windowWidth = event.resize.w;
                            app->windowHeight = event.resize.h;
                            app->surface = SDL_SetVideoMode(app->windowWidth, app->windowHeight, 0, app->sdlFlags);
                            if (app->surface == NULL)
                            {
                                ULOGE("SDL_SetVideoMode() failed: %s", SDL_GetError());
                            }
                            break;
                        }
                        case SDL_KEYDOWN:
                        {
                            int idx = -1;
                            if (event.key.keysym.sym == SDLK_ESCAPE)
                            {
                                stopping = 1;
                            }
                            else if ((event.key.keysym.sym >= SDLK_1) && (event.key.keysym.sym <= SDLK_9))
                            {
                                idx = event.key.keysym.sym - SDLK_1;
                            }
                            else if ((event.key.keysym.sym >= SDLK_KP1) && (event.key.keysym.sym <= SDLK_KP9))
                            {
                                idx = event.key.keysym.sym - SDLK_KP1;
                            }

                            pthread_mutex_lock(&app->ardiscoveryBrowserMutex);

                            if ((idx >= 0) && (idx < app->ardiscoveryDeviceCount))
                            {
                                struct ardiscovery_browser_device *device;
                                int i;
                                for (device = app->ardiscoveryDeviceList, i = 0; device && i < idx; device = device->next, i++);

                                if ((device) && (i == idx))
                                {
                                    switch (device->product)
                                    {
                                        case ARDISCOVERY_PRODUCT_ARDRONE:
                                        case ARDISCOVERY_PRODUCT_BEBOP_2:
                                        case ARDISCOVERY_PRODUCT_EVINRUDE:
                                        case ARDISCOVERY_PRODUCT_SKYCONTROLLER:
                                        case ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG:
                                        case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2:
                                            strncpy(app->ipAddr, device->ipAddr, sizeof(app->ipAddr));
                                            app->arsdkDiscoveryPort = device->port;
                                            app->arsdkConnect = 1;
                                            app->arsdkProduct = device->product;
                                            app->arsdkStartStream = 1;
                                            app->receiveStream = 1;
                                            selected = 1;
                                            break;
                                        default:
                                            break;
                                    }
                                }
                            }

                            pthread_mutex_unlock(&app->ardiscoveryBrowserMutex);

                            break;
                        }
                    }
                }
#else /* BUILD_SDL */
                sleep(1);
#endif /* BUILD_SDL */
            }
            else
            {
                pthread_mutex_lock(&app->ardiscoveryBrowserMutex);

                struct ardiscovery_browser_device *device;
                int i;
                for (device = app->ardiscoveryDeviceList; device; device = device->next, i++)
                {
                    if (!strncmp(app->ipAddr, device->ipAddr, sizeof(app->ipAddr)))
                    {
                        app->arsdkDiscoveryPort = device->port;
                        app->arsdkConnect = 1;
                        app->arsdkProduct = device->product;
                        app->arsdkStartStream = 1;
                        app->receiveStream = 1;
                        selected = 1;
                        break;
                    }
                }

                pthread_mutex_unlock(&app->ardiscoveryBrowserMutex);

                if (!selected)
                {
                    usleep(100000);
                }
            }
        }

        if (selected)
        {
            ARDISCOVERY_AvahiDiscovery_StopBrowsing(app->ardiscoveryBrowserData);
            summary(app, 1);
        }
        else
        {
            failed = 1;
        }
    }

    if ((!failed) && (app->scRestream))
    {
        failed = skyControllerRestreamConnect(app);
    }

    if (app->arsdkConnect)
    {
        if (!failed)
        {
            failed = ardiscoveryConnect(app);
        }

        if (!failed)
        {
            failed = startArnetwork(app);
        }
    }

    if ((!failed) && ((app->receiveStream) || (app->playRecord)))
    {
        failed = startPdraw(app);
    }

    if (app->arsdkConnect)
    {
        if (!failed)
        {
            failed = startArcommand(app);
        }

        if (!failed)
        {
            int cmdSend = 0;

            cmdSend = sendDateAndTime(app);
        }

        if (!failed)
        {
            int cmdSend = 0;

            cmdSend = sendAllStates(app);
        }

        if ((!failed) && (app->arsdkStartStream))
        {
            int cmdSend = 0;

            cmdSend = sendStreamingVideoEnable(app);
        }
    }

    if (!failed)
    {
        printf("Rock'n'roll!\n");
        ULOGI("Running...");
    }

    /* Run until interrupted */
    while ((!failed) && (!stopping) && (!app->disconnected))
    {
#ifdef BUILD_SDL
        SDL_Event event;
        while ((!failed) && (!stopping) && (SDL_PollEvent(&event)))
        {
            switch(event.type)
            {
                case SDL_QUIT:
                    stopping = 1;
                    break;
                case SDL_VIDEORESIZE:
                {
                    app->windowWidth = event.resize.w;
                    app->windowHeight = event.resize.h;
                    app->surface = SDL_SetVideoMode(app->windowWidth, app->windowHeight, 0, app->sdlFlags);
                    if (app->surface == NULL)
                    {
                        ULOGE("SDL_SetVideoMode() failed: %s", SDL_GetError());
                    }
                    else
                    {
                        int ret = pdraw_start_renderer(app->pdraw,
                                                       app->windowWidth, app->windowHeight, 0, 0,
                                                       app->windowWidth, app->windowHeight,
                                                       app->hmd, app->headtracking, NULL);
                        if (ret < 0)
                        {
                            ULOGE("pdraw_start_renderer() failed (%d)", ret);
                        }
                        ret = 0;
                    }
                    break;
                }
                case SDL_KEYDOWN:
                    switch(event.key.keysym.sym)
                    {
                        case SDLK_ESCAPE:
                            stopping = 1;
                            break;
                        case SDLK_SPACE:
                            if (pdraw_is_paused(app->pdraw) == 0)
                            {
                                int ret = pdraw_pause(app->pdraw);
                                if (ret != 0)
                                {
                                    ULOGW("pdraw_pause() failed (%d)", ret);
                                }
                            }
                            else
                            {
                                int ret = pdraw_start(app->pdraw);
                                if (ret != 0)
                                {
                                    ULOGW("pdraw_start() failed (%d)", ret);
                                }
                            }
                            break;
                        case SDLK_RIGHT:
                        {
                            int ret = pdraw_seek_forward(app->pdraw, 10000000);
                            if (ret != 0)
                            {
                                ULOGW("pdraw_seek_forward() failed (%d)", ret);
                            }
                            break;
                        }
                        case SDLK_LEFT:
                        {
                            int ret = pdraw_seek_back(app->pdraw, 10000000);
                            if (ret != 0)
                            {
                                ULOGW("pdraw_seek_back() failed (%d)", ret);
                            }
                            break;
                        }
                        case SDLK_HOME:
                        {
                            int ret = pdraw_seek_to(app->pdraw, 0);
                            if (ret != 0)
                            {
                                ULOGW("pdraw_seek_to() failed (%d)", ret);
                            }
                            break;
                        }
                        case SDLK_END:
                        {
                            int ret = pdraw_seek_to(app->pdraw, (uint64_t)-1);
                            if (ret != 0)
                            {
                                ULOGW("pdraw_seek_to() failed (%d)", ret);
                            }
                            break;
                        }
                        case SDLK_r:
                        {
                            if (!isRecording)
                            {
                                int ret = pdraw_start_recorder(app->pdraw, "record.mp4");
                                if (ret != 0)
                                {
                                    ULOGW("pdraw_start_recorder() failed (%d)", ret);
                                }
                                else
                                {
                                    isRecording = 1;
                                }
                            }
                            else
                            {
                                int ret = pdraw_stop_recorder(app->pdraw);
                                if (ret != 0)
                                {
                                    ULOGW("pdraw_stop_recorder() failed (%d)", ret);
                                }
                                else
                                {
                                    isRecording = 0;
                                }
                            }
                            break;
                        }
                        case SDLK_RETURN:
                            /*options ^= SDL_FULLSCREEN;
                            screen = SDL_SetVideoMode(width, height, 0, options);*/
                            break;
                        default:
                            break;
                    }
                    break;
                case SDL_MOUSEBUTTONDOWN:
                    if ((event.button.button == SDL_BUTTON_LEFT) || (event.button.button == SDL_BUTTON_RIGHT))
                    {
                        mouseDown = 1;
                        mouseDownX = event.button.x;
                        mouseDownY = event.button.y;
                        pdraw_get_self_head_orientation_euler(app->pdraw, &mouseDownHeadOrientation);
                        app->headOrientation = mouseDownHeadOrientation;
                    }
                    break;
                case SDL_MOUSEBUTTONUP:
                    if ((event.button.button == SDL_BUTTON_LEFT) || (event.button.button == SDL_BUTTON_RIGHT))
                    {
                        mouseDown = 0;
                    }
                    break;
                case SDL_MOUSEMOTION:
                    if ((mouseDown) && (event.motion.x != 0) && (event.motion.x != app->windowWidth - 1) &&
                        (event.motion.y != 0) && (event.motion.y != app->windowHeight - 1))
                    {
                        int deltaX = (int)event.motion.x - mouseDownX;
                        int deltaY = mouseDownY - (int)event.motion.y;
                        app->headOrientation = mouseDownHeadOrientation;
                        if (event.motion.state & SDL_BUTTON(SDL_BUTTON_LEFT))
                        {
                            app->headOrientation.psi += (float)deltaX / (float)app->windowWidth * 78. * M_PI / 180.;
                            app->headOrientation.theta += (float)deltaY / (float)app->windowHeight * 49. * M_PI / 180;
                        }
                        else if (event.motion.state & SDL_BUTTON(SDL_BUTTON_RIGHT))
                        {
                            app->headOrientation.phi += (float)deltaX / (float)app->windowWidth * 180. * M_PI / 180.;
                        }
                    }
                    break;
            }
        }

        struct timespec t1;
        clock_gettime(CLOCK_MONOTONIC, &t1);
        uint64_t curTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
        pdraw_set_self_head_orientation_euler(app->pdraw, &app->headOrientation);
        if ((app->arsdkConnect) && (app->headtracking) && (curTime >= app->lastCameraOrientationTime + PDRAW_CAMERA_ORIENTATION_MIN_INTERVAL))
        {
            float pan = 0., tilt = 0.;
            int ret = pdraw_get_camera_orientation_for_headtracking(app->pdraw, &pan, &tilt);
            if (ret >= 0)
            {
                sendCameraOrientation(app, pan * 180. / M_PI, tilt * 180. / M_PI);
                app->lastCameraOrientationTime = curTime;
            }
        }

        if (app->pdraw)
        {
            int ret = pdraw_render(app->pdraw, lastRenderTime);
            if (ret < 0)
            {
                ULOGE("pdraw_render() failed (%d)", ret);
                failed = 1;
            }
        }

        SDL_GL_SwapBuffers();
        clock_gettime(CLOCK_MONOTONIC, &t1);
        lastRenderTime = (uint64_t)t1.tv_sec * 1000000 + (uint64_t)t1.tv_nsec / 1000;
#else /* BUILD_SDL */
        sleep(1);
#endif /* BUILD_SDL */
    }

    printf("Terminating PDrAW...\n");
    ULOGI("Terminating...");

    if (app != NULL)
    {
        app->run = 0; // break threads loops

        stopPdraw(app);
        stopUi(app);
        stopArcommand(app);
        stopArnetwork(app);
        stopArdiscoveryBrowse(app);
        free(app);
    }

    printf("Hasta la vista, PDrAW!\n");
    ULOGI("Finished");

    exit(EXIT_SUCCESS);
}


int startUi(struct pdraw_app *app)
{
    int ret = 0;

#ifdef BUILD_SDL
    ULOGI("Start UI");

    if (ret == 0)
    {
        ret = SDL_Init(SDL_INIT_VIDEO);
        if (ret != 0)
        {
            ULOGE("SDL_Init() failed (%d): %s", ret, SDL_GetError());
            ret = -1;
        }
    }

    if (ret == 0)
    {
        app->sdlFlags = SDL_HWSURFACE | SDL_GL_DOUBLEBUFFER | SDL_OPENGL | SDL_RESIZABLE;

        SDL_GL_SetAttribute(SDL_GL_SWAP_CONTROL, 1);
        SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 2);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

        app->surface = SDL_SetVideoMode(app->windowWidth, app->windowHeight, 0, app->sdlFlags);
        if (app->surface == NULL)
        {
            ULOGE("SDL_SetVideoMode() failed: %s", SDL_GetError());
            ret = -1;
        }
        SDL_WM_SetCaption("PDrAW", "PDrAW");
    }
#endif /* BUILD_SDL */

    return ret;
}


void stopUi(struct pdraw_app *app)
{
#ifdef BUILD_SDL
    SDL_Quit();
#endif /* BUILD_SDL */
}


int startPdraw(struct pdraw_app *app)
{
    int ret = 0;

    ULOGI("Start libpdraw");

    app->pdraw = pdraw_new();
    if (app->pdraw == NULL)
    {
        ULOGE("pdraw_new() failed");
        ret = -1;
    }

    if (ret == 0)
    {
        ret = pdraw_set_self_friendly_name(app->pdraw, "PDrAW-Linux"); //TODO
        if (ret != 0)
        {
            ULOGE("pdraw_set_self_friendly_name() failed (%d)", ret);
        }
    }

    if (ret == 0)
    {
        ret = pdraw_set_self_serial_number(app->pdraw, "00000000"); //TODO
        if (ret != 0)
        {
            ULOGE("pdraw_set_self_serial_number() failed (%d)", ret);
        }
    }

    if (ret == 0)
    {
        ret = pdraw_set_self_software_version(app->pdraw, "PDrAW"); //TODO
        if (ret != 0)
        {
            ULOGE("pdraw_set_self_software_version() failed (%d)", ret);
        }
    }

    if (ret == 0)
    {
        ret = pdraw_set_display_screen_settings(app->pdraw, PDRAW_HMD_DEFAULT_XDPI,
            PDRAW_HMD_DEFAULT_YDPI, PDRAW_HMD_DEFAULT_DEVICE_MARGIN);
        if (ret != 0)
        {
            ULOGE("pdraw_set_display_screen_settings() failed (%d)", ret);
        }
    }

    if (ret == 0)
    {
        ret = pdraw_set_hmd_distorsion_correction_settings(app->pdraw, app->hmdModel,
            PDRAW_HMD_DEFAULT_IPD, PDRAW_HMD_DEFAULT_SCALE, PDRAW_HMD_DEFAULT_PAN_H, PDRAW_HMD_DEFAULT_PAN_V);
        if (ret != 0)
        {
            ULOGE("pdraw_set_hmd_distorsion_correction_settings() failed (%d)", ret);
        }
    }

    if (ret == 0)
    {
        pdraw_get_self_head_orientation_euler(app->pdraw, &app->headOrientation);
    }

    if (ret == 0)
    {
        if ((app->receiveStream) && (strlen(app->url)))
        {
            ret = pdraw_open_url_mcast(app->pdraw, app->url, app->ifaceAddr);
        }
        else if (app->receiveStream)
        {
            ret = pdraw_open_single_stream(app->pdraw, app->ipAddr, app->ifaceAddr, app->srcStreamPort, app->srcControlPort,
                                           app->dstStreamPort, app->dstControlPort, app->qosMode);
        }
        else if (app->playRecord)
        {
            ret = pdraw_open_url(app->pdraw, app->url);
        }
        if (ret != 0)
        {
            ULOGE("pdraw_open() failed (%d)", ret);
        }
    }

    if (ret == 0)
    {
        ret = pdraw_start_renderer(app->pdraw,
                                   app->windowWidth, app->windowHeight, 0, 0,
                                   app->windowWidth, app->windowHeight,
                                   app->hmd, app->headtracking, NULL);
        if (ret < 0)
        {
            ULOGE("pdraw_start_renderer() failed (%d)", ret);
        }
        ret = 0;
    }

    if (ret == 0)
    {
        ret = pdraw_start(app->pdraw);
        if (ret != 0)
        {
            ULOGE("pdraw_start() failed (%d)", ret);
        }
    }

    return ret;
}


void stopPdraw(struct pdraw_app *app)
{
    if (app->pdraw)
    {
        int ret;

        ULOGI("Stop libpdraw");

        ret = pdraw_destroy(app->pdraw);
        if (ret != 0)
        {
            ULOGE("pdraw_destroy() failed (%d)", ret);
        }

        app->pdraw = NULL;
    }
}


int startArdiscoveryBrowse(struct pdraw_app *app)
{
    int failed = 0;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    ULOGI("Start ARDiscovery browsing");

    if (!failed)
    {
        int ret = pthread_mutex_init(&app->ardiscoveryBrowserMutex, NULL);
        if (ret != 0)
        {
            ULOGE("Mutex creation failed (%d)", ret);
            failed = 1;
        }
    }

    if (!failed)
    {
        char serviceTypes[6][128];
        char *serviceTypesPtr[6];
        uint8_t serviceTypesNb = 6, i;

        snprintf(serviceTypes[0], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_ARDRONE));
        snprintf(serviceTypes[1], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_BEBOP_2));
        snprintf(serviceTypes[2], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_EVINRUDE));
        snprintf(serviceTypes[3], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_SKYCONTROLLER));
        snprintf(serviceTypes[4], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG));
        snprintf(serviceTypes[5], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_SKYCONTROLLER_2));
        for (i = 0; i < serviceTypesNb; i++)
        {
            serviceTypesPtr[i] = serviceTypes[i];
        }

        app->ardiscoveryBrowserData = ARDISCOVERY_AvahiDiscovery_Browser_New(ardiscoveryBrowserCallback, (void*)app, serviceTypesPtr, serviceTypesNb, &err);
        if (!app->ardiscoveryBrowserData)
        {
            ULOGE("ARDISCOVERY_AvahiDiscovery_Browser_New() failed: %d", err);
            failed = 1;
        }
    }

    if (!failed)
    {
        if (pthread_create(&(app->ardiscoveryBrowserThread), NULL, (void*(*)(void *))ARDISCOVERY_AvahiDiscovery_Browse, app->ardiscoveryBrowserData) != 0)
        {
            ULOGE("Creation of discovery browser thread failed");
            failed = 1;
        }
        else
        {
            app->ardiscoveryBrowserThreadLaunched = 1;
        }
    }

    return failed;
}


void stopArdiscoveryBrowse(struct pdraw_app *app)
{
    ULOGI("Stop ARDiscovery browsing");

    if (app->ardiscoveryBrowserData != NULL)
    {
        ARDISCOVERY_AvahiDiscovery_StopBrowsing(app->ardiscoveryBrowserData);
        if (app->ardiscoveryBrowserThreadLaunched)
        {
            pthread_join(app->ardiscoveryBrowserThread, NULL);
            app->ardiscoveryBrowserThreadLaunched = 0;
        }
    }

    ARDISCOVERY_AvahiDiscovery_Browser_Delete(&app->ardiscoveryBrowserData);

    struct ardiscovery_browser_device *device, *next;
    for (device = app->ardiscoveryDeviceList; device; device = next)
    {
        next = device->next;
        free(device->serviceName);
        free(device->serviceType);
        free(device->ipAddr);
        free(device);
    }
    app->ardiscoveryDeviceList = NULL;

    pthread_mutex_destroy(&app->ardiscoveryBrowserMutex);
}


eARDISCOVERY_ERROR ardiscoveryBrowserCallback(void *userdata, uint8_t state, const char *serviceName, const char *serviceType, const char *ipAddr, uint16_t port)
{
    struct pdraw_app *app = (struct pdraw_app*)userdata;
    struct ardiscovery_browser_device *device = NULL;

    if (state)
    {
        ULOGI("ARSDK device discovered: %s %s %s %d", serviceName, serviceType, ipAddr, port);
        device = malloc(sizeof(*device));
        if (!device)
        {
            ULOGE("Allocation failed");
            return ARDISCOVERY_ERROR;
        }
        else
        {
            memset(device, 0, sizeof(*device));
            device->serviceName = strdup(serviceName);
            device->serviceType = strdup(serviceType);
            device->ipAddr = strdup(ipAddr);
            device->port = port;
            unsigned int productId = 0;
            sscanf(serviceType, "_arsdk-%04x", &productId);
            device->product = ARDISCOVERY_getProductFromProductID(productId);

            pthread_mutex_lock(&app->ardiscoveryBrowserMutex);

            device->next = app->ardiscoveryDeviceList;
            if (app->ardiscoveryDeviceList)
            {
                app->ardiscoveryDeviceList->prev = device;
            }
            app->ardiscoveryDeviceList = device;
            app->ardiscoveryDeviceCount++;

            pthread_mutex_unlock(&app->ardiscoveryBrowserMutex);
        }
    }
    else
    {
        ULOGI("ARSDK device removed: %s %s %s %d", serviceName, serviceType, ipAddr, port);
        int found = 0;

        pthread_mutex_lock(&app->ardiscoveryBrowserMutex);

        for (device = app->ardiscoveryDeviceList; device; device = device->next)
        {
            if ((device->serviceName) && (!strcmp(device->serviceName, serviceName))
                    && (device->serviceType) && (!strcmp(device->serviceType, serviceType))
                    && (device->ipAddr) && (!strcmp(device->ipAddr, ipAddr))
                    && (device->port == port))
            {
                found = 1;
                break;
            }
        }
        if (found)
        {
            free(device->serviceName);
            free(device->serviceType);
            free(device->ipAddr);
            if (device->prev)
            {
                device->prev->next = device->next;
            }
            else
            {
                app->ardiscoveryDeviceList = device->next;
            }
            if (device->next)
            {
                device->next->prev = device->prev;
            }
            free(device);
            app->ardiscoveryDeviceCount--;
        }

        pthread_mutex_unlock(&app->ardiscoveryBrowserMutex);
    }

    if (!strlen(app->ipAddr))
    {
        pthread_mutex_lock(&app->ardiscoveryBrowserMutex);

        printf("\nDevice discovery:\n");
        int i;
        for (device = app->ardiscoveryDeviceList, i = 1; device; device = device->next, i++)
        {
            printf("  - %d - %s (%s) %s:%d\n", i, device->serviceName, device->serviceType, device->ipAddr, device->port);
        }

        if (app->ardiscoveryDeviceCount == 0)
        {
            printf("  - none\n");
        }

        pthread_mutex_unlock(&app->ardiscoveryBrowserMutex);
    }

    return ARDISCOVERY_OK;
}


int ardiscoveryConnect(struct pdraw_app *app)
{
    int failed = 0;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    ARDISCOVERY_Connection_ConnectionData_t *discoveryData;

    ULOGI("ARDiscovery connection");

    discoveryData = ARDISCOVERY_Connection_New(ardiscoveryConnectionSendJsonCallback,
                                               ardiscoveryConnectionReceiveJsonCallback, app, &err);
    if (discoveryData == NULL || err != ARDISCOVERY_OK)
    {
        ULOGE("Error while creating discoveryData: %s", ARDISCOVERY_Error_ToString(err));
        failed = 1;
    }

    if (!failed)
    {
        err = ARDISCOVERY_Connection_ControllerConnection(discoveryData, app->arsdkDiscoveryPort, app->ipAddr);
        if (err != ARDISCOVERY_OK)
        {
            ULOGE("Error while opening discovery connection: %s", ARDISCOVERY_Error_ToString(err));
            failed = 1;
        }
    }

    ARDISCOVERY_Connection_Delete(&discoveryData);

    return failed;
}


eARDISCOVERY_ERROR ardiscoveryConnectionSendJsonCallback(uint8_t *dataTx, uint32_t *dataTxSize, void *customData)
{
    struct pdraw_app *app = (struct pdraw_app*)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataTx != NULL) && (dataTxSize != NULL) && (app != NULL))
    {
        *dataTxSize = sprintf((char*)dataTx, "{ \"%s\": %d, \"%s\": \"%s\", \"%s\": \"%s\", \"%s\": %d, \"%s\": %d, \"%s\": %d, \"%s\": %d }",
                              ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY, app->arsdkD2CPort,
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, "PDrAW",
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, "Unix",
                              ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY, 1,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_STREAM_PORT_KEY, app->dstStreamPort,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_CONTROL_PORT_KEY, app->dstControlPort,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SUPPORTED_METADATA_VERSION_KEY, 1) + 1;
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}


eARDISCOVERY_ERROR ardiscoveryConnectionReceiveJsonCallback(uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData)
{
    struct pdraw_app *app = (struct pdraw_app*)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataRx != NULL) && (dataRxSize != 0) && (app != NULL))
    {
        char *json = (char*)malloc(dataRxSize + 1);
        strncpy(json, (char*)dataRx, dataRxSize);
        json[dataRxSize] = '\0';

        ULOGD("Received JSON: %s", json);

        free(json);

        int error = 0;
        json_object* jsonObj_All;
        json_object* jsonObj_Item;
        json_bool jsonRet;

        /* Parse the whole Rx buffer */
        if (error == 0)
        {
            jsonObj_All = json_tokener_parse((const char*)dataRx);
            if (jsonObj_All == NULL)
                error = -1;
        }

        /* Find the c2dPort */
        if (error == 0)
        {
            jsonRet = json_object_object_get_ex(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_C2DPORT_KEY, &jsonObj_Item);
            if ((jsonRet) && (jsonObj_Item != NULL))
                app->arsdkC2DPort = json_object_get_int(jsonObj_Item);
        }

        /* Find the QoS mode */
        if (error == 0)
        {
            jsonRet = json_object_object_get_ex(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY, &jsonObj_Item);
            if ((jsonRet) && (jsonObj_Item != NULL))
                app->qosMode = json_object_get_int(jsonObj_Item);
        }

        /* Find the srcStreamPort */
        if (error == 0)
        {
            jsonRet = json_object_object_get_ex(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_STREAM_PORT_KEY, &jsonObj_Item);
            if ((jsonRet) && (jsonObj_Item != NULL))
                app->srcStreamPort = json_object_get_int(jsonObj_Item);
        }

        /* Find the srcControlPort */
        if (error == 0)
        {
            jsonRet = json_object_object_get_ex(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_CONTROL_PORT_KEY, &jsonObj_Item);
            if ((jsonRet) && (jsonObj_Item != NULL))
                app->srcControlPort = json_object_get_int(jsonObj_Item);
        }
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}


int startArnetwork(struct pdraw_app *app)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ULOGI("Start ARNetwork");

    app->arnetworkalManager = ARNETWORKAL_Manager_New(&netAlError);
    if (netAlError != ARNETWORKAL_OK)
    {
        failed = 1;
    }

    if (!failed)
    {
        netAlError = ARNETWORKAL_Manager_InitWifiNetwork(app->arnetworkalManager, app->ipAddr, app->arsdkC2DPort, app->arsdkD2CPort, 1);
        if (netAlError != ARNETWORKAL_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        if (app->qosMode == 1)
        {
            netAlError = ARNETWORKAL_Manager_SetSendClassSelector(app->arnetworkalManager, ARSAL_SOCKET_CLASS_SELECTOR_CS6);
            if (netAlError != ARNETWORKAL_OK)
            {
                failed = 1;
            }
            netAlError = ARNETWORKAL_Manager_SetRecvClassSelector(app->arnetworkalManager, ARSAL_SOCKET_CLASS_SELECTOR_CS6);
            if (netAlError != ARNETWORKAL_OK)
            {
                failed = 1;
            }
        }
    }

    if (!failed)
    {
        app->arnetworkManager = ARNETWORK_Manager_New(app->arnetworkalManager, c2dParamsCount, c2dParams, d2cParamsCount, d2cParams, pingDelay, arnetworkOnDisconnectCallback, app, &netError);
        if (netError != ARNETWORK_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        if (pthread_create(&(app->arnetworkRxThread), NULL, ARNETWORK_Manager_ReceivingThreadRun, app->arnetworkManager) != 0)
        {
            ULOGE("Creation of rx thread failed");
            failed = 1;
        }
        else
        {
            app->arnetworkRxThreadLaunched = 1;
        }

        if (pthread_create(&(app->arnetworkTxThread), NULL, ARNETWORK_Manager_SendingThreadRun, app->arnetworkManager) != 0)
        {
            ULOGE("Creation of tx thread failed");
            failed = 1;
        }
        else
        {
            app->arnetworkTxThreadLaunched = 1;
        }
    }

    if (failed)
    {
        if (netAlError != ARNETWORKAL_OK)
        {
            ULOGE("ARNetWorkAL Error: %s", ARNETWORKAL_Error_ToString(netAlError));
        }

        if (netError != ARNETWORK_OK)
        {
            ULOGE("ARNetWork Error: %s", ARNETWORK_Error_ToString(netError));
        }
    }

    return failed;
}


void stopArnetwork(struct pdraw_app *app)
{
    ULOGI("Stop ARNetwork");

    if (app->arnetworkManager != NULL)
    {
        ARNETWORK_Manager_Stop(app->arnetworkManager);
        if (app->arnetworkRxThreadLaunched)
        {
            pthread_join(app->arnetworkRxThread, NULL);
            app->arnetworkRxThreadLaunched = 0;
        }

        if (app->arnetworkTxThreadLaunched)
        {
            pthread_join(app->arnetworkTxThread, NULL);
            app->arnetworkTxThreadLaunched = 0;
        }
    }

    if (app->arnetworkalManager != NULL)
    {
        ARNETWORKAL_Manager_Unlock(app->arnetworkalManager);

        ARNETWORKAL_Manager_CloseWifiNetwork(app->arnetworkalManager);
    }

    ARNETWORK_Manager_Delete(&(app->arnetworkManager));
    ARNETWORKAL_Manager_Delete(&(app->arnetworkalManager));
}


void arnetworkOnDisconnectCallback(ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData)
{
    ULOGD("ARNetwork disconnection callback");

    struct pdraw_app *app = (struct pdraw_app*)customData;

    if (!app)
    {
        return;
    }

    app->disconnected = 1;
}


eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause)
{
    eARNETWORK_MANAGER_CALLBACK_RETURN retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;

    ULOGD("ARNetwork command callback %d, cause:%d ", buffer_id, cause);

    if (cause == ARNETWORK_MANAGER_CALLBACK_STATUS_TIMEOUT)
    {
        retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DATA_POP;
    }

    return retval;
}


void *arnetworkCmdReaderRun(void* data)
{
    struct pdraw_app *app = NULL;
    int bufferId = 0;
    int failed = 0;
    
    const size_t maxLength = 128 * 1024;
    void *readData = malloc(maxLength);
    if (readData == NULL)
    {
        failed = 1;
    }
    
    if (!failed)
    {
        if (data != NULL)
        {
            bufferId = ((struct arcmd_reader_data*)data)->readerBufferId;
            app = ((struct arcmd_reader_data*)data)->app;
            
            if (app == NULL)
            {
                failed = 1;
            }
        }
        else
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        while (app->run)
        {
            eARNETWORK_ERROR netError = ARNETWORK_OK;
            int length = 0;
            int skip = 0;
            
            netError = ARNETWORK_Manager_ReadDataWithTimeout(app->arnetworkManager, bufferId, (uint8_t*)readData, maxLength, &length, 1000);
            if (netError != ARNETWORK_OK)
            {
                if (netError != ARNETWORK_ERROR_BUFFER_EMPTY)
                {
                    ULOGE("ARNETWORK_Manager_ReadDataWithTimeout () failed: %s", ARNETWORK_Error_ToString(netError));
                }
                skip = 1;
            }
            
            if (!skip)
            {
                eARCOMMANDS_DECODER_ERROR cmdError = ARCOMMANDS_DECODER_OK;
                cmdError = ARCOMMANDS_Decoder_DecodeCommand(app->arcmdDecoder, (uint8_t*)readData, length);
                if ((cmdError != ARCOMMANDS_DECODER_OK) && (cmdError != ARCOMMANDS_DECODER_ERROR_NO_CALLBACK))
                {
                    char msg[128];
                    ARCOMMANDS_Decoder_DescribeBuffer((uint8_t*)readData, length, msg, sizeof(msg));
                    ULOGE("ARCOMMANDS_Decoder_DecodeBuffer () failed: %d %s", cmdError, msg);
                }
            }
        }
    }
    
    if (readData != NULL)
    {
        free(readData);
        readData = NULL;
    }
    
    return NULL;
}


int startArcommand(struct pdraw_app *app)
{
    int failed = 0;

    app->arcmdDecoder = ARCOMMANDS_Decoder_NewDecoder(NULL);
    if (app->arcmdDecoder == NULL)
    {
        ULOGE("Failed to create decoder");
        failed = 1;
    }

    app->arcmdReaderThreads = (pthread_t*)calloc(commandBufferIdsCount, sizeof(pthread_t));
    if (app->arcmdReaderThreads == NULL)
    {
        ULOGE("Allocation of reader threads failed");
        failed = 1;
    }

    app->arcmdReaderThreadsLaunched = (int*)calloc(commandBufferIdsCount, sizeof(int));
    if (app->arcmdReaderThreadsLaunched == NULL)
    {
        ULOGE("Allocation of reader threads failed");
        failed = 1;
    }

    if (!failed)
    {
        app->arcmdThreadData = (struct arcmd_reader_data*)calloc(commandBufferIdsCount, sizeof(struct arcmd_reader_data));
        if (app->arcmdThreadData == NULL)
        {
            ULOGE("Allocation of reader threads data failed");
            failed = 1;
        }
    }

    if (!failed)
    {
        size_t readerThreadIndex = 0;
        for (readerThreadIndex = 0 ; readerThreadIndex < commandBufferIdsCount ; readerThreadIndex++)
        {
            app->arcmdThreadData[readerThreadIndex].app = app;
            app->arcmdThreadData[readerThreadIndex].readerBufferId = commandBufferIds[readerThreadIndex];

            if (pthread_create(&(app->arcmdReaderThreads[readerThreadIndex]), NULL, arnetworkCmdReaderRun, &(app->arcmdThreadData[readerThreadIndex])) != 0)
            {
                ULOGE("Creation of reader thread failed");
                failed = 1;
            }
            else
            {
                app->arcmdReaderThreadsLaunched[readerThreadIndex] = 1;
            }
        }
    }

    if (!failed)
    {
        if ((app->arsdkProduct == ARDISCOVERY_PRODUCT_SKYCONTROLLER) ||
            (app->arsdkProduct == ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG) ||
            (app->arsdkProduct == ARDISCOVERY_PRODUCT_SKYCONTROLLER_2))
        {
            ARCOMMANDS_Decoder_SetSkyControllerSkyControllerStateBatteryChangedCb(app->arcmdDecoder,
                skyControllerSkyControllerStateBatteryChangedCallback, (void*)app);
            ARCOMMANDS_Decoder_SetSkyControllerSkyControllerStateGpsPositionChangedCb(app->arcmdDecoder,
                skyControllerSkyControllerStateGpsPositionChangedCallback, (void*)app);
            ARCOMMANDS_Decoder_SetSkyControllerSkyControllerStateAttitudeChangedCb(app->arcmdDecoder,
                skyControllerSkyControllerStateAttitudeChangedCallback, (void*)app);
        }
    }

    return failed;
}


void stopArcommand(struct pdraw_app *app)
{
    if ((app->arcmdReaderThreads != NULL) && (app->arcmdReaderThreadsLaunched != NULL))
    {
        size_t readerThreadIndex = 0;
        for (readerThreadIndex = 0 ; readerThreadIndex < commandBufferIdsCount ; readerThreadIndex++)
        {
            if (app->arcmdReaderThreadsLaunched[readerThreadIndex])
            {
                pthread_join(app->arcmdReaderThreads[readerThreadIndex], NULL);
                app->arcmdReaderThreadsLaunched[readerThreadIndex] = 0;
            }
        }
        
        free(app->arcmdReaderThreads);
        app->arcmdReaderThreads = NULL;
        free(app->arcmdReaderThreadsLaunched);
        app->arcmdReaderThreadsLaunched = NULL;
    }
    
    if (app->arcmdThreadData != NULL)
    {
        free(app->arcmdThreadData);
        app->arcmdThreadData = NULL;
    }

    ARCOMMANDS_Decoder_DeleteDecoder(&app->arcmdDecoder);
}


int sendDateAndTime(struct pdraw_app *app)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ULOGD("Send date and time commands");

    char strDate[30];
    char strTime[30];
    time_t rawDate;
    struct tm* tmDateTime;

    time(&rawDate);
    tmDateTime = localtime(&rawDate);
    strftime(strDate, 30, "%F", tmDateTime);
    strftime(strTime, 30, "T%H%M%S%z", tmDateTime);

    /* Send date command */
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentDate(cmdBuffer, sizeof(cmdBuffer), &cmdSize, strDate);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(app->arnetworkManager, PDRAW_ARSDK_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ULOGW("Failed to send date command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    /* Send time command */
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentTime(cmdBuffer, sizeof(cmdBuffer), &cmdSize, strTime);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(app->arnetworkManager, PDRAW_ARSDK_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ULOGW("Failed to send time command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}


int sendAllStates(struct pdraw_app *app)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ULOGD("Send all states command");

    /* Send all states command */
    if ((app->arsdkProduct == ARDISCOVERY_PRODUCT_SKYCONTROLLER) ||
        (app->arsdkProduct == ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG) ||
        (app->arsdkProduct == ARDISCOVERY_PRODUCT_SKYCONTROLLER_2))
    {
        cmdError = ARCOMMANDS_Generator_GenerateSkyControllerCommonAllStates(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    }
    else
    {
        cmdError = ARCOMMANDS_Generator_GenerateCommonCommonAllStates(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    }
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(app->arnetworkManager, PDRAW_ARSDK_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ULOGW("Failed to send all states command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}


int sendStreamingVideoEnable(struct pdraw_app *app)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ULOGD("Send streaming video enable command");
    
    /* Send streaming begin command */
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(cmdBuffer, sizeof(cmdBuffer), &cmdSize, 1);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(app->arnetworkManager, PDRAW_ARSDK_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ULOGW("Failed to send streaming video enable command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}


int sendCameraOrientation(struct pdraw_app *app, float pan, float tilt)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    /* Send camera orientation command */
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3CameraOrientationV2(cmdBuffer, sizeof(cmdBuffer), &cmdSize, tilt, pan);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(app->arnetworkManager, PDRAW_ARSDK_CD_NONACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ULOGW("Failed to send camera orientation command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}


void skyControllerSkyControllerStateBatteryChangedCallback(uint8_t percent, void *custom)
{
    struct pdraw_app *app = (struct pdraw_app*)custom;
    if (app)
    {
        int ret = pdraw_set_self_controller_battery_level(app->pdraw, percent);
        if (ret != 0)
        {
            ULOGE("pdraw_set_self_controller_battery_level() failed (%d)", ret);
        }
    }
}


void skyControllerSkyControllerStateGpsPositionChangedCallback(double latitude, double longitude, double altitude, float heading, void *custom)
{
    struct pdraw_app *app = (struct pdraw_app*)custom;
    if (app)
    {
        pdraw_location_t loc;
        loc.isValid = ((latitude != 500.) && (longitude != 500.)) ? 1 : 0;
        loc.latitude = latitude;
        loc.longitude = longitude;
        loc.altitude = altitude;
        loc.svCount = 0;
        int ret = pdraw_set_self_location(app->pdraw, &loc);
        if (ret != 0)
        {
            ULOGE("pdraw_set_self_location() failed (%d)", ret);
        }
    }
}


void skyControllerSkyControllerStateAttitudeChangedCallback(float q0, float q1, float q2, float q3, void *custom)
{
    struct pdraw_app *app = (struct pdraw_app*)custom;
    if (app)
    {
        pdraw_quaternion_t quat;
        quat.w = q0;
        quat.x = q1;
        quat.y = q2;
        quat.z = q3;
        int ret = pdraw_set_self_controller_orientation_quat(app->pdraw, &quat);
        if (ret != 0)
        {
            ULOGE("pdraw_set_self_controller_orientation_quat() failed (%d)", ret);
        }
    }
}


int skyControllerRestreamConnect(struct pdraw_app *app)
{
    int failed = 0;
    char url[100];
    snprintf(url, sizeof(url), "http://%s:7711/video", app->ipAddr);

    ULOGI("SkyController restream connection: %s", url);

    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_DEFAULT);

    curl = curl_easy_init();
    if (curl)
    {
        curl_easy_setopt(curl, CURLOPT_URL, url);
        FILE *devnull = fopen("/dev/null", "w+");
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, devnull);

        res = curl_easy_perform(curl);
        if (res != CURLE_OK)
        {
            ULOGE("curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
            failed = 1;
        }

        curl_easy_cleanup(curl);
        fclose(devnull);
    }

    curl_global_cleanup();

    app->srcStreamPort = 5004;
    app->srcControlPort = 5005;
    app->dstStreamPort = 55004;
    app->dstControlPort = 55005;

    return failed;
}
