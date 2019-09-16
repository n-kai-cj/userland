/*
  Copyright (c) 2018, Raspberry Pi (Trading) Ltd.
  Copyright (c) 2013, Broadcom Europe Ltd.
  Copyright (c) 2013, James Hughes
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  * Neither the name of the copyright holder nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// We use some GNU extensions (basename)
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <sysexits.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
#include "RaspiHelpers.h"
//#include "RaspiGPS.h"

//#include <semaphore.h>

#include <stdbool.h>

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Port configuration for the splitter component
#define SPLITTER_OUTPUT_PORT 0
#define SPLITTER_PREVIEW_PORT 1

// Video format information
// 0 implies variable
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

/// Capture/Pause switch method
/// Simply capture for time specified
enum
{
  WAIT_METHOD_NONE,     /// Simply capture for time specified
  WAIT_METHOD_TIMED,    /// Cycle between capture and pause for times specified
  WAIT_METHOD_KEYPRESS, /// Switch between capture and pause on keypress
  WAIT_METHOD_SIGNAL,   /// Switch between capture and pause on signal
  WAIT_METHOD_FOREVER   /// Run/record forever
};

// Forward
typedef struct RASPIVID_STATE_S RASPIVID_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
  FILE *file_handle;      /// File handle to write buffer data to.
  RASPIVID_STATE *pstate; /// pointer to our state in case required in callback
  int abort;              /// Set to 1 in callback if an error occurs to attempt to abort the capture
  char *cb_buff;          /// Circular buffer
  int cb_len;             /// Length of buffer
  int cb_wptr;            /// Current write pointer
  int cb_wrap;            /// Has buffer wrapped at least once?
  int cb_data;            /// Valid bytes in buffer
#define IFRAME_BUFSIZE (60 * 1000)
  int iframe_buff[IFRAME_BUFSIZE]; /// buffer of iframe pointers
  int iframe_buff_wpos;
  int iframe_buff_rpos;
  char header_bytes[29];
  int header_wptr;
  FILE *imv_file_handle; /// File handle to write inline motion vectors to.
  FILE *raw_file_handle; /// File handle to write raw data to.
  int flush_buffers;
  FILE *pts_file_handle; /// File timestamps
} PORT_USERDATA;

/** Possible raw output formats
 */
typedef enum
{
  RAW_OUTPUT_FMT_YUV = 0,
  RAW_OUTPUT_FMT_RGB,
  RAW_OUTPUT_FMT_GRAY,
} RAW_OUTPUT_FMT;

/** Structure containing all state information for the current run
 */
struct RASPIVID_STATE_S
{
  RASPICOMMONSETTINGS_PARAMETERS common_settings; /// Common settings
  int timeout;                                    /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
  MMAL_FOURCC_T encoding;                         /// Requested codec video encoding (MJPEG or H264)
  int bitrate;                                    /// Requested bitrate
  int framerate;                                  /// Requested frame rate (fps)
  int intraperiod;                                /// Intra-refresh period (key frame rate)
  int quantisationParameter;                      /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
  int bInlineHeaders;                             /// Insert inline headers to stream (SPS, PPS)
  int demoMode;                                   /// Run app in demo mode
  int demoInterval;                               /// Interval between camera settings changes
  int immutableInput;                             /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
  /// the camera output or the encoder output (with compression artifacts)
  int profile;    /// H264 profile to use for encoding
  int level;      /// H264 level to use for encoding
  int waitMethod; /// Method for switching between pause and capture

  int onTime;  /// In timed cycle mode, the amount of time the capture is on per cycle
  int offTime; /// In timed cycle mode, the amount of time the capture is off per cycle

  int segmentSize;   /// Segment mode In timed cycle mode, the amount of time the capture is off per cycle
  int segmentWrap;   /// Point at which to wrap segment counter
  int segmentNumber; /// Current segment counter
  int splitNow;      /// Split at next possible i-frame if set to 1.
  int splitWait;     /// Switch if user wants splited files

  RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
  RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

  MMAL_COMPONENT_T *camera_component;     /// Pointer to the camera component
  MMAL_COMPONENT_T *splitter_component;   /// Pointer to the splitter component
  MMAL_COMPONENT_T *encoder_component;    /// Pointer to the encoder component
  MMAL_CONNECTION_T *preview_connection;  /// Pointer to the connection from camera or splitter to preview
  MMAL_CONNECTION_T *splitter_connection; /// Pointer to the connection from camera to splitter
  MMAL_CONNECTION_T *encoder_connection;  /// Pointer to the connection from camera to encoder

  MMAL_POOL_T *splitter_pool; /// Pointer to the pool of buffers used by splitter output port 0
  MMAL_POOL_T *encoder_pool;  /// Pointer to the pool of buffers used by encoder output port

  PORT_USERDATA callback_data; /// Used to move data to the encoder callback

  int bCapturing;      /// State of capture/pause
  int bCircularBuffer; /// Whether we are writing to a circular buffer

  int inlineMotionVectors;       /// Encoder outputs inline Motion Vectors
  char *imv_filename;            /// filename of inline Motion Vectors output
  int raw_output;                /// Output raw video from camera as well
  RAW_OUTPUT_FMT raw_output_fmt; /// The raw video format
  char *raw_filename;            /// Filename for raw video output
  int intra_refresh_type;        /// What intra refresh type to use. -1 to not set.
  int frame;
  char *pts_filename;
  int save_pts;
  int64_t starttime;
  int64_t lasttime;

  bool netListen;
  MMAL_BOOL_T addSPSTiming;
  int slices;

  int width, height;
};

/// Structure to cross reference H264 profile strings against the MMAL parameter equivalent
static XREF_T profile_map[] =
    {
        {"baseline", MMAL_VIDEO_PROFILE_H264_BASELINE},
        {"main", MMAL_VIDEO_PROFILE_H264_MAIN},
        {"high", MMAL_VIDEO_PROFILE_H264_HIGH},
        //   {"constrained",  MMAL_VIDEO_PROFILE_H264_CONSTRAINED_BASELINE} // Does anyone need this?
};

static int profile_map_size = sizeof(profile_map) / sizeof(profile_map[0]);

/// Structure to cross reference H264 level strings against the MMAL parameter equivalent
static XREF_T level_map[] =
    {
        {"4", MMAL_VIDEO_LEVEL_H264_4},
        {"4.1", MMAL_VIDEO_LEVEL_H264_41},
        {"4.2", MMAL_VIDEO_LEVEL_H264_42},
};

static int level_map_size = sizeof(level_map) / sizeof(level_map[0]);

static XREF_T initial_map[] =
    {
        {"record", 0},
        {"pause", 1},
};

static int initial_map_size = sizeof(initial_map) / sizeof(initial_map[0]);

static XREF_T intra_refresh_map[] =
    {
        {"cyclic", MMAL_VIDEO_INTRA_REFRESH_CYCLIC},
        {"adaptive", MMAL_VIDEO_INTRA_REFRESH_ADAPTIVE},
        {"both", MMAL_VIDEO_INTRA_REFRESH_BOTH},
        {"cyclicrows", MMAL_VIDEO_INTRA_REFRESH_CYCLIC_MROWS},
        //   {"random",       MMAL_VIDEO_INTRA_REFRESH_PSEUDO_RAND} Cannot use random, crashes the encoder. No idea why.
};

static int intra_refresh_map_size = sizeof(intra_refresh_map) / sizeof(intra_refresh_map[0]);

static XREF_T raw_output_fmt_map[] =
    {
        {"yuv", RAW_OUTPUT_FMT_YUV},
        {"rgb", RAW_OUTPUT_FMT_RGB},
        {"gray", RAW_OUTPUT_FMT_GRAY},
};

static int raw_output_fmt_map_size = sizeof(raw_output_fmt_map) / sizeof(raw_output_fmt_map[0]);

MMAL_PORT_T *video_port = NULL;
MMAL_PORT_T *encoder_output = NULL;
void (*mmal_encoded_frame_callback)();

#define ENCODED_FRAME_MAX_SIZE 1048576 // 1MB

// encoded frame buffer
char encoded_frame_buffer[ENCODED_FRAME_MAX_SIZE];

// encoded frame buffer size
int encoded_frame_size = 0;
int64_t encoded_frame_pts = 0;
int64_t encoded_frame_interval = 0;

// main run thread
pthread_t thread;

// loop flag
bool loop_flag = false;

// resolution
int width = 1920;
int height = 1080;

// bitrate [Kbps]
int bitratekbps = 10000;

// framerate [fps]
int frameratefps = 30;

// gop size. framerateFps*10 is recommended
int intra_period = 150;

// video profile
int profile = MMAL_VIDEO_PROFILE_H264_MAIN;
