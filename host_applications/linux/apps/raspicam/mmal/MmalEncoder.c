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

#include "MmalEncoder.h"
#include "MmalEncoderInterface.h"

/**************************************************
 ****************** RaspiVid.c ********************
 **************************************************/
/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(RASPIVID_STATE *state)
{
  if (!state)
  {
    vcos_assert(0);
    return;
  }

  // Default everything to zero
  memset(state, 0, sizeof(RASPIVID_STATE));

  raspicommonsettings_set_defaults(&state->common_settings);

  // Now set anything non-zero
  state->timeout = -1;                 // replaced with 5000ms later if unset
  state->common_settings.width = 1920; // Default to 1080p
  state->common_settings.height = 1080;
  state->encoding = MMAL_ENCODING_H264;
  state->bitrate = 17000000; // This is a decent default bitrate for 1080p
  state->framerate = VIDEO_FRAME_RATE_NUM;
  state->intraperiod = -1; // Not set
  state->quantisationParameter = 0;
  state->demoMode = 0;
  state->demoInterval = 250; // ms
  state->immutableInput = 1;
  state->profile = MMAL_VIDEO_PROFILE_H264_HIGH;
  state->level = MMAL_VIDEO_LEVEL_H264_4;
  state->waitMethod = WAIT_METHOD_NONE;
  state->onTime = 5000;
  state->offTime = 5000;
  state->bCapturing = 0;
  state->bInlineHeaders = 0;
  state->segmentSize = 0; // 0 = not segmenting the file.
  state->segmentNumber = 1;
  state->segmentWrap = 0; // Point at which to wrap segment number back to 1. 0 = no wrap
  state->splitNow = 0;
  state->splitWait = 0;
  state->inlineMotionVectors = 0;
  state->intra_refresh_type = -1;
  state->frame = 0;
  state->save_pts = 0;
  state->netListen = false;
  state->addSPSTiming = MMAL_FALSE;
  state->slices = 1;

  // Set up the camera_parameters to default
  raspicamcontrol_set_defaults(&state->camera_parameters);
}

static void check_camera_model(int cam_num)
{
  MMAL_COMPONENT_T *camera_info;
  MMAL_STATUS_T status;

  // Try to get the camera name
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
  if (status == MMAL_SUCCESS)
  {
    MMAL_PARAMETER_CAMERA_INFO_T param;
    param.hdr.id = MMAL_PARAMETER_CAMERA_INFO;
    param.hdr.size = sizeof(param) - 4; // Deliberately undersize to check firmware version
    status = mmal_port_parameter_get(camera_info->control, &param.hdr);

    if (status != MMAL_SUCCESS)
    {
      // Running on newer firmware
      param.hdr.size = sizeof(param);
      status = mmal_port_parameter_get(camera_info->control, &param.hdr);
      if (status == MMAL_SUCCESS && param.num_cameras > cam_num)
      {
        if (!strncmp(param.cameras[cam_num].camera_name, "toshh2c", 7))
        {
          fprintf(stderr, "The driver for the TC358743 HDMI to CSI2 chip you are using is NOT supported.\n");
          fprintf(stderr, "They were written for a demo purposes only, and are in the firmware on an as-is\n");
          fprintf(stderr, "basis and therefore requests for support or changes will not be acted on.\n\n");
        }
      }
    }

    mmal_component_destroy(camera_info);
  }
}

/**
 * Update any annotation data specific to the video.
 * This simply passes on the setting from cli, or
 * if application defined annotate requested, updates
 * with the H264 parameters
 *
 * @param state Pointer to state control struct
 *
 */
static void update_annotation_data(RASPIVID_STATE *state)
{
  // So, if we have asked for a application supplied string, set it to the H264 or GPS parameters
  if (state->camera_parameters.enable_annotate & ANNOTATE_APP_TEXT)
  {
    char *text;

    if (state->common_settings.gps)
    {
      /* text = raspi_gps_location_string(); */
    }
    else
    {
      const char *refresh = raspicli_unmap_xref(state->intra_refresh_type, intra_refresh_map, intra_refresh_map_size);

      asprintf(&text, "%dk,%df,%s,%d,%s,%s",
               state->bitrate / 1000, state->framerate,
               refresh ? refresh : "(none)",
               state->intraperiod,
               raspicli_unmap_xref(state->profile, profile_map, profile_map_size),
               raspicli_unmap_xref(state->level, level_map, level_map_size));
    }

    raspicamcontrol_set_annotate(state->camera_component, state->camera_parameters.enable_annotate, text,
                                 state->camera_parameters.annotate_text_size,
                                 state->camera_parameters.annotate_text_colour,
                                 state->camera_parameters.annotate_bg_colour,
                                 state->camera_parameters.annotate_justify,
                                 state->camera_parameters.annotate_x,
                                 state->camera_parameters.annotate_y);

    free(text);
  }
  else
  {
    raspicamcontrol_set_annotate(state->camera_component, state->camera_parameters.enable_annotate, state->camera_parameters.annotate_string,
                                 state->camera_parameters.annotate_text_size,
                                 state->camera_parameters.annotate_text_colour,
                                 state->camera_parameters.annotate_bg_colour,
                                 state->camera_parameters.annotate_justify,
                                 state->camera_parameters.annotate_x,
                                 state->camera_parameters.annotate_y);
  }
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
  MMAL_BUFFER_HEADER_T *new_buffer;
  static int64_t base_time = -1;
  static int64_t last_second = -1;

  // All our segment times based on the receipt of the first encoder callback
  if (base_time == -1)
    base_time = get_microseconds64() / 1000;

  // We pass our file handle and other stuff in via the userdata field.

  PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

  if (pData)
  {
    int bytes_written = buffer->length;
    int64_t current_time = get_microseconds64() / 1000;

    if (pData->pstate->inlineMotionVectors)
      vcos_assert(pData->imv_file_handle);

    // For segmented record mode, we need to see if we have exceeded our time/size,
    // but also since we have inline headers turned on we need to break when we get one to
    // ensure that the new stream has the header in it. If we break on an I-frame, the
    // SPS/PPS header is actually in the previous chunk.
    if (buffer->length)
    {
      mmal_buffer_header_mem_lock(buffer);
      // NOT frame end
      if ((buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END) == 0)
      {
        if ((buffer->length + encoded_frame_size) > ENCODED_FRAME_MAX_SIZE)
        {
          // buffer exceeds
          fprintf(stderr, "error: encoded frame buffer exceeds\n");
          memcpy(&encoded_frame_buffer[encoded_frame_size], buffer->data, ENCODED_FRAME_MAX_SIZE - encoded_frame_size);
          encoded_frame_size = ENCODED_FRAME_MAX_SIZE;
        }
        else
        {
          // fill encoded_frame_buffer until end
          memcpy(&encoded_frame_buffer[encoded_frame_size], buffer->data, buffer->length);
          encoded_frame_size += buffer->length;
          if ((buffer->pts != 0) && ((buffer->pts & ((int64_t)1 << 63)) == 0))
          {
            encoded_frame_interval = buffer->pts - encoded_frame_pts;
            encoded_frame_pts = buffer->pts;
          }
        }
      }

      // frame end
      else
      {
        // some data in encoded_frame_buffer left
        if (encoded_frame_size > 0)
        {
          if (buffer->length + encoded_frame_size > ENCODED_FRAME_MAX_SIZE)
          {
            // buffer exceeds
            fprintf(stderr, "error: encoded frame buffer exceeds\n");
            memcpy(&encoded_frame_buffer[encoded_frame_size], buffer->data, ENCODED_FRAME_MAX_SIZE - encoded_frame_size);
            encoded_frame_size = ENCODED_FRAME_MAX_SIZE;
          }
          else
          {
            // fill encoded_frame_buffer until end
            memcpy(&encoded_frame_buffer[encoded_frame_size], buffer->data, buffer->length);
            encoded_frame_size += buffer->length;

            if ((buffer->pts != 0) && ((buffer->pts & ((int64_t)1 << 63)) == 0))
            {
              encoded_frame_interval = buffer->pts - encoded_frame_pts;
              encoded_frame_pts = buffer->pts;
            }
          }

          // callback
          (*mmal_encoded_frame_callback)(encoded_frame_buffer, encoded_frame_size, buffer->pts, current_time);

          // reset encoded_frame_buffer pointer
          encoded_frame_size = 0;
        }
        else
        {
          if ((buffer->pts != 0) && ((buffer->pts & ((int64_t)1 << 63)) == 0))
          {
            encoded_frame_interval = buffer->pts - encoded_frame_pts;
            encoded_frame_pts = buffer->pts;
          }

          // callback
          (*mmal_encoded_frame_callback)(buffer->data, buffer->length, buffer->pts, current_time);

          // reset encoded_frame_buffer pointer
          encoded_frame_size = 0;
        }
      }

      if (pData->pstate->save_pts &&
          !(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CONFIG) &&
          buffer->pts != MMAL_TIME_UNKNOWN &&
          buffer->pts != pData->pstate->lasttime)
      {
        int64_t pts;
        if (pData->pstate->frame == 0)
          pData->pstate->starttime = buffer->pts;
        pData->pstate->lasttime = buffer->pts;
        pts = buffer->pts - pData->pstate->starttime;
        fprintf(pData->pts_file_handle, "%lld.%03lld\n", pts / 1000, pts % 1000);
        pData->pstate->frame++;
      }

      mmal_buffer_header_mem_unlock(buffer);

      if (bytes_written != buffer->length)
      {
        fprintf(stderr, "error: Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
        vcos_log_error("Failed to write buffer data (%d from %d)- aborting", bytes_written, buffer->length);
        pData->abort = 1;
      }
    }

    // See if the second count has changed and we need to update any annotation
    if (current_time / 1000 != last_second)
    {
      update_annotation_data(pData->pstate);
      last_second = current_time / 1000;
    }
  }
  else
  {
    vcos_log_error("Received a encoder buffer callback with no state");
  }

  // release buffer back to the pool
  mmal_buffer_header_release(buffer);

  // and send one back to the port (if still open)
  if (port->is_enabled)
  {
    MMAL_STATUS_T status;

    new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

    if (new_buffer)
      status = mmal_port_send_buffer(port, new_buffer);

    if (!new_buffer || status != MMAL_SUCCESS)
      vcos_log_error("Unable to return a buffer to the encoder port");
  }
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_camera_component(RASPIVID_STATE *state)
{
  MMAL_COMPONENT_T *camera = 0;
  MMAL_ES_FORMAT_T *format;
  MMAL_PORT_T *preview_port = NULL, *still_port = NULL;
  MMAL_STATUS_T status;

  /* Create the component */
  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("Failed to create camera component");
    goto error;
  }

  status = raspicamcontrol_set_stereo_mode(camera->output[0], &state->camera_parameters.stereo_mode);
  status += raspicamcontrol_set_stereo_mode(camera->output[1], &state->camera_parameters.stereo_mode);
  status += raspicamcontrol_set_stereo_mode(camera->output[2], &state->camera_parameters.stereo_mode);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("Could not set stereo mode : error %d", status);
    goto error;
  }

  MMAL_PARAMETER_INT32_T camera_num =
      {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->common_settings.cameraNum};

  status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("Could not select camera : error %d", status);
    goto error;
  }

  if (!camera->output_num)
  {
    status = MMAL_ENOSYS;
    vcos_log_error("Camera doesn't have output ports");
    goto error;
  }

  status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->common_settings.sensor_mode);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("Could not set sensor mode : error %d", status);
    goto error;
  }

  preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
  video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
  still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

  // Enable the camera, and tell it its control callback function
  status = mmal_port_enable(camera->control, default_camera_control_callback);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("Unable to enable control port : error %d", status);
    goto error;
  }

  //  set up the camera configuration
  {
    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
        {
            {MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
            .max_stills_w = state->common_settings.width,
            .max_stills_h = state->common_settings.height,
            .stills_yuv422 = 0,
            .one_shot_stills = 0,
            .max_preview_video_w = state->common_settings.width,
            .max_preview_video_h = state->common_settings.height,
            .num_preview_video_frames = 3 + vcos_max(0, (state->framerate - 30) / 10),
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC};
    mmal_port_parameter_set(camera->control, &cam_config.hdr);
  }

  // Now set up the port formats

  // Set the encode format on the Preview port
  // HW limitations mean we need the preview to be the same size as the required recorded output

  format = preview_port->format;

  format->encoding = MMAL_ENCODING_OPAQUE;
  format->encoding_variant = MMAL_ENCODING_I420;

  if (state->camera_parameters.shutter_speed > 6000000)
  {
    MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                            {50, 1000},
                                            {166, 1000}};
    mmal_port_parameter_set(preview_port, &fps_range.hdr);
  }
  else if (state->camera_parameters.shutter_speed > 1000000)
  {
    MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                            {166, 1000},
                                            {999, 1000}};
    mmal_port_parameter_set(preview_port, &fps_range.hdr);
  }

  //enable dynamic framerate if necessary
  if (state->camera_parameters.shutter_speed)
  {
    if (state->framerate > 1000000. / state->camera_parameters.shutter_speed)
    {
      state->framerate = 0;
      if (state->common_settings.verbose)
        fprintf(stderr, "Enable dynamic frame rate to fulfil shutter speed requirement\n");
    }
  }

  format->encoding = MMAL_ENCODING_OPAQUE;
  format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
  format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = state->common_settings.width;
  format->es->video.crop.height = state->common_settings.height;
  format->es->video.frame_rate.num = state->framerate;
  format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

  status = mmal_port_format_commit(preview_port);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("camera viewfinder format couldn't be set");
    goto error;
  }

  // Set the encode format on the video  port

  format = video_port->format;
  format->encoding_variant = MMAL_ENCODING_I420;

  if (state->camera_parameters.shutter_speed > 6000000)
  {
    MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                            {50, 1000},
                                            {166, 1000}};
    mmal_port_parameter_set(video_port, &fps_range.hdr);
  }
  else if (state->camera_parameters.shutter_speed > 1000000)
  {
    MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                                            {167, 1000},
                                            {999, 1000}};
    mmal_port_parameter_set(video_port, &fps_range.hdr);
  }

  format->encoding = MMAL_ENCODING_OPAQUE;
  format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
  format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = state->common_settings.width;
  format->es->video.crop.height = state->common_settings.height;
  format->es->video.frame_rate.num = state->framerate;
  format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

  status = mmal_port_format_commit(video_port);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("camera video format couldn't be set");
    goto error;
  }

  // Ensure there are enough buffers to avoid dropping frames
  if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  // Set the encode format on the still  port

  format = still_port->format;

  format->encoding = MMAL_ENCODING_OPAQUE;
  format->encoding_variant = MMAL_ENCODING_I420;

  format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
  format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
  format->es->video.crop.x = 0;
  format->es->video.crop.y = 0;
  format->es->video.crop.width = state->common_settings.width;
  format->es->video.crop.height = state->common_settings.height;
  format->es->video.frame_rate.num = 0;
  format->es->video.frame_rate.den = 1;

  status = mmal_port_format_commit(still_port);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("camera still format couldn't be set");
    goto error;
  }

  /* Ensure there are enough buffers to avoid dropping frames */
  if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

  /* Enable component */
  status = mmal_component_enable(camera);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("camera component couldn't be enabled");
    goto error;
  }

  // Note: this sets lots of parameters that were not individually addressed before.
  raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

  state->camera_component = camera;

  update_annotation_data(state);

  if (state->common_settings.verbose)
    fprintf(stderr, "Camera component done\n");

  return status;

error:

  if (camera)
    mmal_component_destroy(camera);

  return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state)
{
  if (state->camera_component)
  {
    mmal_component_destroy(state->camera_component);
    state->camera_component = NULL;
  }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_encoder_component(RASPIVID_STATE *state)
{
  MMAL_COMPONENT_T *encoder = 0;
  MMAL_PORT_T *encoder_input = NULL;
  MMAL_STATUS_T status;
  MMAL_POOL_T *pool;

  status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("Unable to create video encoder component");
    goto error;
  }

  if (!encoder->input_num || !encoder->output_num)
  {
    status = MMAL_ENOSYS;
    vcos_log_error("Video encoder doesn't have input/output ports");
    goto error;
  }

  encoder_input = encoder->input[0];
  encoder_output = encoder->output[0];

  // We want same format on input and output
  mmal_format_copy(encoder_output->format, encoder_input->format);

  // Only supporting H264 at the moment
  encoder_output->format->encoding = state->encoding;

  encoder_output->format->bitrate = state->bitrate;

  if (state->encoding == MMAL_ENCODING_H264)
    encoder_output->buffer_size = encoder_output->buffer_size_recommended;
  else
    encoder_output->buffer_size = 256 << 10;

  if (encoder_output->buffer_size < encoder_output->buffer_size_min)
    encoder_output->buffer_size = encoder_output->buffer_size_min;

  encoder_output->buffer_num = encoder_output->buffer_num_recommended;

  if (encoder_output->buffer_num < encoder_output->buffer_num_min)
    encoder_output->buffer_num = encoder_output->buffer_num_min;

  // We need to set the frame rate on output to 0, to ensure it gets
  // updated correctly from the input framerate when port connected
  encoder_output->format->es->video.frame_rate.num = 0;
  encoder_output->format->es->video.frame_rate.den = 1;

  // Commit the port changes to the output port
  status = mmal_port_format_commit(encoder_output);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("Unable to set format on video encoder output port");
    goto error;
  }

  // Set the rate control parameter
  if (0)
  {
    MMAL_PARAMETER_VIDEO_RATECONTROL_T param = {{MMAL_PARAMETER_RATECONTROL, sizeof(param)}, MMAL_VIDEO_RATECONTROL_DEFAULT};
    status = mmal_port_parameter_set(encoder_output, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set ratecontrol");
      goto error;
    }
  }

  if (state->encoding == MMAL_ENCODING_H264 &&
      state->intraperiod != -1)
  {
    MMAL_PARAMETER_UINT32_T param = {{MMAL_PARAMETER_INTRAPERIOD, sizeof(param)}, state->intraperiod};
    status = mmal_port_parameter_set(encoder_output, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set intraperiod");
      goto error;
    }
  }

  if (state->encoding == MMAL_ENCODING_H264 && state->slices > 1 && state->common_settings.width <= 1280)
  {
    int frame_mb_rows = VCOS_ALIGN_UP(state->common_settings.height, 16) >> 4;

    if (state->slices > frame_mb_rows) //warn user if too many slices selected
    {
      fprintf(stderr, "H264 Slice count (%d) exceeds number of macroblock rows (%d). Setting slices to %d.\n", state->slices, frame_mb_rows, frame_mb_rows);
      // Continue rather than abort..
    }
    int slice_row_mb = frame_mb_rows / state->slices;
    if (frame_mb_rows - state->slices * slice_row_mb)
      slice_row_mb++; //must round up to avoid extra slice if not evenly divided

    status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_MB_ROWS_PER_SLICE, slice_row_mb);
    if (status != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set number of slices");
      goto error;
    }
  }

  if (state->encoding == MMAL_ENCODING_H264 &&
      state->quantisationParameter)
  {
    MMAL_PARAMETER_UINT32_T param = {{MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param)}, state->quantisationParameter};
    status = mmal_port_parameter_set(encoder_output, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set initial QP");
      goto error;
    }

    MMAL_PARAMETER_UINT32_T param2 = {{MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param)}, state->quantisationParameter};
    status = mmal_port_parameter_set(encoder_output, &param2.hdr);
    if (status != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set min QP");
      goto error;
    }

    MMAL_PARAMETER_UINT32_T param3 = {{MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param)}, state->quantisationParameter};
    status = mmal_port_parameter_set(encoder_output, &param3.hdr);
    if (status != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set max QP");
      goto error;
    }
  }

  if (state->encoding == MMAL_ENCODING_H264)
  {
    MMAL_PARAMETER_VIDEO_PROFILE_T param;
    param.hdr.id = MMAL_PARAMETER_PROFILE;
    param.hdr.size = sizeof(param);

    param.profile[0].profile = state->profile;

    if ((VCOS_ALIGN_UP(state->common_settings.width, 16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height, 16) >> 4) * state->framerate > 245760)
    {
      if ((VCOS_ALIGN_UP(state->common_settings.width, 16) >> 4) * (VCOS_ALIGN_UP(state->common_settings.height, 16) >> 4) * state->framerate <= 522240)
      {
        fprintf(stderr, "Too many macroblocks/s: Increasing H264 Level to 4.2\n");
        state->level = MMAL_VIDEO_LEVEL_H264_42;
      }
      else
      {
        vcos_log_error("Too many macroblocks/s requested");
        status = MMAL_EINVAL;
        goto error;
      }
    }

    param.profile[0].level = state->level;

    status = mmal_port_parameter_set(encoder_output, &param.hdr);
    if (status != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set H264 profile");
      goto error;
    }
  }

  if (mmal_port_parameter_set_boolean(encoder_input, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, state->immutableInput) != MMAL_SUCCESS)
  {
    vcos_log_error("Unable to set immutable input flag");
    // Continue rather than abort..
  }

  if (state->encoding == MMAL_ENCODING_H264)
  {
    //set INLINE HEADER flag to generate SPS and PPS for every IDR if requested
    if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, state->bInlineHeaders) != MMAL_SUCCESS)
    {
      vcos_log_error("failed to set INLINE HEADER FLAG parameters");
      // Continue rather than abort..
    }

    //set flag for add SPS TIMING
    if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, state->addSPSTiming) != MMAL_SUCCESS)
    {
      vcos_log_error("failed to set SPS TIMINGS FLAG parameters");
      // Continue rather than abort..
    }

    //set INLINE VECTORS flag to request motion vector estimates
    if (mmal_port_parameter_set_boolean(encoder_output, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, state->inlineMotionVectors) != MMAL_SUCCESS)
    {
      vcos_log_error("failed to set INLINE VECTORS parameters");
      // Continue rather than abort..
    }

    // Adaptive intra refresh settings
    if (state->intra_refresh_type != -1)
    {
      MMAL_PARAMETER_VIDEO_INTRA_REFRESH_T param;
      param.hdr.id = MMAL_PARAMETER_VIDEO_INTRA_REFRESH;
      param.hdr.size = sizeof(param);

      // Get first so we don't overwrite anything unexpectedly
      status = mmal_port_parameter_get(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
        vcos_log_warn("Unable to get existing H264 intra-refresh values. Please update your firmware");
        // Set some defaults, don't just pass random stack data
        param.air_mbs = param.air_ref = param.cir_mbs = param.pir_mbs = 0;
      }

      param.refresh_mode = state->intra_refresh_type;

      status = mmal_port_parameter_set(encoder_output, &param.hdr);
      if (status != MMAL_SUCCESS)
      {
        vcos_log_error("Unable to set H264 intra-refresh values");
        goto error;
      }
    }
  }

  //  Enable component
  status = mmal_component_enable(encoder);

  if (status != MMAL_SUCCESS)
  {
    vcos_log_error("Unable to enable video encoder component");
    goto error;
  }

  /* Create pool of buffer headers for the output port to consume */
  pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

  if (!pool)
  {
    vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
  }

  state->encoder_pool = pool;
  state->encoder_component = encoder;

  if (state->common_settings.verbose)
    fprintf(stderr, "Encoder component done\n");

  return status;

error:
  if (encoder)
    mmal_component_destroy(encoder);

  state->encoder_component = NULL;

  return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state)
{
  // Get rid of any port buffers first
  if (state->encoder_pool)
  {
    mmal_port_pool_destroy(state->encoder_component->output[0], state->encoder_pool);
  }

  if (state->encoder_component)
  {
    mmal_component_destroy(state->encoder_component);
    state->encoder_component = NULL;
  }
}

static void initial_state(RASPIVID_STATE *state)
{
  // CommandWidth
  state->width = width;
  state->common_settings.width = width;
  // CommandHeight
  state->height = height;
  state->common_settings.height = height;

  // CommandBitrate
  state->bitrate = bitratekbps * 1000;
  // CommandFramerate
  state->framerate = frameratefps;
  // CommandIntraPeriod
  state->intraperiod = intra_period;

  // CommandProfile
  state->profile = profile;
  // CommandSignal
  state->waitMethod = WAIT_METHOD_NONE;
  // Reenable the signal
  signal(SIGUSR1, default_signal_handler);
  // CommandTimeout
  state->timeout = 0;
  // CommandInlineHeaders
  state->bInlineHeaders = 1;
}

bool cr(int kbps, int fps)
{
  int lastkbps = bitratekbps;
  int lastfps = frameratefps;
  bitratekbps = kbps;
  frameratefps = fps;
  // not sure why, but error occurs when fps is less than 3
  if (frameratefps < 3)
  {
    frameratefps = 3;
  }
  MMAL_STATUS_T status;
  if (frameratefps != lastfps)
  {
    MMAL_PARAMETER_FRAME_RATE_T param = {{MMAL_PARAMETER_VIDEO_FRAME_RATE, sizeof(param)}, {frameratefps, 1}};
    if (mmal_port_parameter_set(video_port, &param.hdr) != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set framerate");
      return false;
    }
  }
  if (bitratekbps != lastkbps || frameratefps != lastfps)
  {
    double cor_coef = frameratefps / (double)lastfps;
    if (cor_coef > 1.0)
      cor_coef = 1.0;
    int bps = (int)((15.0 / frameratefps) * bitratekbps * 1000 * cor_coef);
    MMAL_PARAMETER_UINT32_T bps_param = {{MMAL_PARAMETER_VIDEO_BIT_RATE, sizeof(bps_param)}, bps};
    if (mmal_port_parameter_set(encoder_output, &bps_param.hdr) != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set bitrate. kbps=%d, fps=%d, bps=%d", bitratekbps, frameratefps, bps);
      return false;
    }

    intra_period = frameratefps * 10;
    MMAL_PARAMETER_UINT32_T intra_param = {{MMAL_PARAMETER_INTRAPERIOD, sizeof(intra_param)}, intra_period};
    if (mmal_port_parameter_set(encoder_output, &intra_param.hdr) != MMAL_SUCCESS)
    {
      vcos_log_error("Unable to set intraperiod. %d", intra_period);
      return false;
    }
  }

  return true;
}

void s(int w, int h, int kbps, int fps, void (*callback)())
{
  if (loop_flag)
    return;

  loop_flag = true;
  width = w;
  height = h;
  bitratekbps = kbps;
  frameratefps = fps;
  intra_period = frameratefps * 10;

  extern void *mmal_encoder_thread(void *);
  mmal_encoded_frame_callback = callback;
  pthread_create(&thread, NULL, &mmal_encoder_thread, NULL);
}

void st()
{
  if (!loop_flag)
    return;

  loop_flag = false;
  pthread_join(thread, NULL);
}

void *mmal_encoder_thread(void *e)
{
  extern void mmal_encoder_run();
  mmal_encoder_run();
}

void mmal_encoder_run()
{
  // Our main data storage vessel..
  RASPIVID_STATE state;
  int exit_code = EX_OK;

  MMAL_STATUS_T status = MMAL_SUCCESS;
  MMAL_PORT_T *camera_preview_port = NULL;
  MMAL_PORT_T *camera_video_port = NULL;
  MMAL_PORT_T *camera_still_port = NULL;
  MMAL_PORT_T *preview_input_port = NULL;
  MMAL_PORT_T *encoder_input_port = NULL;
  MMAL_PORT_T *encoder_output_port = NULL;

  bcm_host_init();

  char *app_name = "MmalEncoder";

  // Register our application with the logging system
  vcos_log_register(app_name, VCOS_LOG_CATEGORY);

  signal(SIGINT, default_signal_handler);

  // Disable USR1 for the moment - may be reenabled if go in to signal capture mode
  signal(SIGUSR1, SIG_IGN);

  default_status(&state);

  // set initial state
  initial_state(&state);

  check_camera_model(state.common_settings.cameraNum);

  // OK, we have a nice set of parameters. Now set up our components
  // We have three components. Camera, Preview and encoder.

  status = create_camera_component(&state);
  if (status != MMAL_SUCCESS)
  {
    fprintf(stderr, "error: crete_camera_component\n");
    vcos_log_error("%s: Failed to create camera component", __func__);
    exit_code = EX_SOFTWARE;
    goto error;
  }

  status = raspipreview_create(&state.preview_parameters);
  if (status != MMAL_SUCCESS)
  {
    fprintf(stderr, "error: rapsipreview_create\n");
    vcos_log_error("%s: Failed to create preview component", __func__);
    destroy_camera_component(&state);
    exit_code = EX_SOFTWARE;
    goto error;
  }

  status = create_encoder_component(&state);
  if (status != MMAL_SUCCESS)
  {
    fprintf(stderr, "error: create_encoder_component\n");
    vcos_log_error("%s: Failed to create encode component", __func__);
    raspipreview_destroy(&state.preview_parameters);
    destroy_camera_component(&state);
    exit_code = EX_SOFTWARE;
    goto error;
  }

  camera_preview_port = state.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
  camera_video_port = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
  camera_still_port = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
  preview_input_port = state.preview_parameters.preview_component->input[0];
  encoder_input_port = state.encoder_component->input[0];
  encoder_output_port = state.encoder_component->output[0];

  if (status == MMAL_SUCCESS)
  {
    // Now connect the camera to the encoder
    status = connect_ports(camera_video_port, encoder_input_port, &state.encoder_connection);

    if (status != MMAL_SUCCESS)
    {
      fprintf(stderr, "error: connect_ports\n");
      state.encoder_connection = NULL;
      vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
      goto error;
    }
  }

  if (status == MMAL_SUCCESS)
  {
    // Set up our userdata - this is passed though to the callback where we need the information.
    state.callback_data.pstate = &state;
    state.callback_data.abort = 0;

    // Set up our userdata - this is passed though to the callback where we need the information.
    encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

    // Enable the encoder output port and tell it its callback function
    status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

    if (status != MMAL_SUCCESS)
    {
      fprintf(stderr, "error: mmal_port_enable\n");
      vcos_log_error("Failed to setup encoder output");
      goto error;
    }

    int num = mmal_queue_length(state.encoder_pool->queue);
    int q;
    for (q = 0; q < num; q++)
    {
      MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);

      if (!buffer)
        vcos_log_error("Unable to get a required buffer %d from pool queue", q);

      if (mmal_port_send_buffer(encoder_output_port, buffer) != MMAL_SUCCESS)
        vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
    }

    if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
    {
      fprintf(stderr, "error: mmal_port_parameter_set camera_video_port is failed\n");
      // How to handle?
      goto error;
    }

    fprintf(stdout, "mmal_port_parameter_set_boolean succeed...\n");
    // sleep until set loop_flag to false
    while (loop_flag)
    {
      vcos_sleep(1000);
    }

    fprintf(stdout, "while loop exit\n");
  }
error:

  mmal_status_to_int(status);

  // Disable all our ports that are not handled by connections
  check_disable_port(camera_still_port);
  check_disable_port(encoder_output_port);

  if (state.preview_parameters.wantPreview && state.preview_connection)
    mmal_connection_destroy(state.preview_connection);

  if (state.encoder_connection)
    mmal_connection_destroy(state.encoder_connection);

  // Can now close our file. Note disabling ports may flush buffers which causes
  // problems if we have already closed the file!
  if (state.callback_data.imv_file_handle && state.callback_data.imv_file_handle != stdout)
    fclose(state.callback_data.imv_file_handle);
  if (state.callback_data.pts_file_handle && state.callback_data.pts_file_handle != stdout)
    fclose(state.callback_data.pts_file_handle);
  if (state.callback_data.raw_file_handle && state.callback_data.raw_file_handle != stdout)
    fclose(state.callback_data.raw_file_handle);

  /* Disable components */
  if (state.encoder_component)
    mmal_component_disable(state.encoder_component);

  if (state.preview_parameters.preview_component)
    mmal_component_disable(state.preview_parameters.preview_component);

  if (state.camera_component)
    mmal_component_disable(state.camera_component);

  destroy_encoder_component(&state);
  raspipreview_destroy(&state.preview_parameters);
  destroy_camera_component(&state);

  if (status != MMAL_SUCCESS)
    raspicamcontrol_check_configuration(128);
}
