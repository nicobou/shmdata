/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * The GstPipeline RAII class
 */

#include "./gst-pipe.hpp"
#include <glib-object.h>
#include "./gst-utils.hpp"
#include "./scope-exit.hpp"

namespace switcher {
GstPipe::GstPipe(GMainContext* context,
                 GstBusSyncReply (*bus_sync_cb)(GstBus* /*bus*/,
                                                GstMessage* msg,
                                                gpointer user_data),
                 gpointer user_data)

    : gmaincontext_(context), pipeline_(gst_pipeline_new(nullptr)), source_funcs_() {
  source_funcs_.prepare = source_prepare;
  source_funcs_.check = source_check;
  source_funcs_.dispatch = source_dispatch;
  source_funcs_.finalize = source_finalize;
  source_ = g_source_new(&source_funcs_, sizeof(GstBusSource));
  reinterpret_cast<GstBusSource*>(source_)->bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
  g_source_attach(source_, gmaincontext_);
  // add a watch to a bus is not working,
  // (using g_idle_add from sync callback in GstPipeliner instead)
  gst_bus_set_sync_handler(
      reinterpret_cast<GstBusSource*>(source_)->bus, bus_sync_cb, user_data, nullptr);
  reinterpret_cast<GstBusSource*>(source_)->inited = FALSE;
}

GstPipe::~GstPipe() {
  GstUtils::wait_state_changed(pipeline_);
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline_));
  g_source_destroy(source_);
  g_source_unref(source_);
}

void GstPipe::play_pipe(GstPipe* pipe) {
  gst_element_set_state(pipe->pipeline_, GST_STATE_READY);
  GstUtils::wait_state_changed(pipe->pipeline_);
  gst_element_set_state(pipe->pipeline_, GST_STATE_PLAYING);
}

gboolean GstPipe::source_prepare(GSource* source, gint* timeout) {
  GstBusSource* bsrc = (GstBusSource*)source;
  *timeout = -1;
  return gst_bus_have_pending(bsrc->bus);
}

gboolean GstPipe::source_check(GSource* source) {
  GstBusSource* bsrc = (GstBusSource*)source;
  return gst_bus_have_pending(bsrc->bus);
}

gboolean GstPipe::source_dispatch(GSource* source, GSourceFunc callback, gpointer user_data) {
  GstBusFunc handler = (GstBusFunc)callback;
  GstBusSource* bsrc = (GstBusSource*)source;
  gboolean result = FALSE;

  if (handler) {
    GstMessage* message = gst_bus_pop(bsrc->bus);
    if (message) {
      result = handler(bsrc->bus, message, user_data);
      gst_message_unref(message);
    }
  }
  return result;
}

void GstPipe::source_finalize(GSource* source) {
  GstBusSource* bsrc = (GstBusSource*)source;
  gst_object_unref(bsrc->bus);
  bsrc->bus = nullptr;
}

bool GstPipe::play(bool play) {
  if (play) {
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  } else {
    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  }
  // GstUtils::wait_state_changed(pipeline_);
  return play;
}

BoolLog GstPipe::seek(gdouble position) {
  return gst_element_seek(pipeline_,
                          speed_,
                          GST_FORMAT_TIME,
                          (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_ACCURATE),
                          // | GST_SEEK_FLAG_SKIP
                          // using key unit is breaking synchronization
                          // | GST_SEEK_FLAG_KEY_UNIT,
                          GST_SEEK_TYPE_SET,
                          position * GST_MSECOND,
                          GST_SEEK_TYPE_NONE,
                          GST_CLOCK_TIME_NONE)
             ? BoolLog(true)
             : BoolLog(false, "seek not handled");
}

BoolLog GstPipe::speed(gdouble speed) {
  speed_ = speed;
  // query position
  GstQuery* query = gst_query_new_position(GST_FORMAT_TIME);
  gboolean res = gst_element_query(pipeline_, query);
  On_scope_exit { gst_query_unref(query); };
  gint64 cur_pos = 0;
  if (res) {
    gst_query_parse_position(query, nullptr, &cur_pos);
  } else {
    return BoolLog(false, "position query failed...");
  }

  return gst_element_seek(pipeline_,
                          speed,
                          GST_FORMAT_TIME,
                          (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_ACCURATE),
                          GST_SEEK_TYPE_SET,
                          cur_pos,
                          GST_SEEK_TYPE_NONE,
                          GST_CLOCK_TIME_NONE)
             ? BoolLog(true)
             : BoolLog(false, "speed not handled");
}

void GstPipe::query_position_and_length() {
  gint64 pos;
  if (gst_element_query_position(pipeline_, GST_FORMAT_TIME, &pos) &&
      gst_element_query_duration(pipeline_, GST_FORMAT_TIME, &length_)) {
    // g_print ("Time: %" GST_TIME_FORMAT " / %" GST_TIME_FORMAT "\r",
    //  GST_TIME_ARGS (pos), GST_TIME_ARGS (length_));
  }
}

GstElement* GstPipe::get_pipeline() { return pipeline_; }

}  // namespace switcher
