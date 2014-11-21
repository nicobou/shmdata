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

#include <glib-object.h>
#include <gst/interfaces/xoverlay.h>
#include <shmdata/base-reader.h>
#include "./gst-utils.hpp"
#include "./scope-exit.hpp"
#include "./gst-pipe.hpp"

namespace switcher {
GstPipe::GstPipe(GMainContext *context) :
    pipeline_ (gst_pipeline_new(nullptr)),
    gmaincontext_(context),
    source_funcs_() {

  source_funcs_.prepare = source_prepare;
  source_funcs_.check = source_check;
  source_funcs_.dispatch = source_dispatch;
  source_funcs_.finalize = source_finalize;
  source_ = g_source_new(&source_funcs_, sizeof(GstBusSource));
  ((GstBusSource *) source_)->bus =
      gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
  g_source_set_callback(source_,
                        (GSourceFunc) bus_called,
                        this,
                        nullptr);
  g_source_attach(source_, gmaincontext_);
  gst_bus_set_sync_handler(((GstBusSource *) source_)->bus,
                           bus_sync_handler,
                           this);
  ((GstBusSource *) source_)->inited = FALSE;
  {
    std::unique_lock<std::mutex> lock(play_pipe_);
    GstUtils::g_idle_add_full_with_context(gmaincontext_,
                                           G_PRIORITY_DEFAULT_IDLE,
                                           (GSourceFunc)play_pipe,
                                           (gpointer)this,
                                           nullptr);
    play_cond_.wait(lock);
  }

}
GstPipe::~GstPipe() {
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline_));
  g_source_destroy(source_);
  g_source_unref(source_);
}

gboolean GstPipe::bus_called(GstBus */*bus */,
                             GstMessage *msg,
                             gpointer /*user_data*/) {
  gchar *debug = nullptr;
  GError *error = nullptr;
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_EOS:
      g_warning("bus_call End of stream, name: %s", GST_MESSAGE_SRC_NAME(msg));
      break;
    case GST_MESSAGE_SEGMENT_DONE:
      g_debug("bus_call segment done");
      break;
    case GST_MESSAGE_ERROR:
      gst_message_parse_error(msg, &error, &debug);
      g_free(debug);
      g_debug("bus_call Error: %s from %s", error->message,
              GST_MESSAGE_SRC_NAME(msg));
      g_error_free(error);
      return FALSE;
      break;
    case GST_MESSAGE_STATE_CHANGED:
      // GstState old_state, new_state;
      // gst_message_parse_state_changed (msg, &old_state, &new_state, nullptr);
      // g_debug ("Element %s changed state from %s to %s.",
      //        GST_OBJECT_NAME (msg->src),
      //        gst_element_state_get_name (old_state),
      //        gst_element_state_get_name (new_state));
      // if (GST_IS_ELEMENT (GST_ELEMENT_PARENT (msg->src)))
      // {
      //   g_debug ("parent :%s (%s)",
      //    GST_OBJECT_NAME (GST_ELEMENT_PARENT (msg->src)),
      //    gst_element_state_get_name (GST_STATE(GST_ELEMENT_PARENT (msg->src))));
      // }
      break;
    default:
      // g_debug ("message %s from %s",GST_MESSAGE_TYPE_NAME(msg),GST_MESSAGE_SRC_NAME(msg));
      break;
  }
  return TRUE;
}


GstBusSyncReply GstPipe::bus_sync_handler(GstBus * /*bus*/,
                                          GstMessage *msg,
                                          gpointer user_data) {
  shmdata_base_reader_t *reader =
      (shmdata_base_reader_t *) g_object_get_data(G_OBJECT(msg->src),
                                                  "shmdata_base_reader");
  GstPipe *context = static_cast<GstPipe *>(user_data);

  // g_print ("-----------%s-----%s--------------------------\n",
  //      G_OBJECT_TYPE_NAME(G_OBJECT (msg->src)),
  //      GST_MESSAGE_TYPE_NAME (msg));

  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_QOS) {
    GstFormat format;
    guint64 processed;
    guint64 dropped;
    gst_message_parse_qos_stats(msg, &format, &processed, &dropped);
    // g_print ("QOS from %s, format %d, processed %lu dropped %lu\n",
    //  G_OBJECT_TYPE_NAME(G_OBJECT (msg->src)),
    //  format,
    //  processed,
    //  dropped);
    return GST_BUS_PASS;
  }

  if (reader != nullptr) {
    if (nullptr != msg && shmdata_base_reader_process_error(reader, msg))
      return GST_BUS_DROP;
    else
      return GST_BUS_PASS;
  }

  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
    gchar *debug = nullptr;
    GError *error = nullptr;
    gst_message_parse_error(msg, &error, &debug);
    g_free(debug);
    g_debug("Gstreamer error: %s (element %s)", error->message,
            GST_MESSAGE_SRC_NAME(msg));
    g_error_free(error);

    if (context->on_error_function_)
      context->on_error_function_(msg);

    return GST_BUS_DROP;
  }

  if (nullptr != msg->structure) {
    if (gst_structure_has_name(msg->structure, "prepare-xwindow-id")) {
      guintptr *window_handle =
          (guintptr *) g_object_get_data(G_OBJECT(msg->src),
                                         "window-handle");
      if (window_handle != nullptr) {
        gst_x_overlay_set_window_handle(GST_X_OVERLAY(msg->src),
                                        *window_handle);
      }
    }
  }

  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_TAG) {
    // GstTagList *tags = nullptr;
    // gst_message_parse_tag (msg, &tags);
    // g_print ("Got tags from element %s:\n", GST_OBJECT_NAME (msg->src));
    // gst_tag_list_foreach (tags, print_one_tag, nullptr);
    // g_print ("\n");
    // gst_tag_list_free (tags);
  }
  return GST_BUS_PASS;
}

void GstPipe::play_pipe(GstPipe *pipe) {
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  std::unique_lock<std::mutex> lock(pipe->play_pipe_);
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  gst_element_set_state(pipe->pipeline_, GST_STATE_PLAYING);
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  GstUtils::wait_state_changed(pipe->pipeline_);
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  //pipe->make_bin();
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  pipe->play_cond_.notify_one();
  g_print("%s %d\n", __FUNCTION__, __LINE__);
}


gboolean GstPipe::source_prepare(GSource * source, gint *timeout) {
  GstBusSource *bsrc = (GstBusSource *) source;
  *timeout = -1;
  return gst_bus_have_pending(bsrc->bus);
}

gboolean GstPipe::source_check(GSource *source) {
  GstBusSource *bsrc = (GstBusSource *) source;
  return gst_bus_have_pending(bsrc->bus);
}

gboolean
GstPipe::source_dispatch(GSource *source, GSourceFunc callback,
                         gpointer user_data) {
  GstBusFunc handler = (GstBusFunc) callback;
  GstBusSource *bsrc = (GstBusSource *) source;
  gboolean result = FALSE;

  if (handler) {
    GstMessage *message = gst_bus_pop(bsrc->bus);
    if (message) {
      result = handler(bsrc->bus, message, user_data);
      gst_message_unref(message);
    }
  }
  return result;
}

void GstPipe::source_finalize(GSource *source) {
  GstBusSource *bsrc = (GstBusSource *) source;
  gst_object_unref(bsrc->bus);
  bsrc->bus = nullptr;
}

bool GstPipe::play(bool play)
{
  if (play) {
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  } else {
    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  }
  GstUtils::wait_state_changed(pipeline_);
  return play;
}

bool GstPipe::seek(gdouble position) {
  gboolean ret = gst_element_seek(pipeline_,
                                  speed_,
                                  GST_FORMAT_TIME,
                                  (GstSeekFlags)(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_ACCURATE),
                                  // | GST_SEEK_FLAG_SKIP
                                  // using key unit is breaking synchronization
                                  // | GST_SEEK_FLAG_KEY_UNIT,
                                  GST_SEEK_TYPE_SET,
                                  position  * GST_MSECOND,
                                  GST_SEEK_TYPE_NONE,
                                  GST_CLOCK_TIME_NONE);
  if (!ret) {
    g_debug("seek not handled\n");
    return false;
  }
  return true;
}

bool GstPipe::speed(gdouble speed) {
  g_debug("GstPipeliner::speed %f", speed);

  speed_ = speed;
  GstQuery *query;
  gboolean res;

  // query position
  query = gst_query_new_position(GST_FORMAT_TIME);
  res = gst_element_query(pipeline_, query);
  gint64 cur_pos = 0;
  if (res) {
    gst_query_parse_position(query, nullptr, &cur_pos);

    g_debug("cur pos = %" GST_TIME_FORMAT "\n", GST_TIME_ARGS(cur_pos));
  } else {
    g_warning("position query failed...");
  }
  gst_query_unref(query);

  gboolean ret;
  ret = gst_element_seek(pipeline_,
                         speed,
                         GST_FORMAT_TIME,
                         (GstSeekFlags) (GST_SEEK_FLAG_FLUSH |
                                         GST_SEEK_FLAG_ACCURATE),
                         GST_SEEK_TYPE_SET,
                         cur_pos, GST_SEEK_TYPE_NONE, GST_CLOCK_TIME_NONE);

  if (!ret)
    g_debug("speed not handled\n");
  return true;
}

void GstPipe::query_position_and_length() {
  GstFormat fmt = GST_FORMAT_TIME;
  gint64 pos;
  if (gst_element_query_position(pipeline_, &fmt, &pos)
      && gst_element_query_duration(pipeline_, &fmt, &length_)) {
    // g_print ("Time: %" GST_TIME_FORMAT " / %" GST_TIME_FORMAT "\r",
    //  GST_TIME_ARGS (pos), GST_TIME_ARGS (length_));
  }
}

GstElement *GstPipe::get_pipeline() {
  return pipeline_;
}

void GstPipe::set_on_error_function(std::function<void(GstMessage *)> fun) {
  on_error_function_ = std::move(fun);
}

}  // namespace switcher
