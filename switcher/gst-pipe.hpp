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
 * RAII GStreamer pipeline  
 */

#ifndef __SWITCHER_GST_PIPELINE_H__
#define __SWITCHER_GST_PIPELINE_H__

#include <gst/gst.h>
#include <mutex>
#include <condition_variable>

namespace switcher {

class GstPipe {
  using on_error_cb_t = std::function<void(GstMessage *)>;
  using on_eos_cb_t = std::function<void()>;
 public:
  GstPipe(GMainContext *context,
          on_error_cb_t on_error_cb,
          on_eos_cb_t on_eos_cb);
  ~GstPipe();
  GstPipe() = delete;
  GstPipe(const GstPipe &) = delete;
  GstPipe &operator=(const GstPipe &) = delete;
  bool play(bool play);
  bool seek(gdouble position);
  bool speed(gdouble speed);
  GstElement *get_pipeline();  // FIXME remove that
  
 private:
  typedef struct {  // GstBus is a specific context:
    GSource source;
    GstBus *bus;
    gboolean inited;
  } GstBusSource;
  
  on_error_cb_t on_error_cb_;
  on_eos_cb_t on_eos_cb_;
  GstElement *pipeline_ {nullptr};
  GMainContext *gmaincontext_{nullptr};
  GSourceFuncs source_funcs_;
  gdouble speed_ {1.0};
  std::mutex play_pipe_{};
  std::condition_variable play_cond_{};
  GSource *source_ {nullptr};
  gint64 length_ {0};

  void query_position_and_length();
  static gboolean source_prepare(GSource *source,
                                 gint *timeout);
  static gboolean source_check(GSource *source);
  static gboolean source_dispatch(GSource *source,
                                  GSourceFunc callback,
                                  gpointer user_data);
  static void source_finalize(GSource *source);
  
  static gboolean bus_called(GstBus * bus, GstMessage *msg, gpointer data);
  static GstBusSyncReply bus_sync_handler(GstBus * bus, GstMessage *msg,
                                          gpointer user_data);
  static void play_pipe(GstPipe *pipe);
};
}  // namespace switcher

#endif
