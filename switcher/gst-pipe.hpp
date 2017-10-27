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
#include <condition_variable>
#include <mutex>
#include "./bool-log.hpp"

namespace switcher {

class GstPipe {
 public:
  using on_msg_async_cb_t = std::function<void(GstMessage*)>;
  using on_msg_sync_cb_t = std::function<GstBusSyncReply(GstMessage*)>;
  GstPipe(GMainContext* context,
          GstBusSyncReply (*bus_sync_cb)(GstBus* /*bus*/, GstMessage* msg, gpointer user_data),
          gpointer user_data);
  ~GstPipe();
  GstPipe() = delete;
  GstPipe(const GstPipe&) = delete;
  GstPipe& operator=(const GstPipe&) = delete;
  bool play(bool play);
  BoolLog seek(gdouble position);
  BoolLog speed(gdouble speed);
  GstElement* get_pipeline();

 private:
  typedef struct {  // GstBus is a specific context:
    GSource source;
    GstBus* bus;
    gboolean inited;
  } GstBusSource;

  GMainContext* gmaincontext_{nullptr};
  GstElement* pipeline_{nullptr};
  GSourceFuncs source_funcs_;
  gdouble speed_{1.0};
  GSource* source_{nullptr};
  //  GSource *bus_watch_source_ {nullptr};
  gint64 length_{0};
  void query_position_and_length();
  static gboolean source_prepare(GSource* source, gint* timeout);
  static gboolean source_check(GSource* source);
  static gboolean source_dispatch(GSource* source, GSourceFunc callback, gpointer user_data);
  static void source_finalize(GSource* source);
  static void play_pipe(GstPipe* pipe);
};

}  // namespace switcher
#endif
