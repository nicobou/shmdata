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

#ifndef __SWITCHER_GST_PIPELINER_H__
#define __SWITCHER_GST_PIPELINER_H__

#include <gst/gst.h>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "./glibmainloop.hpp"
#include "./gst-pipe.hpp"
#include "./quiddity.hpp"
#include "./unique-gst-element.hpp"

namespace switcher {
class Quiddity;
class QuiddityCommand;
class CustomPropertyHelper;

class GstPipeliner {
 public:
  using on_error_cb_t = std::function<void(GstObject*, GError*)>;
  GstPipeliner(GstPipe::on_msg_async_cb_t on_msg_async_cb,
               GstPipe::on_msg_sync_cb_t on_msg_sync_cb);
  GstPipeliner(GstPipe::on_msg_async_cb_t on_msg_async_cb,
               GstPipe::on_msg_sync_cb_t on_msg_sync_cb,
               on_error_cb_t on_error_cb);
  GstPipeliner() = delete;
  virtual ~GstPipeliner();
  GstPipeliner(const GstPipeliner&) = delete;
  GstPipeliner& operator=(const GstPipeliner&) = delete;

  GstElement* get_pipeline();
  void play(gboolean play);
  bool seek(gdouble position_in_ms);
  void looping(gboolean looping);

 private:
  GstPipe::on_msg_async_cb_t on_msg_async_cb_;
  GstPipe::on_msg_sync_cb_t on_msg_sync_cb_;
  on_error_cb_t on_error_cb_;
  bool watch_added_{false};
  std::mutex watch_mutex_{};
  std::condition_variable cond_watch_{};
  std::unique_ptr<GlibMainLoop> main_loop_;
  std::unique_ptr<GstPipe> gst_pipeline_;
  GstBusSyncReply on_gst_error(GstMessage* msg);
  static gboolean push_thread_context(gpointer user_data);
  static gboolean bus_watch(GstBus* bus, GstMessage* message, gpointer user_data);
  static gboolean bus_async(gpointer user_data);
  static GstBusSyncReply bus_sync_handler(GstBus* bus, GstMessage* msg, gpointer user_data);
};

}  // namespace switcher
#endif
