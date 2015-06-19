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
#include <memory>
#include <vector>
#include <mutex>
#include <list>
#include <string>
#include "./quiddity.hpp"
#include "./glibmainloop.hpp"
#include "./gst-pipe.hpp"
#include "./unique-gst-element.hpp"

namespace switcher {
class Quiddity;
class QuiddityCommand;
class CustomPropertyHelper;
class DecodebinToShmdata;

class GstPipeliner {
  friend DecodebinToShmdata;

 public:
  GstPipeliner(GstPipe::on_msg_async_cb_t on_msg_async_cb,
               GstPipe::on_msg_sync_cb_t on_msg_sync_cb);
  GstPipeliner() = delete;
  virtual ~GstPipeliner();
  GstPipeliner(const GstPipeliner &) = delete;
  GstPipeliner &operator=(const GstPipeliner &) = delete;

  GstElement *get_pipeline();
  void play(gboolean play);
  bool seek(gdouble position_in_ms);
  void looping(gboolean looping);
  
 private:
    typedef struct {
    GstPipeliner *self;
    QuiddityCommand *command;
    GSource *src;
  } QuidCommandArg;

 private:
  GstPipe::on_msg_async_cb_t on_msg_async_cb_;
  GstPipe::on_msg_sync_cb_t on_msg_sync_cb_;
  std::unique_ptr<GlibMainLoop> main_loop_;
  std::unique_ptr<GstPipe> gst_pipeline_;
  std::list<GstMessage *> msgs_{};
  std::vector<QuidCommandArg *>commands_ {};
  GstBusSyncReply on_gst_error(GstMessage *msg);
  static gboolean bus_async(gpointer user_data);
  static GstBusSyncReply bus_sync_handler(GstBus * bus,
                                          GstMessage *msg,
                                          gpointer user_data);
};

}  // namespace switcher
#endif
