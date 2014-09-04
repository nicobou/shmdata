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

#ifndef __SWITCHER_SHMDATA_READER_H__
#define __SWITCHER_SHMDATA_READER_H__

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <shmdata/base-reader.h>
#include "./gst-element-cleaner.hpp"
#include "./json-builder.hpp"
#include "./on-caps.hpp"

namespace switcher {
class ShmdataReader:public OnCaps, public GstElementCleaner {
 public:
  typedef std::shared_ptr < ShmdataReader > ptr;
  typedef void (*on_first_data_hook) (ShmdataReader * caller,
                                      void *user_data);

  ShmdataReader();
  ~ShmdataReader();
  ShmdataReader(const ShmdataReader &) = delete;
  ShmdataReader & operator=(const ShmdataReader &) = delete;
  void set_path(const char *absolute_path);
  void set_bin(GstElement * bin);
  void set_g_main_context(GMainContext * context);
  void set_sink_element(GstElement * sink_element);
  void set_on_first_data_hook(on_first_data_hook cb, void *user_data);
  std::string get_path();
  void start();
  void stop();
  // get json doc:
  JSONBuilder::Node get_json_root_node();

 private:
  on_first_data_hook connection_hook_;
  void *hook_user_data_;
  std::string path_;
  shmdata_base_reader_t *reader_;
  GstElement *bin_;
  GstElement *sink_element_;
  GstElement *funnel_;
  GMainContext *g_main_context_;
  std::vector < GstElement * >elements_to_remove_;
  JSONBuilder::ptr json_description_;
  std::mutex start_mutex_;
  std::condition_variable start_cond_;
  static void on_first_data(shmdata_base_reader_t * context,
                            void *user_data);
  // static GstBusSyncReply bus_sync_handler (GstBus *bus, GstMessage *msg, gpointer user_data);
  static void unlink_pad(GstPad * pad);
  static void on_have_type(shmdata_base_reader_t * base_reader,
                           GstCaps * caps, void *user_data);
  void make_json_description();
  static gboolean start_idle(void *user_data);
};
}  // namespace switcher

#endif                          // ifndef