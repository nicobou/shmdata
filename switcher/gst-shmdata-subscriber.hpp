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

#ifndef __SWITCHER_GST_SHMDATA_TO_TREE_H__
#define __SWITCHER_GST_SHMDATA_TO_TREE_H__

#include <gst/gst.h>
#include <future>
#include <atomic>
#include "./periodic-task.hpp"

namespace switcher {
class GstShmdataSubscriber {
 public:
  using num_bytes_t = guint64;
  using on_caps_cb_t = std::function<void(const std::string &)>;
  using on_byte_monitor_t = std::function<void(num_bytes_t)>;
  GstShmdataSubscriber(GstElement *element,
                       on_caps_cb_t on_caps_cb,
                       on_byte_monitor_t on_byte_monitor_cb);
  GstShmdataSubscriber() = delete;
  GstShmdataSubscriber(const GstShmdataSubscriber &) = delete;
  GstShmdataSubscriber &operator=(const GstShmdataSubscriber &) = delete;
  ~GstShmdataSubscriber() = default;
  
 private:
  GstElement *element_;
  on_caps_cb_t on_caps_cb_;
  on_byte_monitor_t on_byte_monitor_cb_;
  PeriodicTask ptask_;
  static void on_caps_cb(GObject *gobject, GParamSpec *pspec, gpointer user_data);
  void byte_monitor();
  void notify_caps();
};

}  // namespace switcher
#endif
