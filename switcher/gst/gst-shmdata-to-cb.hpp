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

#ifndef __SWITCHER_GST_SHMDATA_TO_CB_H__
#define __SWITCHER_GST_SHMDATA_TO_CB_H__

#include <functional>
#include <map>
#include <mutex>
#include "../utils/safe-bool-idiom.hpp"
#include "./gst-pipeliner.hpp"

namespace switcher {

class GstShmdataToCb : public SafeBoolIdiom {
 public:
  using id_t = size_t;
  using data_cb_t = std::function<void(void*, size_t)>;
  using on_caps_cb_t = std::function<GstElement*(const std::string& caps)>;
  // on_caps can be used for making a filter that will be inserted between the
  // shmdata reader
  // and the callback
  // After fun has been invoked, filter is owned by the created GstShmdataToCb
  // object
  GstShmdataToCb(const std::string& shmpath, on_caps_cb_t fun);
  ~GstShmdataToCb() = default;
  GstShmdataToCb() = delete;
  GstShmdataToCb(const GstShmdataToCb&) = delete;
  GstShmdataToCb(GstShmdataToCb&&) = delete;
  GstShmdataToCb& operator=(const GstShmdataToCb&) = delete;

  id_t add_cb(data_cb_t fun);
  bool remove_cb(id_t cb_id);
  // get caps for data in the callback:
  std::string get_caps() const;

 private:
  id_t counter_{0};
  bool is_valid_{false};
  bool safe_bool_idiom() const { return is_valid_; }
  GstPipeliner pipe_;
  on_caps_cb_t filter_cb_;
  std::map<id_t, data_cb_t> data_cbs_{};
  std::mutex mtx_{};
  GstElement* fakesink_{nullptr};
  mutable std::string fakesink_caps_{};
  static void on_handoff_cb(GstElement* /*object*/,
                            GstBuffer* buf,
                            GstPad* pad,
                            gpointer user_data);

  static void on_caps(GstElement* typefind,
                      guint /*probability */,
                      GstCaps* caps,
                      gpointer /*user_data*/);
};

}  // namespace switcher
#endif
