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

#ifndef __SWITCHER_GST_SHM_TREE_UPDATER_H__
#define __SWITCHER_GST_SHM_TREE_UPDATER_H__

#include "./gst-shmdata-subscriber.hpp"

namespace switcher {
class GstShmTreeUpdater {
 public:
  using on_caps_cb_t = std::function<void(const std::string&)>;
  using on_delete_t = std::function<void()>;
  enum class Direction { writer, reader };
  GstShmTreeUpdater(Quiddity* quid,
                    GstElement* element,
                    const std::string& shmpath,
                    Direction d,
                    on_caps_cb_t on_caps_cb = nullptr,
                    on_delete_t on_delete_cb = nullptr);
  ~GstShmTreeUpdater();
  GstShmTreeUpdater() = delete;

 private:
  Quiddity* quid_;
  std::string shmpath_;
  Direction dir_;
  std::string key_;
  on_delete_t on_del_;
  GstShmdataSubscriber shm_sub_;
};

}  // namespace switcher
#endif
