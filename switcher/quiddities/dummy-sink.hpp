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

#ifndef __SWITCHER_DUMMY_SINK_H__
#define __SWITCHER_DUMMY_SINK_H__

#include "../quiddity/quiddity.hpp"
#include "../shmdata/connector.hpp"
#include "../shmdata/follower.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;
class DummySink : public Quiddity {
 public:
  DummySink(quiddity::Config&&);
  ~DummySink() = default;
  DummySink(const DummySink&) = delete;
  DummySink& operator=(const DummySink&) = delete;

 private:
  bool frame_received_{false};
  property::prop_id_t frame_received_id_;

  // registering connect/disconnect/can_sink_caps:
  shmdata::Connector shmcntr_;
  // shmdata follower
  std::unique_ptr<shmdata::Follower> shm_{nullptr};
  bool connect(const std::string& shmdata_path);
  bool disconnect();
  bool can_sink_caps(const std::string& caps);
};

}  // namespace quiddities
}  // namespace switcher
#endif
