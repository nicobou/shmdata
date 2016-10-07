/*
 * This file is part of switcher-vrpn.
 *
 * switcher-vrpn is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_VRPN_SOURCE_H__
#define __SWITCHER_VRPN_SOURCE_H__

#include <memory>
#include "switcher/periodic-task.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"
#include "vrpn-client-connection.hpp"

namespace switcher {

class VRPNSource : public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(VRPNSource);
  VRPNSource(const std::string&);
  ~VRPNSource() = default;
  VRPNSource(const VRPNSource&) = delete;
  VRPNSource& operator=(const VRPNSource&) = delete;

 private:
  // PROPERTIES
  std::string host_{"localhost"};
  PContainer::prop_id_t host_id_;
  int port_{3883};
  PContainer::prop_id_t port_id_;
  PContainer::prop_id_t advanced_id_;
  bool debug_{false};
  PContainer::prop_id_t debug_id_;

  // SHMDATA
  std::unique_ptr<ShmdataWriter> shm_{nullptr};

  // SWITCHER
  bool init() final;
  bool start() final;
  bool stop() final;

  // VRPN
  static const unsigned int vrpnLoopInterval{16};
  std::unique_ptr<VRPNClientConnection> connection_{};
  static int handle_message(void* userData, vrpn_HANDLERPARAM p);
  std::unique_ptr<PeriodicTask> loopTask_{};
  void loop();
};

SWITCHER_DECLARE_PLUGIN(VRPNSource);

}  // namespace switcher
#endif
