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

#ifndef __SWITCHER_VRPN_SINK_H__
#define __SWITCHER_VRPN_SINK_H__

#include <chrono>
#include <memory>
#include "switcher/periodic-task.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/startable-quiddity.hpp"
#include "vrpn-server-connection.hpp"
#include "vrpn_Connection.h"
#include "vrpn_Shared.h"

namespace switcher {
class VRPNSink : public Quiddity, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(VRPNSink);
  VRPNSink(const std::string&);
  ~VRPNSink() = default;
  VRPNSink(const VRPNSink&) = delete;
  VRPNSink& operator=(const VRPNSink&) = delete;

 private:
  // PROPERTIES
  int port_{3883};
  PContainer::prop_id_t port_id_;
  PContainer::prop_id_t advanced_id_;
  bool debug_{false};
  PContainer::prop_id_t debug_id_;

  // SWITCHER
  bool init() final;
  bool start() final;
  bool stop() final;

  // SHMDATA
  ShmdataConnector shmcntr_;
  std::unique_ptr<ShmdataFollower> shm_{nullptr};
  bool connect(std::string path);
  bool disconnect();
  bool can_sink_caps(std::string caps);
  void on_shmreader_data(void* data, size_t data_size);

  // VRPN
  static const unsigned int vrpnLoopInterval{16};
  std::unique_ptr<VRPNServerConnection> connection_{};
  std::unique_ptr<PeriodicTask> loopTask_{};
  void loop();
};

SWITCHER_DECLARE_PLUGIN(VRPNSink);
}  // namespace switcher
#endif
