/**
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

#include <iomanip>
#include <memory>
#include <switcher/periodic-task.hpp>
#include <switcher/property-container.hpp>
#include <switcher/quiddity.hpp>
#include <switcher/shmdata-writer.hpp>
#include <switcher/startable-quiddity.hpp>
#include <switcher/threaded-wrapper.hpp>
#include "./devices/analog-source-device.hpp"
#include "./devices/button-source-device.hpp"
#include "./devices/source-device.hpp"
#include "./devices/tracker-source-device.hpp"
#include "shared/connection/vrpn-client-connection.hpp"

namespace switcher {
namespace vrpn {

class VRPNSource : public Quiddity, public StartableQuiddity {
 public:
  VRPNSource(QuiddityConfiguration&&);
  ~VRPNSource();
  VRPNSource(const VRPNSource&) = delete;
  VRPNSource& operator=(const VRPNSource&) = delete;

 private:
  /**
   * FIXME: Hack to stop handling messages when quiddity is being destroyed
   */
  bool destroyed_{false};

  /**
   * Host property value
   */
  std::string host_{"localhost"};

  /**
   * Host property id
   */
  PContainer::prop_id_t host_id_;

  /**
   * Port property value
   */
  int port_{3883};

  /**
   * Port property id
   */
  PContainer::prop_id_t port_id_;

  /**
   * Debug property value
   */
  bool debug_{false};

  // SWITCHER
  bool start() final;
  bool stop() final;

  // CUSTOM SAVE
  InfoTree::ptr on_saving() final;
  void on_loading(InfoTree::ptr&& tree) final;

  // SHMDATA
  std::unique_ptr<ShmdataWriter> shmDataWriter_{nullptr};

  // VRPN
  std::mutex vrpnMutex_{};
  std::unique_ptr<ThreadedWrapper<>> vrpnPropertyNotify_{std::make_unique<ThreadedWrapper<>>()};
  static const unsigned int vrpnLoopInterval;
  std::unique_ptr<PeriodicTask<>> loopTask_{};
  std::map<std::string, std::unique_ptr<SourceDevice>> devices_{};
  static int handleMessage(void* userData, vrpn_HANDLERPARAM p);
  void watchDevice(const std::string& senderName, const std::string& typeName);
  void createAnalogDevice(const std::string& id,
                          const std::string& name,
                          const std::string& uri,
                          int numChannels = 0);
  void createButtonDevice(const std::string& id,
                          const std::string& name,
                          const std::string& uri,
                          int numButtons = 0);
  void createTrackerDevice(const std::string& id, const std::string& name, const std::string& uri);
  void loop();

  // Destroy connection last
  std::unique_ptr<VRPNClientConnection> connection_{};
};
}  // Namespace vrpn
}  // Namespace switcher
#endif
