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

#ifndef __SWITCHER_VRPN_SINK_H__
#define __SWITCHER_VRPN_SINK_H__

#include <chrono>
#include <iomanip>
#include <memory>
#include <switcher/periodic-task.hpp>
#include <switcher/quiddity.hpp>
#include <switcher/shmdata-connector.hpp>
#include <switcher/shmdata-follower.hpp>
#include <switcher/startable-quiddity.hpp>
#include "./devices/analog-sink-device.hpp"
#include "./devices/button-sink-device.hpp"
#include "./devices/sink-device.hpp"
#include "shared/connection/vrpn-server-connection.hpp"

namespace switcher {
namespace vrpn {

class VRPNSink : public Quiddity, public StartableQuiddity {
 public:
  VRPNSink(QuiddityConfiguration&&);
  ~VRPNSink() = default;
  VRPNSink(const VRPNSink&) = delete;
  VRPNSink& operator=(const VRPNSink&) = delete;

 private:
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
  void on_loaded() final;
  // This is needed so that we ignore methods while loading a save file
  bool loading_{false};

  // SHMDATA
  ShmdataConnector shmdataConnector_;
  std::unique_ptr<ShmdataFollower> shmDataFollower_{nullptr};
  bool connect(const std::string& path);
  bool disconnect();
  bool canSinkCaps(const std::string& caps);
  void onShmReaderData(void* data, size_t data_size);

  // VRPN
  std::mutex vrpnMutex_{};
  static const unsigned int vrpnLoopInterval;
  std::unique_ptr<PeriodicTask<>> loopTask_{};
  std::map<std::string, std::unique_ptr<SinkDevice>> devices_{};
  void loop();

  /**
   * Device properties
   * Map of vector of property ids per device id
   */
  std::map<std::string, std::unique_ptr<std::vector<PContainer::prop_id_t>>> devicesProperties_{};

  // ANALOG DEVICE
  static bool createAnalogDeviceMethod(gchar* deviceName, void* user_data);
  AnalogSinkDevice* getAnalogDevice(const std::string& deviceId);
  bool createAnalogDevice(const std::string& deviceName);
  void updateAnalogProperties(const std::string& deviceName, int numChannels);

  // BUTTON DEVICE
  static bool createButtonDeviceMethod(gchar* method_name, void* user_data);
  ButtonSinkDevice* getButtonDevice(const std::string& deviceId);
  bool createButtonDevice(const std::string& deviceName);
  void updateButtonProperties(const std::string& deviceName, int numButtons);

  // Destroy connection last
  std::unique_ptr<VRPNServerConnection> connection_{};
};
}  // Namespace vrpn
}  // Namespace switcher
#endif
