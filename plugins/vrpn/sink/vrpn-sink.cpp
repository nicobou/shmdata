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

#include "./vrpn-sink.hpp"

namespace switcher {
namespace vrpn {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(VRPNSink,
                                     "vrpnsink",
                                     "VRPN Sink Server",
                                     "vrpn",
                                     "reader/device",
                                     "Plugin to create a local VRPN server from shmdata sources.",
                                     "LGPL",
                                     "François Ubald Brien");

// Have to define it here, otherwise symbol not found...
const unsigned int VRPNSink::vrpnLoopInterval{16};

VRPNSink::VRPNSink(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      shmdataConnector_(static_cast<Quiddity*>(this)) {
  init_startable(this);

  shmdataConnector_.install_connect_method(
      [this](const std::string& shmPath) { return this->connect(shmPath); },
      [this](const std::string&) { return this->disconnect(); },
      [this]() { return this->disconnect(); },
      [this](const std::string& caps) { return this->canSinkCaps(caps); },
      1);

  port_id_ = pmanage<MPtr(&PContainer::make_int)>("port",
                                                  [this](int val) {
                                                    port_ = val;
                                                    return true;
                                                  },
                                                  [this]() { return port_; },
                                                  "Port",
                                                  "Port that the VRPN server will listen to.",
                                                  port_,
                                                  1,
                                                  65536);

  pmanage<MPtr(&PContainer::make_group)>(
      "advanced", "Advanced configuration", "Advanced configuration");

  pmanage<MPtr(&PContainer::make_parented_bool)>("debug",
                                                 "advanced",
                                                 [this](bool val) {
                                                   debug_ = val;
                                                   return true;
                                                 },
                                                 [this]() { return debug_; },
                                                 "Debug",
                                                 "Debug values to console",
                                                 debug_);

  install_method(
      "Create Analog Device",
      "create_analog_device",
      "Create a custom analog device that will be sent to connected VRPN clients.",
      "Success or Failure",
      Method::make_arg_description("Device Name", "device_name", "Name of the device", nullptr),
      (Method::method_ptr)&createAnalogDeviceMethod,
      G_TYPE_BOOLEAN,
      Method::make_arg_type_description(G_TYPE_STRING, nullptr),
      this);

  install_method(
      "Create Button Device",
      "create_button_device",
      "Create a custom button device that will be sent to connected VRPN clients.",
      "Success or Failure",
      Method::make_arg_description("Device Name", "device_name", "Name of the device", nullptr),
      (Method::method_ptr)&createButtonDeviceMethod,
      G_TYPE_BOOLEAN,
      Method::make_arg_type_description(G_TYPE_STRING, nullptr),
      this);
}

/**
 * @thread switcher
 */
InfoTree::ptr VRPNSink::on_saving() {
  InfoTree::ptr devicesTree = InfoTree::make();

  {
    std::lock_guard<std::mutex> _(vrpnMutex_);
    unsigned int index = 0;
    for (auto& device : devices_) {
      devicesTree->graft(std::to_string(index), device.second->getTree());
      ++index;
    }
    devicesTree->make_array(true);
  }

  InfoTree::ptr customData = InfoTree::make();
  customData->graft("devices", devicesTree);
  return customData;
}

/**
 * @thread switcher
 */
void VRPNSink::on_loading(InfoTree::ptr&& tree) {
  if (tree->empty()) {
    return;
  }

  std::lock_guard<std::mutex> _(vrpnMutex_);

  loading_ = true;
  InfoTree::ptr devicesTree = tree->get_tree(".devices");
  if (devicesTree->is_array()) {
    std::list<std::string> keys = devicesTree->get_child_keys(".");
    for (auto key : keys) {
      InfoTree::ptr deviceTree = devicesTree->get_tree(key);
      std::string type = deviceTree->branch_get_value("type");
      std::string name = deviceTree->branch_get_value("name");
      if (type == "vrpn_Analog") {
        createAnalogDevice(name);
        // No need to go into the number of channels, it'll be updated by the properties in the save
        // file
      } else if (type == "vrpn_Button") {
        createButtonDevice(name);
        // No need to go into the number of channels, it'll be updated by the properties in the save
        // file
      }
    }
  }
}

/**
 * @thread switcher
 */
void VRPNSink::on_loaded() { loading_ = false; }

/**
 * @thread switcher
 */
bool VRPNSink::createAnalogDeviceMethod(gchar* deviceName, void* user_data) {
  VRPNSink* context = static_cast<VRPNSink*>(user_data);
  if (context->loading_) {
    return false;
  }

  return context->createAnalogDevice(std::string(deviceName));
}

/**
 * @thread switcher
 */
bool VRPNSink::createAnalogDevice(const std::string& deviceName) {
  if (deviceName.empty()) {
    message("ERROR: Device name is required.");
    return false;
  }

  // Sync with vrpn thread before accessing connection/devices
  std::lock_guard<std::mutex> _(vrpnMutex_);

  // Check if we already have a device of the same name + type
  std::string deviceId = deviceName + "-analog";
  if (getAnalogDevice(deviceId) != nullptr) {
    message("ERROR: A device of the same type exists with that name.");
    return false;
  }

  // Create the device
  std::unique_ptr<AnalogSinkDevice> analogDevice =
      std::make_unique<AnalogSinkDevice>(deviceName.c_str(), 0);

  devicesProperties_[deviceId] = std::make_unique<std::vector<PContainer::prop_id_t>>();

  // Create the device property group
  std::string groupName = std::string(deviceName) + "-analog";
  pmanage<MPtr(&PContainer::make_group)>(
      groupName, "\"" + std::string(deviceName) + "\" Analog Device", "Analog Device Properties");

  // Add a property to the group, controlling the number of channels
  pmanage<MPtr(&PContainer::make_parented_int)>(groupName + "-numChannels",
                                                groupName,
                                                [this, deviceName](int val) {
                                                  // Called from switcher's thread
                                                  // Sync with vrpn thread before accessing
                                                  // connection/devices
                                                  std::lock_guard<std::mutex> _(vrpnMutex_);
                                                  this->updateAnalogProperties(deviceName, val);
                                                  return true;
                                                },
                                                [this, deviceId]() {
                                                  // Called from switcher's thread
                                                  // Sync with vrpn thread before accessing
                                                  // connection/devices
                                                  std::lock_guard<std::mutex> _(vrpnMutex_);
                                                  AnalogSinkDevice* analog =
                                                      getAnalogDevice(deviceId);
                                                  return analog ? analog->getNumChannels() : 0;
                                                },
                                                "Number of channels",
                                                "Number of allocated channels in the device.",
                                                0,
                                                0,
                                                vrpn_CHANNEL_MAX);

  // Create a container for the properties
  pmanage<MPtr(&PContainer::make_parented_group)>(
      groupName + "-channels", groupName, "Channels", "Channel Properties");

  // Start the device if we are already started
  if (is_started()) {
    analogDevice->start(connection_.get());
  }

  devices_.emplace(deviceId, std::move(analogDevice));

  return true;
}

/**
 * @thread switcher
 * @unsafe
 */
AnalogSinkDevice* VRPNSink::getAnalogDevice(const std::string& deviceId) {
  auto search = devices_.find(deviceId);
  if (search != devices_.end()) {
    return dynamic_cast<AnalogSinkDevice*>(search->second.get());
  } else {
    return nullptr;
  }
}

/**
 * @thread switcher
 * @unsafe
 */
void VRPNSink::updateAnalogProperties(const std::string& deviceName, int numChannels) {
  std::string deviceId = deviceName + "-analog";
  AnalogSinkDevice* analog = getAnalogDevice(deviceId);
  if (analog == nullptr) {
    return;
  }

  // Set VRPN's device channel count
  analog->setNumChannels(numChannels);

  // Find and update the properties if necessary
  auto search = devicesProperties_.find(deviceId);
  if (search != devicesProperties_.end()) {
    std::vector<PContainer::prop_id_t>* props = search->second.get();
    int initialSize = (int)props->size();

    if (numChannels > initialSize) {
      // Add missing properties
      for (int i = initialSize; i < numChannels; ++i) {
        auto property = pmanage<MPtr(&PContainer::make_parented_double)>(
            deviceName + "-analog-" + std::to_string(i),
            std::string(deviceName) + "-analog-channels",
            [this, deviceId, i](double val) {
              // Called from switcher's thread
              // Sync with vrpn thread before accessing connection/devices
              std::lock_guard<std::mutex> _(vrpnMutex_);
              AnalogSinkDevice* analog = getAnalogDevice(deviceId);
              if (analog) {
                analog->setChannel(i, val);
              }
              return analog != nullptr;
            },
            [this, deviceId, i]() {
              // Called from switcher's thread
              // Sync with vrpn thread before accessing connection/devices
              std::lock_guard<std::mutex> _(vrpnMutex_);
              AnalogSinkDevice* analog = getAnalogDevice(deviceId);
              return analog ? analog->getChannel(i) : 0;
            },
            deviceName + " Analog " + std::to_string(i),
            "Value of analog channel " + std::to_string(i) + " in " + deviceName,
            0,
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::max());
        props->push_back(property);
        if (!is_started()) {
          pmanage<MPtr(&PContainer::disable)>(property, StartableQuiddity::disabledWhenStopedMsg);
        }
      }
    } else if (numChannels < initialSize) {
      // Remove extra properties
      for (int i = initialSize; i > numChannels; --i) {
        pmanage<MPtr(&PContainer::remove)>(props->at((size_t)i - 1));
      }
      props->resize((size_t)numChannels);
    }
  }
}

/**
 * @thread switcher
 */
bool VRPNSink::createButtonDeviceMethod(gchar* deviceName, void* user_data) {
  VRPNSink* context = static_cast<VRPNSink*>(user_data);
  if (context->loading_) {
    return false;
  }

  return context->createButtonDevice(std::string(deviceName));
}

/**
 * @thread switcher
 */
bool VRPNSink::createButtonDevice(const std::string& deviceName) {
  if (deviceName.empty()) {
    message("ERROR: Device name is required.");
    return false;
  }

  // Sync with vrpn thread before accessing connection/devices
  std::lock_guard<std::mutex> _(vrpnMutex_);

  // Check if we already have a device of the same name + type
  std::string deviceId = deviceName + "-button";
  if (getButtonDevice(deviceId) != nullptr) {
    message("ERROR: A device of the same type exists with that name.");
    return false;
  }

  // Create the device
  std::unique_ptr<ButtonSinkDevice> buttonDevice =
      std::make_unique<ButtonSinkDevice>(deviceName.c_str(), 0);

  devicesProperties_[deviceId] = std::make_unique<std::vector<PContainer::prop_id_t>>();

  // Create the device property group
  std::string groupName = std::string(deviceName) + "-button";
  pmanage<MPtr(&PContainer::make_group)>(groupName,
                                         "\"" + std::string(deviceName) + "\" Button Device Button",
                                         "Button Device Properties");

  // Add a property to the group, controlling the number of channels
  pmanage<MPtr(&PContainer::make_parented_int)>(groupName + "-numChannels",
                                                groupName,
                                                [this, deviceName](int val) {
                                                  // Called from switcher's thread
                                                  // Sync with vrpn thread before accessing
                                                  // connection/devices
                                                  std::lock_guard<std::mutex> _(vrpnMutex_);
                                                  this->updateButtonProperties(deviceName, val);
                                                  return true;
                                                },
                                                [this, deviceId]() {
                                                  // Called from switcher's thread
                                                  // Sync with vrpn thread before accessing
                                                  // connection/devices
                                                  std::lock_guard<std::mutex> _(vrpnMutex_);
                                                  ButtonSinkDevice* button =
                                                      getButtonDevice(deviceId);
                                                  return button ? button->getNumButtons() : 0;
                                                },
                                                "Number of channels",
                                                "Number of allocated channels in the device.",
                                                0,
                                                0,
                                                vrpn_BUTTON_MAX_BUTTONS);

  // Create a container for the properties
  pmanage<MPtr(&PContainer::make_parented_group)>(
      groupName + "-buttons", groupName, "Buttons", "Button Properties");

  // Start the device if we are already started
  if (is_started()) {
    buttonDevice->start(connection_.get());
  }

  devices_.emplace(deviceId, std::move(buttonDevice));

  return true;
}

/**
 * @thread switcher
 * @unsafe
 */
ButtonSinkDevice* VRPNSink::getButtonDevice(const std::string& deviceId) {
  auto search = devices_.find(deviceId);
  if (search != devices_.end()) {
    return dynamic_cast<ButtonSinkDevice*>(search->second.get());
  } else {
    return nullptr;
  }
}

/**
 * @thread switcher
 * @unsafe
 */
void VRPNSink::updateButtonProperties(const std::string& deviceName, int numButtons) {
  std::string deviceId = deviceName + "-button";
  ButtonSinkDevice* button = getButtonDevice(deviceId);
  if (button == nullptr) {
    return;
  }

  // Set VRPN's device channel count
  button->setNumButtons(numButtons);

  // Find and update the properties if necessary
  auto search = devicesProperties_.find(deviceId);
  if (search != devicesProperties_.end()) {
    std::vector<PContainer::prop_id_t>* props = search->second.get();
    int initialSize = (int)props->size();

    if (numButtons > initialSize) {
      // Add missing properties
      for (int i = initialSize; i < numButtons; ++i) {
        auto property = pmanage<MPtr(&PContainer::make_parented_bool)>(
            deviceName + "-button-" + std::to_string(i),
            std::string(deviceName) + "-button-buttons",
            [this, deviceId, i](bool val) {
              // Called from switcher's thread
              // Sync with vrpn thread before accessing connection/devices
              std::lock_guard<std::mutex> _(vrpnMutex_);
              ButtonSinkDevice* button = getButtonDevice(deviceId);
              if (button) {
                button->setButton(i, val);
              }
              return button != nullptr;
            },
            [this, deviceId, i]() {
              // Called from switcher's thread
              // Sync with vrpn thread before accessing connection/devices
              std::lock_guard<std::mutex> _(vrpnMutex_);
              ButtonSinkDevice* button = getButtonDevice(deviceId);
              return button ? button->getButton(i) : false;
            },
            deviceName + " Button " + std::to_string(i),
            "Value of button channel " + std::to_string(i) + " in " + deviceName,
            false);
        props->push_back(property);
        if (!is_started()) {
          pmanage<MPtr(&PContainer::disable)>(property, StartableQuiddity::disabledWhenStopedMsg);
        }
      }
    } else if (numButtons < initialSize) {
      // Remove extra properties
      for (int i = initialSize; i > numButtons; --i) {
        pmanage<MPtr(&PContainer::remove)>(props->at((size_t)i - 1));
      }
      props->resize((size_t)numButtons);
    }
  }
}

/**
 * @thread switcher
 */
void VRPNSink::onShmReaderData(void* data, size_t size) {
  if (!is_started()) {
    return;
  }

  if (debug_) {
    unsigned char* d = static_cast<unsigned char*>(data);
    std::stringstream ss;
    ss << "VRPNSink   <<< ";
    for (int i = 0; i < (int)size; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << (int)d[i] << " ";
    }
    debug("%", ss.str());
  }

  // BUFFER
  char* data_ptr = static_cast<char*>(data);

  // SENDER
  uint32_t senderNameLength = ntohl(*((uint32_t*)data_ptr));
  data_ptr += sizeof(uint32_t);
  std::string senderName(data_ptr, senderNameLength);
  data_ptr += senderNameLength;

  // TYPE
  uint32_t typeNameLength = ntohl(*((uint32_t*)data_ptr));
  data_ptr += sizeof(uint32_t);
  std::string typeName(data_ptr, typeNameLength);
  data_ptr += typeNameLength;

  // TIME
  timeval time;
  time.tv_sec = ntohl(*((uint32_t*)data_ptr));
  data_ptr += sizeof(time.tv_sec);
  time.tv_usec = ntohl(*((uint32_t*)data_ptr));
  data_ptr += sizeof(time.tv_usec);

  // PACK MESSAGE
  uint32_t payload_len = ntohl(*((uint32_t*)data_ptr));
  data_ptr += sizeof(uint32_t);

  {
    std::lock_guard<std::mutex> _(vrpnMutex_);

    // All flags are ignored except vrpn_CONNECTION_RELIABLE for TCP,
    // otherwise use vrpn_CONNECTION_LOW_LATENCY for UDP
    connection_->raw()->pack_message(payload_len,
                                     time,
                                     connection_->raw()->register_message_type(typeName.c_str()),
                                     connection_->raw()->register_sender(senderName.c_str()),
                                     data_ptr,
                                     vrpn_CONNECTION_RELIABLE);
  }
}

/**
 * @thread switcher
 */
bool VRPNSink::start() {
  std::lock_guard<std::mutex> _(vrpnMutex_);

  connection_ = std::make_unique<VRPNServerConnection>(port_);
  if (!connection_->raw()->doing_okay()) {
    message("ERROR: VRPN sink connection is not doing okay.");
    return false;
  }

  pmanage<MPtr(&PContainer::disable)>(port_id_, disabledWhenStartedMsg);
  enable_method("create_analog_device");
  enable_method("create_button_device");

  for (auto& device : devices_) {
    device.second->start(connection_.get());
  }

  for (auto& deviceProperties : devicesProperties_) {
    for (auto property : *deviceProperties.second) {
      pmanage<MPtr(&PContainer::enable)>(property);
    }
  }

  loopTask_ = std::make_unique<PeriodicTask<>>([this]() { this->loop(); },
                                               std::chrono::milliseconds(vrpnLoopInterval));

  debug("Started VRPN sink server");

  return true;
}

/**
 * @thread switcher
 */
bool VRPNSink::stop() {
  debug("Stopping VRPN sink server");

  loopTask_.reset(nullptr);

  std::lock_guard<std::mutex> _(vrpnMutex_);
  for (auto& device : devices_) {
    device.second->stop();
  }
  connection_.reset(nullptr);

  pmanage<MPtr(&PContainer::enable)>(port_id_);
  for (auto& deviceProperties : devicesProperties_) {
    for (auto property : *deviceProperties.second) {
      pmanage<MPtr(&PContainer::disable)>(property, StartableQuiddity::disabledWhenStopedMsg);
    }
  }

  return true;
}

/**
 * @thread switcher
 */
bool VRPNSink::connect(const std::string& path) {
  shmDataFollower_.reset(new ShmdataFollower(
      this, path, [this](void* data, size_t size) { this->onShmReaderData(data, size); }));
  return true;
}

/**
 * @thread switcher
 */
bool VRPNSink::disconnect() {
  shmDataFollower_.reset(nullptr);
  return true;
}

/**
 * @thread switcher
 */
bool VRPNSink::canSinkCaps(const std::string& caps) { return (0 == caps.find("application/vrpn")); }

/**
 * @thread vrpn
 */
void VRPNSink::loop() {
  std::lock_guard<std::mutex> _(vrpnMutex_);

  if (!connection_->raw()->doing_okay()) {
    warning("VRPN sink connection is not doing okay.");
  }
  connection_->raw()->mainloop();
  for (auto& device : devices_) {
    device.second->loop();
  }
}
}  // Namespace vrpn
}  // Namespace switcher
