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

#include "analog-source-device.hpp"

namespace switcher {
namespace vrpn {

AnalogSourceDevice::AnalogSourceDevice(const std::string& name,
                                       const std::string& uri,
                                       int numProperties,
                                       CreateDoublePropertyCallback createDoubleProperty,
                                       SourceDevice::NotifyPropertyCallback notifyProperty)
    : SourceDevice(name, uri, notifyProperty), createDoubleProperty_(createDoubleProperty) {
  // Create the required number of channels
  // TODO: Could be optimized into calls to a createAnalogChannel method instead
  for (int i = 0; i < numProperties; ++i) {
    handleAnalogChannel(i, 0);
  }
}

void AnalogSourceDevice::start(VRPNConnection* connection) {
  std::unique_ptr<vrpn_Analog_Remote> analog =
      std::make_unique<vrpn_Analog_Remote>(uri_.c_str(), connection->raw());
  analog->register_change_handler(this, handleAnalogCallback);
  device_ = std::move(analog);
}

void AnalogSourceDevice::stop() { device_.reset(nullptr); }

InfoTree::ptr AnalogSourceDevice::getTree() const {
  InfoTree::ptr tree = SourceDevice::getTree();
  tree->graft("type", InfoTree::make("vrpn_Analog"));
  tree->graft("numChannels", InfoTree::make(properties_.size()));
  return tree;
}

void AnalogSourceDevice::handleAnalogChannel(int index, double value) {
  std::string id = name_ + "-analog-" + std::to_string(index);
  Property<double>* property = getProperty<double>(id);

  if (property == nullptr) {
    std::unique_ptr<Property<double>> newProperty = std::make_unique<Property<double>>(value);
    PContainer::prop_id_t prop_id =
        createDoubleProperty_(newProperty.get(),
                              id,
                              name_ + " Analog " + std::to_string(index),
                              "Value of analog channel " + std::to_string(index) + " in " + name_,
                              value,
                              std::numeric_limits<double>::lowest(),
                              std::numeric_limits<double>::max());
    newProperty->setPropId(prop_id);
    properties_[id] = std::move(newProperty);

  } else {
    property->setValue(value);
    notifyProperty_(property->getPropId());
  }
}

void AnalogSourceDevice::handleAnalogCallback(void* userData, const vrpn_ANALOGCB info) {
  auto* context = static_cast<AnalogSourceDevice*>(userData);
  for (int i = 0; i < info.num_channel; i++) {
    // Get out of the static method ASAP
    context->handleAnalogChannel(i, info.channel[i]);
  }
}
}  // Namespace vrpn
}  // Namespace switcher
