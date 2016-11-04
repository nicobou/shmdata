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

#include "button-source-device.hpp"

namespace switcher {
namespace vrpn {

ButtonSourceDevice::ButtonSourceDevice(const std::string& name,
                                       const std::string& uri,
                                       int numProperties,
                                       CreateBoolPropertyCallback createBoolProperty,
                                       SourceDevice::NotifyPropertyCallback notifyProperty)
    : SourceDevice(name, uri, notifyProperty), createBoolProperty_(createBoolProperty) {
  // Create the required number of buttons
  // TODO: Could be optimized into calls to a createButton method instead
  for (int i = 0; i < numProperties; ++i) {
    handleButton(i, false);
  }
}

void ButtonSourceDevice::start(VRPNConnection* connection) {
  std::unique_ptr<vrpn_Button_Remote> button =
      std::make_unique<vrpn_Button_Remote>(uri_.c_str(), connection->raw());
  button->register_change_handler(this, handleButtonCallback);
  device_ = std::move(button);
}

void ButtonSourceDevice::stop() { device_.reset(nullptr); }

InfoTree::ptr ButtonSourceDevice::getTree() const {
  InfoTree::ptr tree = SourceDevice::getTree();
  tree->graft("type", InfoTree::make("vrpn_Button"));
  tree->graft("numButtons", InfoTree::make(properties_.size()));
  return tree;
}

void ButtonSourceDevice::handleButton(int index, bool value) {
  std::string id = name_ + "-button-" + std::to_string(index);
  Property<bool>* property = getProperty<bool>(id);

  if (property == nullptr) {
    std::unique_ptr<Property<bool>> newProperty = std::make_unique<Property<bool>>(value);
    PContainer::prop_id_t prop_id =
        createBoolProperty_(newProperty.get(),
                            id,
                            name_ + " Button " + std::to_string(index),
                            "Value of button channel " + std::to_string(index) + " in " + name_,
                            value);
    newProperty->setPropId(prop_id);
    properties_[id] = std::move(newProperty);

  } else {
    property->setValue(value);
    notifyProperty_(property->getPropId());
  }
}

void ButtonSourceDevice::handleButtonCallback(void* userData, const vrpn_BUTTONCB info) {
  // Get out of the static method ASAP
  static_cast<ButtonSourceDevice*>(userData)->handleButton(info.button, info.state != 0);
}
}  // Namespace vrpn
}  // Namespace switcher
