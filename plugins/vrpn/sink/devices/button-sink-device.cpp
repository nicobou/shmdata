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

#include "./button-sink-device.hpp"

namespace switcher {
namespace vrpn {

ButtonSinkDevice::ButtonSinkDevice(const std::string& name, int numButtons)
    : SinkDevice(name), numButtons_(numButtons) {}

void ButtonSinkDevice::start(VRPNConnection* connection) {
  device_ = std::make_unique<VRPNButtonServer>(name_.c_str(), connection->raw(), numButtons_);
}

void ButtonSinkDevice::stop() { device_.reset(nullptr); }

void ButtonSinkDevice::loop() {
  if (device_) {
    device_->mainloop();
  }
}

InfoTree::ptr ButtonSinkDevice::getTree() const {
  InfoTree::ptr tree = SinkDevice::getTree();
  tree->graft("type", InfoTree::make("vrpn_Button"));
  tree->graft("numButtons", InfoTree::make(numButtons_));
  return tree;
}

int ButtonSinkDevice::getNumButtons() const { return numButtons_; }

void ButtonSinkDevice::setNumButtons(int numButtons) {
  numButtons_ = numButtons;

  if (device_) {
    device_->setNumButtons(numButtons_);
  }
}

bool ButtonSinkDevice::getButton(int index) const {
  return device_ ? device_->getButtons()[index] != 0 : false;
}

void ButtonSinkDevice::setButton(int index, bool value) {
  if (device_) {
    device_->getButtons()[index] = static_cast<unsigned char>(value ? 1 : 0);
  }
}

}  // Namespace vrpn
}  // Namespace switcher
