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

#ifndef __SWITCHER_VRPN_SOURCE_DEVICE_PROPERTY_H__
#define __SWITCHER_VRPN_SOURCE_DEVICE_PROPERTY_H__

#include <string>
#include <typeinfo>
#include "./property-base.hpp"
#include "switcher/quiddity/property/property-container.hpp"

namespace switcher {
namespace quiddities {
namespace vrpn {

template <typename T>
class Property : public PropertyBase {
 public:
  /**
   * Property Constructor
   *
   * @param id
   * @param name
   * @param description
   * @param notifyProperty
   * @param value
   * @return
   */
  Property(T value) : value_(value) {}

  /**
   * VRPN property value getter
   */
  T value() { return value_; };

  /**
   * VRPN property value setter
   */
  void setValue(T value) { value_ = value; };

  /**
   * @see property-base.hpp
   */
  void noop() {}

 protected:
  /**
   * VRPN property value
   */
  T value_;
};
}  // namespace vrpn
}  // namespace quiddities
}  // namespace switcher
#endif
