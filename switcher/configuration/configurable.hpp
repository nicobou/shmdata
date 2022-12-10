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

#ifndef __SWITCHER_SWITCHER_CONFIGURATION_CONFIGURABLE_H__
#define __SWITCHER_SWITCHER_CONFIGURATION_CONFIGURABLE_H__

#include "./configuration.hpp"

namespace switcher {
namespace configuration {

/**
 * A wrapper class to be inherited by other classes (like Switcher and Quiddity)
 * in order to get a switcher::Configuration available.
 **/
class Configurable {
 public:
  Configurable() = delete;
  /**
   * Construct a Configurable
   *
   * \param on_reloaded A function to be called when a new configuration
   *        has been loaded (not called during construction)
   **/
  Configurable(std::function<void()> on_reloaded);

 protected:
  Configuration conf_;
};

}  // namespace configuration
}  // namespace switcher

#endif
