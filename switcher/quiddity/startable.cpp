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

#include "./startable.hpp"
#include "./quiddity.hpp"

namespace switcher {
namespace quiddity {

const std::string Startable::disabledWhenStartedMsg =
    "this property is disabled in started state";
const std::string Startable::disabledWhenStopedMsg =
    "this property is disabled in stopped state";

Startable::Startable(void* quiddity) { init_startable(quiddity); }

void Startable::init_startable(void* quiddity) {
  Quiddity* quid = static_cast<Quiddity*>(quiddity);
  quid->pmanage<MPtr(&property::PBag::make_bool)>("started",
                                                  [this](bool val) {
                                                    if (__started_ == val) return true;
                                                    if (val) {
                                                      if (!start()) return false;
                                                    } else {
                                                      if (!stop()) return false;
                                                    }
                                                    __started_ = val;
                                                    return true;
                                                  },
                                                  [this]() { return __started_; },
                                                  "Started",
                                                  "Start/stop the processing",
                                                  __started_);
}

bool Startable::is_started() const { return __started_; }

}  // namespace quiddity
}  // namespace switcher
