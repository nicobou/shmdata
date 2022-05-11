/*
 * This file is part of switcher-crashtest.
 *
 * switcher-top is free software; you can redistribute it and/or
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

#include "./crashtest.hpp"
#include "switcher/infotree/json-serializer.hpp"

using namespace std;

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(CrashTest,
                                     "crashtest",
                                     "CrashTest plugin",
                                     "Crash Switcher with a segfault when crash property is set true",
                                     "LGPL",
                                     "Olivier Gauthier");

CrashTest::CrashTest(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      crash_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "crash",
          [this](bool val) {
            crash_ = val;
            if (crash_) CrashTest::crash();
            return true;
          },
          [this]() { return crash_; },
          "Crash Switcher",
          "This property crashes Switcher when set true (SEGFAULT)",
          crash_)) {
  LOGGER_DEBUG(this->logger, "CrashTest::CrashTest created.");
}

void CrashTest::crash() {
  // Access null pointer to generate a segfault
  LOGGER_DEBUG(this->logger, "CrashTest::crashSwitcher method called.");
  int* p = nullptr;
  *p = 1;
}

}  // namespace quiddities
}  // namespace switcher
