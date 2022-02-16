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

#ifndef __SWITCHER_CRASHTEST_PLUGIN_H__
#define __SWITCHER_CRASHTEST_PLUGIN_H__

#include <memory>
#include <string>

#include "switcher/quiddity/quiddity.hpp"

namespace switcher {
namespace quiddities {

using namespace quiddity;
class CrashTest : public Quiddity {
 public:
  CrashTest(quiddity::Config&&);
  ~CrashTest() = default;
  CrashTest(const CrashTest&) = delete;
  CrashTest& operator=(const CrashTest&) = delete;

 private:
  property::prop_id_t crash_id_;
  bool crash_{false};
  void crash();
};

SWITCHER_DECLARE_PLUGIN(CrashTest);

}  // namespace quiddities
}  // namespace switcher
#endif
