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

#undef NDEBUG  // get assert in release mode

#include <gst/gst.h>
#include <string>
#include <vector>
#include "switcher/switcher.hpp"

void property_cb(const std::string& /*subscriber_name */,
                 const std::string& /*quiddity_name */,
                 const std::string& /*property_name */,
                 const std::string& /*value */,
                 void* user_data) {
  uint* val = (uint*)user_data;
  *val = *val + 1;
}

int main() {
  uint count = 0;
  {
    switcher::Switcher::ptr mgr = switcher::Switcher::make_switcher("property_mapper");
    mgr->make_property_subscriber("sub", property_cb, &count);

    // map freq  property from audio1 to audio 2
    std::string audio1 = mgr->create("audiotestsrc", "audio1");
    if (!mgr->set_property(audio1, "started", "true")) return 1;
    std::string audio2 = mgr->create("audiotestsrc", "audio2");
    if (!mgr->set_property(audio2, "started", "true")) return 1;
    mgr->subscribe_property("sub", audio2.c_str(), "freq");
    std::string mapper = mgr->create("property-mapper", "mapper");
    if (!mgr->invoke_va(
            mapper.c_str(), "set-source-property", nullptr, audio1.c_str(), "freq", nullptr))
      return 1;
    if (!mgr->invoke_va(
            mapper.c_str(), "set-sink-property", nullptr, audio2.c_str(), "freq", nullptr))
      return 1;
    if (!mgr->set_property(audio1, "freq", "1000")) return 1;
    if (!mgr->set_property(audio1, "freq", "100")) return 1;
  }

  gst_deinit();
  if (2 == count)
    return 0;
  else
    return 1;
}
