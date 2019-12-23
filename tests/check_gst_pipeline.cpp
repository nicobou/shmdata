/*
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#undef NDEBUG  // get assert in release mode

#include "switcher/gst/glibmainloop.hpp"
#include "switcher/gst/gst-pipe.hpp"

int main() {
  using namespace switcher;

  gst_init(nullptr, nullptr);
  {
    auto ml = std::make_unique<GlibMainLoop>();
    auto gp = std::make_unique<GstPipe>(ml->get_main_context(), nullptr, nullptr);
  }
  gst_deinit();

  return 0;
}
