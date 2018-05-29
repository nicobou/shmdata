/*
 * This file is part of switcher-resample.
 *
 * switcher-resample is free software: you can redistribute it and/or modify
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

#include <cassert>
#include <chrono>
#include <thread>
#include "switcher/quiddity-basic-test.hpp"
#include "switcher/switcher.hpp"

int main() {
  using namespace switcher;
  // generic switcher testing
  Switcher::ptr sw = Switcher::make_switcher("test_manager");
  sw->factory<MPtr(&quid::Factory::scan_dir)>("./");
  assert(test::full(sw, "resample"));

  // now testing a audio resampling pipeline
  // load the resamplebundle quiddity
  sw->conf<MPtr(&Configuration::from_file)>("./check_resample.json");
  auto qrox = sw->quids<MPtr(&quid::Container::create)>("resamplebundle", std::string());
  assert(qrox);
  assert(qrox.get()->prop<MPtr(&PContainer::set_str_str)>("started", "true"));

  auto frame_received_id = qrox.get()->prop<MPtr(&PContainer::get_id)>("dummy/frame-received");
  assert(0 != frame_received_id);

  // check an audio frame has been resampled
  qrox.get()->prop<MPtr(&PContainer::subscribe)>(frame_received_id, [&]() {
    if (qrox.get()->prop<MPtr(&PContainer::get<bool>)>(frame_received_id)) exit(0);  // success
  });

  using namespace std::chrono_literals;
  std::this_thread::sleep_for(3s);
  return 1;
}