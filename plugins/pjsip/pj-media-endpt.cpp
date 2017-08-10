/*
 * This file is part of switcher-pjsip.
 *
 * switcher-pjsip is free software: you can redistribute it and/or modify
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

#include "./pj-media-endpt.hpp"
#include <glib.h>
#include "./pj-sip.hpp"

namespace switcher {
pjmedia_endpt* PJMediaEndpt::med_endpt_ = nullptr;

PJMediaEndpt::PJMediaEndpt() {
  if (med_endpt_ != nullptr) g_error("BUG pjmedia_endpt is suposed to be a singleton");
  pj_status_t status = pjmedia_endpt_create(&PJSIP::this_->cp_.factory, nullptr, 1, &med_endpt_);
  if (status != PJ_SUCCESS) {
#ifdef DEBUG
    std::cerr << "Init media failed" << '\n';
#endif
  }
}

PJMediaEndpt::~PJMediaEndpt() {
  if (med_endpt_) {
    pjmedia_endpt_destroy(med_endpt_);
    med_endpt_ = nullptr;
  }
}

}  // namespace switcher
