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

#ifndef __SWITCHER_PJ_STUN_TURN_H__
#define __SWITCHER_PJ_STUN_TURN_H__

#include <mutex>
#include <condition_variable>

namespace switcher {
class PJSIP;
class PJCall;

class PJStunTurn {
  friend PJSIP;
  friend PJCall;

 public:
  PJStunTurn() = delete;
  explicit PJStunTurn(PJSIP *sip_instance);
  ~PJStunTurn();
  PJStunTurn(const PJStunTurn &) = delete;
  PJStunTurn &operator=(const PJStunTurn &) = delete;

 private:
  PJSIP *sip_instance_{nullptr};
  std::mutex connection_mutex_{};
  std::condition_variable connection_cond_{};
  bool connected_{false};
  pj_ice_strans_cfg ice_cfg_;
};

}  // namespace switcher
#endif
