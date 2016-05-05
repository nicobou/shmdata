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

#include <glib.h>
#include <condition_variable>
#include <mutex>
#include "./pj-ice-stream-trans.hpp"

namespace switcher {
class SIPPlugin;
class PJCall;

class PJStunTurn {
  friend SIPPlugin;
  friend PJCall;

 public:
  PJStunTurn();
  ~PJStunTurn();
  PJStunTurn(const PJStunTurn&) = delete;
  PJStunTurn& operator=(const PJStunTurn&) = delete;

  std::unique_ptr<PJICEStreamTrans> get_ice_transport(unsigned comp_cnt,
                                                      pj_ice_sess_role role);

 private:
  std::mutex connection_mutex_{};
  std::condition_variable connection_cond_{};
  bool connected_{false};
  pj_ice_strans_cfg ice_cfg_;
  bool worker_quit_{false};
  pj_thread_t* thread_{nullptr};
  std::string stun_srv_;
  std::string turn_srv_;
  std::string turn_user_;
  std::string turn_pass_;
  bool stun_turn_valid_{false};

  static int worker_thread(void* data);
  pj_status_t handle_events(unsigned max_msec, unsigned* p_count);
  static gboolean set_stun_turn(gchar* stun,
                                gchar* turn,
                                gchar* turn_user,
                                char* turn_pass,
                                void* user_data);
};

}  // namespace switcher
#endif
