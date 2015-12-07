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

#ifndef __SWITCHER_PJSIP_H__
#define __SWITCHER_PJSIP_H__

#include <pjsua-lib/pjsua.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {
class SIPPlugin;
class PJCall;
class PJPresence;
class PJStunTurn;

class PJSIP: public SafeBoolIdiom {
  friend SIPPlugin;
  friend PJCall;
  friend PJPresence;
  friend PJStunTurn;

 public:
  PJSIP(std::function<bool()> init_fun,
        std::function<void()> destruct_fun);
  ~PJSIP();
  PJSIP(const PJSIP &) = delete;
  PJSIP &operator=(const PJSIP &) = delete;
 private:
  bool is_valid_{false};
  pj_thread_desc thread_handler_desc_ {};
  pj_thread_t *pj_thread_ref_ {nullptr};
  pj_pool_t *pool_ {nullptr};
  pjsip_endpoint *sip_endpt_{nullptr};
  std::thread sip_worker_ {};
  bool sip_work_ {true};
  pj_thread_desc worker_handler_desc_ {};
  pj_thread_t *worker_thread_ref_ {nullptr};
  pj_caching_pool cp_;
  // singleton related members:
  bool i_m_the_one_{false};
  static std::atomic<unsigned short> sip_endpt_used_;
  static PJSIP *this_;
  std::function<void()> destruct_fun_;
  int log_level_{2};
  bool safe_bool_idiom() const final {return is_valid_;}
  void sip_worker_thread();
};

}  // namespace switcher
#endif
