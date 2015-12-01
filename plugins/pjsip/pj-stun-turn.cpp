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

#include <pjnath.h>
#include <glib.h>
#include "./pj-sip-plugin.hpp"
#include "./pj-stun-turn.hpp"

namespace switcher {
PJStunTurn::PJStunTurn(){
  if (PJ_SUCCESS != pjnath_init()){
    g_warning("cannot init pjnath");
    return;
  }
  pj_ice_strans_cfg_default(&ice_cfg_);
  pj_timer_heap_create(PJSIP::this_->pool_, 100, 
                       &ice_cfg_.stun_cfg.timer_heap);
  pj_ioqueue_create(PJSIP::this_->pool_, 128, 
                    &ice_cfg_.stun_cfg.ioqueue);
  ice_cfg_.af = pj_AF_INET();
  
}

PJStunTurn::~PJStunTurn() {
  if (ice_cfg_.stun_cfg.ioqueue)
    pj_ioqueue_destroy(ice_cfg_.stun_cfg.ioqueue);
  
  if (ice_cfg_.stun_cfg.timer_heap)
    pj_timer_heap_destroy(ice_cfg_.stun_cfg.timer_heap);
}
   
}  // namespace switcher
