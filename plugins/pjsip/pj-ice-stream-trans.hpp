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

#ifndef __SWITCHER_PJ_ICE_STREAM_TRANS_H__
#define __SWITCHER_PJ_ICE_STREAM_TRANS_H__

#include <pjnath.h>
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {
class PJICEStreamTrans : public SafeBoolIdiom {
 public:
  PJICEStreamTrans() = delete;
  PJICEStreamTrans(pj_ice_strans_cfg &ice_cfg,
                   unsigned comp_cnt,  // number of wanted UDP channels
                   pj_ice_sess_role role);  
  ~PJICEStreamTrans();
  PJICEStreamTrans(const PJICEStreamTrans &) = delete;
  PJICEStreamTrans &operator=(const PJICEStreamTrans &) = delete;

 private:
  bool is_valid_{false};
  pj_ice_strans	*icest_{nullptr};
  unsigned comp_cnt_{1};

  bool safe_bool_idiom() const final {return is_valid_;}
  static void cb_on_rx_data(pj_ice_strans *ice_st,
                            unsigned comp_id, 
                            void *pkt, pj_size_t size,
                            const pj_sockaddr_t *src_addr,
                            unsigned src_addr_len);
  static void cb_on_ice_complete(pj_ice_strans *ice_st, 
                                 pj_ice_strans_op op,
                                 pj_status_t status);      
};

}  // namespace switcher
#endif
