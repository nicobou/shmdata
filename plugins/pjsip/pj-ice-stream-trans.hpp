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
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <utility>
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {
class PJICEStreamTrans : public SafeBoolIdiom {
 public:
  using on_data_cb_t = std::function<void(void *data, size_t size)>;
  PJICEStreamTrans() = delete;
  PJICEStreamTrans(pj_ice_strans_cfg &ice_cfg,
                   unsigned comp_cnt,  // number of wanted UDP channels
                   pj_ice_sess_role role);  
  ~PJICEStreamTrans();
  PJICEStreamTrans(const PJICEStreamTrans &) = delete;
  PJICEStreamTrans &operator=(const PJICEStreamTrans &) = delete;

  // role CONTROLLED:
  std::pair<pj_str_t,  pj_str_t> get_ufrag_and_passwd();
  std::vector<std::vector<std::string>> get_components();
  bool set_data_cb(unsigned comp_id, on_data_cb_t cb);
  // role CONTROLLING:
  bool start_nego(const pj_str_t *rem_ufrag,
                  const pj_str_t *rem_passwd,
                  unsigned rcand_cnt,
                  const pj_ice_sess_cand rcand[]);
  bool sendto(unsigned comp_id,
              const void *data,
              pj_size_t data_len,
              const pj_sockaddr_t *dst_addr,
              int dst_addr_len);
  
 private:
  bool is_valid_{false};
  pj_ice_strans	*icest_{nullptr};
  unsigned comp_cnt_;
  // candidates are ready to be used
  bool cand_ready_{false};
  std::mutex cand_ready_mtx_{};
  std::condition_variable cand_ready_cv_{};
  std::vector<on_data_cb_t> data_cbs_;
  
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
