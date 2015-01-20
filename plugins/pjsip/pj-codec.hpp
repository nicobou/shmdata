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

#ifndef __SWITCHER_PJCODEC_H__
#define __SWITCHER_PJCODEC_H__

// pjsip
#include <pjsip.h>
#include <pjmedia.h>
#include <pjmedia-codec.h>
#include <pjsip_ua.h>
#include <pjsip_simple.h>
#include <pjlib-util.h>
#include <pjlib.h>

namespace switcher {
class PJCall;

class PJCodec {
 public:
  PJCodec() = delete;
  ~PJCodec() = delete;
  PJCodec(const PJCodec &) = delete;
  PJCodec &operator=(const PJCodec &) = delete;

  static pj_status_t install_codecs();

 private:
  typedef struct alt_codec {
    pj_str_t encoding_name;
    pj_uint8_t payload_type;
    // unsigned clock_rate;
    // unsigned channel_cnt;
    // unsigned frm_ptime;
    // unsigned avg_bps;
    // unsigned max_bps;
  } alt_codec_t;

  typedef struct alt_codec_factory {
    pjmedia_codec_factory base;
  } alt_codec_factory_t;

 private:
  static alt_codec_t codec_list[];
  static alt_codec_factory_t alt_codec_factory;
  static pjmedia_codec_factory_op alt_codec_factory_op;
  static pj_status_t alt_codec_test_alloc(pjmedia_codec_factory *factory,
                                          const pjmedia_codec_info *id);
  static pj_status_t alt_codec_default_attr(pjmedia_codec_factory *
                                            factory,
                                            const pjmedia_codec_info *id,
                                            pjmedia_codec_param *attr);
  static pj_status_t alt_codec_enum_codecs(pjmedia_codec_factory *factory,
                                           unsigned *count,
                                           pjmedia_codec_info codecs[]);
  static pj_status_t alt_codec_alloc_codec(pjmedia_codec_factory *factory,
                                           const pjmedia_codec_info *id,
                                           pjmedia_codec ** p_codec);
  static pj_status_t alt_codec_dealloc_codec(pjmedia_codec_factory *
                                             factory,
                                             pjmedia_codec *codec);
  static pj_status_t alt_codec_deinit(void);
};
}  // namespace switcher
#endif
