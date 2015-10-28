/*
 * This file is part of switcher-nvenc.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef __SWITCHER_NVENC_API_H__
#define __SWITCHER_NVENC_API_H__

#include <nvEncodeAPI.h>
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {

class NVencAPI: public SafeBoolIdiom {
 public:
  NVencAPI();
  NVencAPI(const NVencAPI &) = delete;
  NVencAPI(NVencAPI &&) = delete;
  NVencAPI &operator=(const NVencAPI &) = delete;
  NVencAPI &operator=(NVencAPI &&) = delete;
  
 private:
  static NV_ENCODE_API_FUNCTION_LIST nvenc_api;
  static bool nvenc_api_has_been_set_;
  bool safe_bool_idiom() const {return nvenc_api_has_been_set_;}
};

}  // namespace switcher
#endif
