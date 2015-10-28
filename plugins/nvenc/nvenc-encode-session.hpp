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

#ifndef __SWITCHER_NVENC_ENCODE_SESSION_H__
#define __SWITCHER_NVENC_ENCODE_SESSION_H__

#include <cstdint>  // uint32_t
#include <nvEncodeAPI.h>
#include "switcher/safe-bool-idiom.hpp"
#include "./cuda-context.hpp"

namespace switcher {
class NVencES: public SafeBoolIdiom {
 public:
  NVencES(uint32_t device_id);
  NVencES(): NVencES (0){}
  ~NVencES();
  NVencES(const NVencES &) = delete;
  NVencES(NVencES &&) = delete;
  NVencES &operator=(const NVencES &) = delete;
  NVencES &operator=(NVencES &&) = delete;

 private:
  void *encoder_{nullptr};
  CudaContext cu_ctx_;
  bool safe_bool_idiom() const {return nullptr != encoder_;}
};

}  // namespace switcher
#endif
