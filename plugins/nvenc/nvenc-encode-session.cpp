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

#include <glib.h>  // log
#include "./nvenc-encode-session.hpp"
#include "switcher/scope-exit.hpp"

namespace switcher {
NVencES::NVencES(uint32_t device_id):
    cu_ctx_(device_id){
  On_scope_exit{
    if (!safe_bool_idiom())
      g_warning("NV encoder session initialization failled");
  };
  if (!cu_ctx_)
    return;
  NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS params;
  params.version = NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER;
  params.apiVersion = NVENCAPI_VERSION;
  params.device = cu_ctx_.cuda_ctx_;
  params.deviceType = NV_ENC_DEVICE_TYPE_CUDA;
  if (NV_ENC_SUCCESS != NvEncOpenEncodeSessionEx (&params, &encoder_))
      encoder_ = nullptr;
}

NVencES::~NVencES(){
  if (safe_bool_idiom() && !(NV_ENC_SUCCESS == NvEncDestroyEncoder (encoder_)))
    g_warning("BUG! (destroying NV encoder session)");
}

}  // namespace switcher
