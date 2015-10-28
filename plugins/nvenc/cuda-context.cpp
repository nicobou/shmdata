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
#include "./cuda-context.hpp"
#include "./cu-res.hpp"
#include "switcher/scope-exit.hpp"

namespace switcher {

CudaContext::CudaContext(uint32_t device_id){
  On_scope_exit{if (safe_bool_idiom()) g_warning("cuda context creation failled");};
  if(!CuRes(cuInit(0))) return;
  int dev_count = 0;
  if(!CuRes(cuDeviceGetCount(&dev_count))) return;
  char name[256];
  int min = 0, maj = 0;
  CUdevice cdev = 0;
  for (int i = 0; i < dev_count; ++i) {
    if (CuRes(cuDeviceGet (&cdev, i))
        && CuRes(cuDeviceGetName(name, sizeof (name), cdev))
        && CuRes(cuDeviceComputeCapability(&maj, &min, cdev))) {
      g_message("GPU #%d supports NVENC: %s (%s) (Compute SM %d.%d)",
                i, (((maj << 4) + min) >= 0x30) ? "yes" : "no", name, maj, min);
      if (i == static_cast<int>(device_id))
        cuda_dev_ = cdev;
    }
  }
  if (cuda_dev_ == -1)
    return;
  if (!CuRes(cuCtxCreate (&cuda_ctx_, 0, cuda_dev_)))
    cuda_ctx_ = nullptr;
}

CudaContext::~CudaContext(){
  if(safe_bool_idiom() && !CuRes(cuCtxDestroy(cuda_ctx_)))
    g_warning("BUG! (destroying cuda context)");
}

}  // namespace switcher
