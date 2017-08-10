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

#include "./cuda-context.hpp"
#include <glib.h>  // log
#include "./cu-res.hpp"
#include "switcher/scope-exit.hpp"

namespace switcher {

CudaContext::CudaContext(uint32_t device_id, BaseLogger* log) : Logged(log) {
  On_scope_exit {
    if (!safe_bool_idiom()) warning("cuda context creation failed");
  };
  if (!CuRes(cuInit(0))) return;
  int dev_count = 0;
  if (!CuRes(cuDeviceGetCount(&dev_count))) return;
  char name[256];
  int min = 0, maj = 0;
  CUdevice cdev = 0;
  if (CuRes(cuDeviceGet(&cdev, device_id)) && CuRes(cuDeviceGetName(name, sizeof(name), cdev)) &&
      CuRes(cuDeviceComputeCapability(&maj, &min, cdev)))
    cuda_dev_ = cdev;
  else
    return;

  if (!CuRes(cuCtxCreate(&cuda_ctx_, 0, cuda_dev_))) cuda_ctx_ = nullptr;
}

CudaContext::~CudaContext() {
  if (safe_bool_idiom() && !CuRes(cuCtxDestroy(cuda_ctx_)))
    warning("BUG! (destroying cuda context)");
}

std::vector<std::pair<int, std::string>> CudaContext::get_devices() {
  std::vector<std::pair<int, std::string>> res;
  if (!CuRes(cuInit(0))) return res;
  int dev_count = 0;
  if (!CuRes(cuDeviceGetCount(&dev_count))) return res;
  char name[256];
  int min = 0, maj = 0;
  CUdevice cdev = 0;
  for (int i = 0; i < dev_count; ++i) {
    if (CuRes(cuDeviceGet(&cdev, i)) && CuRes(cuDeviceGetName(name, sizeof(name), cdev)) &&
        CuRes(cuDeviceComputeCapability(&maj, &min, cdev))) {
      if (((maj << 4) + min) >= 0x30) res.push_back(std::make_pair(i, std::string(name)));
      // debug("GPU #% supports NVENC: % (%) (Compute SM %.%)",
      //       std::to_string(i),
      //       std::string((((maj << 4) + min) >= 0x30) ? "yes" : "no"),
      //       name,
      //       std::to_string(maj),
      //       std::to_string(min));
    }
  }
  return res;
}

}  // namespace switcher
