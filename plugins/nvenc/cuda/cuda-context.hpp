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

#ifndef __SWITCHER_CUDA_CONTEXT_H__
#define __SWITCHER_CUDA_CONTEXT_H__

#include <cuda.h>
#include <cstdint>  // uint32_t
#include <string>
#include <utility>
#include <vector>
#include "switcher/logged.hpp"
#include "switcher/safe-bool-idiom.hpp"

namespace switcher {

class NVencES;
// Creates a new CUDA context and associates it with the calling thread:
class CudaContext : public Logged, public SafeBoolIdiom {
  friend NVencES;

 public:
  CudaContext(uint32_t device_id, BaseLogger* log);
  CudaContext(BaseLogger* log) : CudaContext(0, log) {}
  CudaContext() = delete;
  ~CudaContext();
  CudaContext(const CudaContext&) = delete;
  CudaContext(CudaContext&&) = delete;
  CudaContext& operator=(const CudaContext&) = delete;
  CudaContext& operator=(CudaContext&&) = delete;

  static std::vector<std::pair<int, std::string>> get_devices();

 private:
  CUdevice cuda_dev_{-1};
  CUcontext cuda_ctx_{nullptr};
  bool safe_bool_idiom() const { return nullptr != cuda_ctx_; }
};

}  // namespace switcher
#endif
