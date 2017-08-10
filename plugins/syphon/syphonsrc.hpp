/*
 * This file is part of syphonsrc.
 *
 * switcher-top is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_SYPHONSRC_H__
#define __SWITCHER_SYPHONSRC_H__

#include <memory>
#include <string>
#include "./syphonreader.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

namespace switcher {
class SyphonSrc : public Quiddity, public StartableQuiddity {
 public:
  SyphonSrc(QuiddityConfiguration&&);
  ~SyphonSrc() = default;
  SyphonSrc(const SyphonSrc&) = delete;
  SyphonSrc& operator=(const SyphonSrc&) = delete;

 private:
  std::shared_ptr<SyphonReader> reader_{};
  std::unique_ptr<ShmdataWriter> writer_{nullptr};
  std::string syphon_servername_{""};
  std::string syphon_appname_{""};
  int width_{0}, height_{0};

  bool start() final;
  bool stop() final;
  static void frameCallback(void*, const char*, int&, int&);
};

SWITCHER_DECLARE_PLUGIN(SyphonSrc);
}  // namespace switcher
#endif
