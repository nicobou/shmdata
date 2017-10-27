/*
 * This file is part of switcher-osc.
 *
 * switcher-osc is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_OSC_CTRL_SERVER_H__
#define __SWITCHER_OSC_CTRL_SERVER_H__

#include "lo/lo.h"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/startable-quiddity.hpp"

namespace switcher {
class OscToShmdata : public Quiddity, public StartableQuiddity {
 public:
  OscToShmdata(QuiddityConfiguration&&);
  ~OscToShmdata();
  OscToShmdata(const OscToShmdata&) = delete;
  OscToShmdata& operator=(const OscToShmdata&) = delete;

 private:
  gint port_;
  lo_server_thread osc_thread_{nullptr};
  std::unique_ptr<ShmdataWriter> shm_{nullptr};

  bool start() final;
  bool stop() final;
  static int osc_handler(
      const char* path, const char* types, lo_arg** argv, int argc, void* data, void* user_data);
  static void osc_error(int num, const char* msg, const char* path);
};

SWITCHER_DECLARE_PLUGIN(OscToShmdata);
}  // namespace switcher
#endif
