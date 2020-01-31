/*
 * This file is part of switcher-top.
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

#ifndef __SWITCHER_SYSTEM_USAGE_H__
#define __SWITCHER_SYSTEM_USAGE_H__

#include <memory>
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/quiddity/startable.hpp"
#include "switcher/utils/periodic-task.hpp"

namespace switcher {
namespace quiddities {
struct Cpu {
  long user{0};
  long nice{0};
  long system{0};
  long idle{0};
  long io{0};
  long irq{0};
  long softIrq{0};
  long steal{0};
  long guest{0};
  int totalTime{0};
};

struct Net {
  long rx_rate{0};
  long rx_bytes{0};
  long rx_packets{0};
  long rx_errors{0};
  long rx_drop{0};
  long tx_rate{0};
  long tx_bytes{0};
  long tx_packets{0};
  long tx_errors{0};
  long tx_drop{0};
};

using namespace quiddity;
class SystemUsage : public Quiddity {
 public:
  SystemUsage(quiddity::Config&&);
  ~SystemUsage() = default;
  SystemUsage(const SystemUsage&) = delete;
  SystemUsage& operator=(const SystemUsage&) = delete;

 private:
  InfoTree::ptr tree_;
  float period_;
  std::map<std::string, Cpu> _cpus{};
  std::map<std::string, Net> _net{};
  std::unique_ptr<PeriodicTask<>> pollStateTask_;

  bool init_tree();
  void pollState();
};

SWITCHER_DECLARE_PLUGIN(SystemUsage);
}  // namespace quiddities
}  // namespace switcher
#endif
