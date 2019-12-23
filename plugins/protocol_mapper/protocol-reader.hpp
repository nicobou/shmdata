/*
 * This file is part of switcher-protocol-mapper.
 *
 * switcher-curl is free software; you can redistribute it and/or
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
#ifndef SWITCHER_PROTOCOL_READER_HPP
#define SWITCHER_PROTOCOL_READER_HPP

#include <map>
#include <switcher/utils/safe-bool-idiom.hpp>
#include "switcher/infotree/information-tree.hpp"
#include "switcher/quiddity/property/property-container.hpp"
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/utils/periodic-task.hpp"

namespace switcher {

class ProtocolReader : public SafeBoolIdiom {
 public:
  ProtocolReader(Quiddity* quid, const InfoTree* tree);
  static std::unique_ptr<ProtocolReader> create_protocol_reader(Quiddity* quid,
                                                                const InfoTree* tree);
  virtual bool make_properties(Quiddity* quid, const InfoTree* tree) = 0;
  void set_emission_period(const unsigned int& period);

 protected:
  struct Command {
    Command(PeriodicTask<>::task_t task, bool cont) : command(task), continuous(cont) {}
    PeriodicTask<>::task_t command{};
    bool continuous{false};
  };
  bool continuous_{false};

  // Value of the properties (might change in the future with new property type).
  std::map<std::string, bool> vals_{};

  std::unique_ptr<PeriodicTask<>> ptask_{nullptr};
  std::mutex ptask_mutex_{};
  unsigned int emission_period_{1000};
  std::map<std::string, ProtocolReader::Command> tasks_{};

 private:
  enum class ProtocolType : uint8_t { CURL = 0, OSC, UNDEFINED };
  static const std::string kCurlProtocol;
  static const std::string kOscProtocol;

  void continuous_emission_task();

  // Must be redefined in the derived class
  virtual bool safe_bool_idiom() const { return false; };
  static ProtocolType get_protocol_from_json(const InfoTree* tree);
};
}

#endif
