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

#include "protocol-reader.hpp"
#include "protocol-curl.hpp"
#include "protocol-osc.hpp"

namespace switcher {

const std::string ProtocolReader::kCurlProtocol{"curl"};
const std::string ProtocolReader::kOscProtocol{"osc"};

ProtocolReader::ProtocolReader(Quiddity* quid, const InfoTree* tree) {
  continuous_ = tree->branch_read_data<std::string>("continuous") == "true";
  if (!continuous_) return;

  auto emission_prop = quid->pmanage<MPtr(&PContainer::make_unsigned_int)>(
      "emission_period",
      [this](const unsigned int& val) {
        emission_period_ = val;
        set_emission_period(emission_period_);
        return true;
      },
      [this]() { return emission_period_; },
      "Emission period",
      "Duration between ticks for continuous commands (in ms)",
      emission_period_,
      50,
      10000);
  quid->pmanage<MPtr(&PContainer::set_to_current)>(emission_prop);
}

std::unique_ptr<ProtocolReader> ProtocolReader::create_protocol_reader(Quiddity* quid,
                                                                       const InfoTree* tree) {
  std::unique_ptr<ProtocolReader> reader;
  switch (get_protocol_from_json(tree)) {
    case ProtocolType::CURL:
      reader = std::make_unique<ProtocolCurl>(quid, tree);
      break;
    case ProtocolType::OSC:
      reader = std::make_unique<ProtocolOsc>(quid, tree);
      break;
    default:
      return nullptr;
  }

  if (!reader->safe_bool_idiom()) reader = nullptr;

  return reader;
}

void ProtocolReader::set_emission_period(const unsigned int& period) {
  ptask_ = std::make_unique<PeriodicTask<>>([this]() { continuous_emission_task(); },
                                            std::chrono::milliseconds(period));
}

void ProtocolReader::continuous_emission_task() {
  std::vector<std::string> bangs;
  std::lock_guard<std::mutex> lock(ptask_mutex_);
  for (auto& it : tasks_) {
    it.second.command();
    if (!it.second.continuous) bangs.push_back(it.first);
  }
  // Remove the command task if it is not continuous
  for (auto& bang : bangs) {
    tasks_.erase(bang);
  }
}

ProtocolReader::ProtocolType ProtocolReader::get_protocol_from_json(const InfoTree* tree) {
  auto protocol_type = tree->branch_get_value("protocol");

  if (protocol_type.is_null()) return ProtocolType::UNDEFINED;

  auto protocol_str = protocol_type.copy_as<std::string>();
  StringUtils::tolower(protocol_str);

  if (protocol_str == kCurlProtocol)
    return ProtocolType::CURL;
  else if (protocol_str == kOscProtocol)
    return ProtocolType::OSC;

  return ProtocolType::UNDEFINED;
}
}
