/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_QUIDDITY_CONFIGURATION_H__
#define __SWITCHER_QUIDDITY_CONFIGURATION_H__

#include <string>

#include "../infotree/information-tree.hpp"
#include "../logger/base.hpp"
#include "./quid-id-t.hpp"

namespace switcher {
namespace quiddity {
class Container;
struct Config {
  Config() = delete;
  Config(qid_t id,
         const std::string& nickname,
         const std::string& kind,
         const InfoTree::ptrc tree_config,
         Container* qc,
         log::Base* log)
      : id_(id), nickname_(nickname), kind_(kind), tree_config_(tree_config), qc_(qc), log_(log) {}
  qid_t id_;
  std::string nickname_;
  std::string kind_;
  InfoTree::ptrc tree_config_;
  Container* qc_;
  log::Base* log_;
};

}  // namespace quiddity
}  // namespace switcher
#endif
