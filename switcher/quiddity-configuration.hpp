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

#include "./base-logger.hpp"
#include "./information-tree.hpp"

namespace switcher {
class QuiddityContainer;

struct QuiddityConfiguration {
  QuiddityConfiguration() = delete;
  QuiddityConfiguration(const std::string& name,
                        const std::string& type,
                        const InfoTree::ptr& tree_config,
                        QuiddityContainer* qc,
                        BaseLogger* log)
      : name_(name), type_(type), tree_config_(tree_config), qc_(qc), log_(log) {}
  std::string name_;
  std::string type_;
  InfoTree::ptr tree_config_;
  QuiddityContainer* qc_;
  BaseLogger* log_;
};

}  // namespace switcher
#endif
