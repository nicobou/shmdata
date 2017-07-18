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

/**
 * The invocation spec for invocations being saved into the manager history
 */

#ifndef __SWITCHER_INVOCATION_SPEC_H__
#define __SWITCHER_INVOCATION_SPEC_H__

#include <map>
#include <memory>
#include <string>
#include <vector>
#include "./information-tree.hpp"

namespace switcher {
class InvocationSpec {
 public:
  std::vector<std::string> args_{};
  std::vector<std::string> vector_arg_{};
  std::vector<std::string> result_{};
  std::vector<std::string> expected_result_{};
  bool success_{false};
  void add_arg(std::string arg);
  void set_vector_arg(std::vector<std::string> vector_arg);
  static InvocationSpec get_invocation_spec_from_tree(InfoTree::ptr tree);
  InfoTree::ptr get_info_tree() const;
};

}  // namespace switcher
#endif
