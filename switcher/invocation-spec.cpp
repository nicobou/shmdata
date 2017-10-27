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

#include "./invocation-spec.hpp"
#include <glib.h>  // logs

namespace switcher {

void InvocationSpec::add_arg(std::string arg) { args_.push_back(arg); }

void InvocationSpec::set_vector_arg(std::vector<std::string> vector_arg) {
  vector_arg_ = vector_arg;
}

InfoTree::ptr InvocationSpec::get_info_tree() const {
  static std::string inv = "invoke";
  auto tree = InfoTree::make();
  tree->graft("command", InfoTree::make(inv));
  for (unsigned int i = 0; i < args_.size(); i++) {
    tree->graft(std::string("arguments.") + std::to_string(i), InfoTree::make(args_[i]));
  }
  tree->tag_as_array("arguments", true);
  if (vector_arg_.empty()) {
    tree->graft("vector argument.", InfoTree::make());
  } else {
    for (unsigned int i = 0; i < vector_arg_.size(); i++) {
      tree->graft(std::string("vector argument.") + std::to_string(i),
                  InfoTree::make(vector_arg_[i]));
    }
  }
  tree->tag_as_array("vector argument", true);
  if (result_.empty()) {
    tree->graft("results.0.", InfoTree::make(""));
  } else {
    for (unsigned int i = 0; i < result_.size(); i++) {
      tree->graft(std::string("results.") + std::to_string(i), InfoTree::make(result_[i]));
    }
  }
  tree->tag_as_array("results", true);
  return tree;
}

InvocationSpec InvocationSpec::get_invocation_spec_from_tree(InfoTree::ptr tree) {
  InvocationSpec invocation;
  std::string invocation_str = tree->branch_get_value("command");
#ifdef DEBUG
  assert(invocation_str == "invoke");
#endif
  {
    std::string args_str("arguments.");
    auto arguments = tree->get_child_keys(args_str);
    for (auto& it : arguments) {
      std::string arg = tree->branch_get_value(args_str + it);
      if (!arg.empty()) {
        invocation.add_arg(arg);
      } else {
        invocation.add_arg("null");
      }
    }
  }
  {
    std::string vect_str("vector argument.");
    auto arguments = tree->get_child_keys(vect_str);
    std::vector<std::string> string_vect_arg;
    for (auto& it : arguments) {
      std::string arg = tree->branch_get_value(vect_str + it);
      if (!arg.empty()) {
        string_vect_arg.push_back(arg);
      } else {
        string_vect_arg.push_back("null");
      }
    }
    invocation.set_vector_arg(string_vect_arg);
  }
  {
    std::string res_str("results.");
    auto results = tree->get_child_keys(res_str);
    std::vector<std::string> expected_result;
    for (auto& it : results) {
      std::string res = tree->branch_get_value(res_str + it);
      if (!res.empty()) expected_result.push_back(res);
    }
    invocation.expected_result_ = expected_result;
  }
  return invocation;
}

}  // namespace switcher
