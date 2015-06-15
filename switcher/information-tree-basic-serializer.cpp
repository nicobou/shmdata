/*
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "./information-tree-basic-serializer.hpp"
#include <iostream>
#include <string>
#include <iterator>

namespace switcher {
namespace data {
namespace BasicSerializer {
typedef struct {
  std::list<std::string> path_ {};
  std::string result_ {};
} BasicSerializerData;

std::string path_to_string(std::list<std::string> path) {
  std::stringstream result;
  std::copy(path.begin(),
            path.end(),
            std::ostream_iterator<std::string> (result, "."));
  return result.str();
}

void
on_visiting_node(std::string key,
                 const Tree::ptrc node,
                 bool,
                 BasicSerializerData *data) {
  data->path_.push_back(key);
  auto value = node->read_data();
  if (value.not_null())
    data->result_.append("."
                         + BasicSerializer::path_to_string(data->path_)
                         + " "
                         + Any::to_string(value) + "\n");
}

void
on_node_visited(std::string,
                const Tree::ptrc, 
		bool, 
		BasicSerializerData *data) {
  data->path_.pop_back();
}

std::string serialize(Tree::ptrc tree) {
  BasicSerializerData data;
  Tree::preorder_tree_walk(tree,
                           std::bind(BasicSerializer::on_visiting_node,
                                     std::placeholders::_1,
                                     std::placeholders::_2,
                                     std::placeholders::_3,
                                     &data),
                           std::bind(BasicSerializer::on_node_visited,
                                     std::placeholders::_1,
                                     std::placeholders::_2,
                                     std::placeholders::_3, &data));
  return data.result_;
}

Tree::ptr deserialize(const std::string &serialized) {
  Tree::ptr tree = Tree::make();
  std::istringstream ss(serialized);
  std::string line;
  while (std::getline(ss, line)) {
    std::istringstream line_ss(line);
    std::string absolute_key;
    while (std::getline(line_ss, absolute_key, ' ')
           && absolute_key.empty()) {
    }
    std::string value;
    while (std::getline(line_ss, value, ' ') && value.empty()) {}
    // getting the rest of the value, if it contains space in it
    std::string val_cont;
    std::getline(line_ss, val_cont);
    if (!val_cont.empty())
      value += ' ' + val_cont;
    if (!absolute_key.empty() && !value.empty())
      tree->graft(absolute_key, Tree::make(value));
  }
  return tree;
}

}  // namespace BasicSerializer
}  // namespace data
}  // namespace switcher
