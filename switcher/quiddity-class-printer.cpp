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

#include "./quiddity-class-printer.hpp"
#include <string>

namespace switcher {
namespace QuiddityClassPrinter {

typedef struct {
  std::string result_{};
} Data;

bool on_visiting_node(std::string key, const InfoTree::ptrc node, bool, Data* data) {
  auto value = node->read_data();
  if (value.not_null()) {
    if (key == "class")
      data->result_.append("\n \n" + Any::to_string(value) + '\n');
    else if (key == "name" || key == "category" || key == "description" || key == "license" ||
             key == "author")
      data->result_.append(std::string("   ") + key + ": " + Any::to_string(value) + "\n");
  }
  return true;
}

bool on_node_visited(std::string, const InfoTree::ptrc, bool, Data*) { return true; }

std::string print(InfoTree::ptrc tree) {
  Data data;
  InfoTree::preorder_tree_walk(tree,
                               std::bind(QuiddityClassPrinter::on_visiting_node,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3,
                                         &data),
                               std::bind(QuiddityClassPrinter::on_node_visited,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         std::placeholders::_3,
                                         &data));
  return data.result_;
}

}  // namespace QuiddityClassPrinter
}  // namespace switcher
