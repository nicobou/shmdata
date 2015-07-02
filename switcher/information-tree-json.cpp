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

#include "./information-tree-json.hpp"
#include "./scope-exit.hpp"
#include <json-glib/json-glib.h>
#include <iostream>

namespace switcher {
namespace data {
namespace JSONSerializer {
void
on_visiting_node(std::string key,
                 Tree::ptrc node,
                 bool is_array_element,
                 JsonBuilder *builder) {
  key = data::Tree::unescape_dots(key);
  if (!is_array_element)  // discarding here to get it as a member called "name"
    json_builder_set_member_name(builder, key.c_str());
  if (node->is_leaf()){
    if (!node->read_data().is_null()) {
      json_builder_add_string_value(builder, Any::to_string(node->read_data()).c_str());
    } else {
      if(node->is_array()) {
        json_builder_begin_array(builder);
      } else {
        json_builder_add_null_value(builder);
      }
    }
    return;
  } else {  // adding node value with the key "key_value" along with other childrens
    if (node->is_array()) {
    json_builder_begin_array(builder);
      // json_builder_begin_object (builder);
    } else {
      json_builder_begin_object(builder);
      if (is_array_element) {
        json_builder_set_member_name(builder, "name");
        json_builder_add_string_value(builder, key.c_str());
      }
      const Any value = node->read_data();
      if (value.not_null()) {
        json_builder_set_member_name(builder, "key_value");
        json_builder_add_string_value(builder, Any::to_string(value).c_str());
      }
    }
  }
}

void
on_node_visited(std::string,
                Tree::ptrc node,
                bool /*is_array_element*/, 
		JsonBuilder *builder) {
  if (node->is_array()) {
    // json_builder_end_object (builder);
    json_builder_end_array(builder);
    return;
  }
  if (!node->is_leaf())
    json_builder_end_object(builder);
}

std::string serialize(Tree::ptrc tree) {
  JsonBuilder *json_builder = json_builder_new();
  On_scope_exit {
    g_object_unref(json_builder);
  };
  if (tree->is_leaf()){
    return std::string("\"" + Any::to_string(tree->read_data()) + "\"");
  }
  json_builder_begin_object(json_builder);
  Tree::preorder_tree_walk(tree,
                           std::bind(JSONSerializer::on_visiting_node,
                                     std::placeholders::_1,
                                     std::placeholders::_2,
                                     std::placeholders::_3,
                                     json_builder),
                           std::bind(JSONSerializer::on_node_visited,
                                     std::placeholders::_1,
                                     std::placeholders::_2,
                                     std::placeholders::_3, json_builder));
  json_builder_end_object(json_builder);
  JsonNode *node = json_builder_get_root(json_builder);
  On_scope_exit {json_node_free(node);};
  if (nullptr == node)
    return std::string();
  JsonGenerator *generator = json_generator_new();
  On_scope_exit {g_object_unref(generator);};
  json_generator_set_pretty(generator, TRUE);
  json_generator_set_root(generator, node);
  gsize length = 0;
  gchar *data = json_generator_to_data(generator, &length);
  On_scope_exit {g_free(data);};
  std::string result(data);
  return result;
}

// Tree::ptr
// deserialize (std::string &serialized)
// {
//   // JsonParser *parser = json_parser_new ();
//   // GError *error = nullptr;
//   // json_parser_load_from_data (parser,
//   //   serialized.c_str (),
//   //   &error);
//   // if (error != nullptr)
//   // {
//   //   g_warning ("%s",error->message);
//   //   g_object_unref(parser);
//   //   g_error_free (error);
//   //   return Tree::ptr ();
//   // }
//   return tree;
// }
}  // namespace JSONSerializer
}  // namespace data
}  // namespace switcher
