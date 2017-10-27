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
#include <json-glib/json-glib.h>
#include "./scope-exit.hpp"

namespace switcher {
namespace JSONSerializer {

void write_typed_member(JsonBuilder* builder, const Any& value) {
  switch (value.get_category()) {
    case AnyCategory::BOOLEAN:
      json_builder_add_boolean_value(builder, value.copy_as<bool>());
      break;
    case AnyCategory::INTEGRAL:
      json_builder_add_int_value(builder, value.copy_as<gint64>());
      break;
    case AnyCategory::FLOATING_POINT:
      json_builder_add_double_value(builder, value.copy_as<gdouble>());
      break;
    case AnyCategory::OTHER:
      json_builder_add_string_value(builder, Any::to_string(value).c_str());
      break;
    case AnyCategory::NONE:
      json_builder_add_null_value(builder);
      break;
  }
}

void on_visiting_node(std::string key,
                      InfoTree::ptrc node,
                      bool is_array_element,
                      JsonBuilder* builder) {
  key = InfoTree::unescape_dots(key);
  if (!is_array_element)  // discarding here to get it as a member called "name"
    json_builder_set_member_name(builder, key.c_str());

  if (!node) {
    json_builder_add_null_value(builder);
    return;
  }

  if (node->is_leaf()) {
    if (!node->read_data().is_null()) {
      write_typed_member(builder, node->read_data());
    } else {
      if (node->is_array()) {
        json_builder_begin_array(builder);
      } else {
        json_builder_add_null_value(builder);
      }
    }
    return;
  } else {  // adding node value with the key "key_value" along with other
            // childrens
    if (node->is_array()) {
      json_builder_begin_array(builder);
      // json_builder_begin_object (builder);
    } else {
      json_builder_begin_object(builder);
      if (is_array_element) {
        json_builder_set_member_name(builder, "id");
        json_builder_add_string_value(builder, key.c_str());
      }
      const Any& value = node->read_data();
      if (value.not_null()) {
        json_builder_set_member_name(builder, "key_value");
        write_typed_member(builder, value);
      }
    }
  }
}

void on_node_visited(std::string,
                     InfoTree::ptrc node,
                     bool /*is_array_element*/,
                     JsonBuilder* builder) {
  if (!node) return;
  if (node->is_array()) {
    // json_builder_end_object (builder);
    json_builder_end_array(builder);
    return;
  }
  if (!node->is_leaf()) json_builder_end_object(builder);
}

std::string serialize(InfoTree::ptrc tree) {
  JsonBuilder* json_builder = json_builder_new();
  On_scope_exit { g_object_unref(json_builder); };
  if (tree->is_leaf()) {
    if (!tree->read_data().is_null()) {
      switch (tree->read_data().get_category()) {
        case AnyCategory::BOOLEAN:
          return std::string(tree->read_data().copy_as<bool>() ? "true" : "false");
          break;
        case AnyCategory::INTEGRAL:
          return std::string(Any::to_string(tree->read_data()));
          break;
        case AnyCategory::FLOATING_POINT:
          return std::string(Any::to_string(tree->read_data()));
          break;
        case AnyCategory::OTHER:
          // We tried to get known types but sometimes values are of type OTHER
          // because they were created as strings
          return std::string("\"" + Any::to_string(tree->read_data()) + "\"");
          break;
        case AnyCategory::NONE:
          return std::string("null");
          break;
      }
    } else if (tree->is_array()) {
      return std::string("[]");
    } else {
      return std::string("null");
    }
  }
  bool is_array = tree->is_array();
  if (!is_array)
    json_builder_begin_object(json_builder);
  else
    json_builder_begin_array(json_builder);
  InfoTree::preorder_tree_walk(
      tree,
      [&json_builder](std::string key, InfoTree::ptrc node, bool is_array_element) {
        JSONSerializer::on_visiting_node(key, node, is_array_element, json_builder);
      },
      [&json_builder](std::string key, InfoTree::ptrc node, bool is_array_element) {
        JSONSerializer::on_node_visited(key, node, is_array_element, json_builder);
      });

  if (!is_array)
    json_builder_end_object(json_builder);
  else
    json_builder_end_array(json_builder);
  JsonNode* node = json_builder_get_root(json_builder);
  On_scope_exit { json_node_free(node); };
  if (nullptr == node) return std::string();
  JsonGenerator* generator = json_generator_new();
  On_scope_exit { g_object_unref(generator); };
  json_generator_set_pretty(generator, TRUE);
  json_generator_set_root(generator, node);
  gsize length = 0;
  gchar* data = json_generator_to_data(generator, &length);
  On_scope_exit { g_free(data); };
  std::string result(data);
  return result;
}

void add_json_node(InfoTree::rptr tree, JsonReader* reader) {
  if (json_reader_is_array(reader)) {
    for (gint i = 0; i < json_reader_count_elements(reader); ++i) {
      auto index = std::to_string(i);
      json_reader_read_element(reader, i);
      if (!tree->graft(index, InfoTree::make())) {
        return;
      }
      add_json_node(tree->get_tree(index).get(), reader);
      json_reader_end_element(reader);
    }
    tree->make_array(true);
  } else if (json_reader_is_object(reader)) {
    auto members = json_reader_list_members(reader);
    int nb_members = json_reader_count_members(reader);
    for (int member_index = 0; member_index < nb_members; ++member_index) {
      auto current_member = members[member_index];
      json_reader_read_member(reader, current_member);
      if (!tree->graft(current_member, InfoTree::make())) {  // FIXME do escape
        break;
      }
      add_json_node(tree->get_tree(current_member).get(), reader);
      json_reader_end_member(reader);
    }
  } else if (json_reader_is_value(reader)) {
    auto val = json_reader_get_null_value(reader) ? nullptr : json_reader_get_value(reader);
    if (!val || json_node_is_null(val)) {
      return;
    } else if (G_TYPE_INT64 == json_node_get_value_type(val)) {
      tree->set_value(json_node_get_int(val));
    } else if (G_TYPE_BOOLEAN == json_node_get_value_type(val)) {
      tree->set_value(static_cast<bool>(json_node_get_boolean(val)));
    } else if (G_TYPE_DOUBLE == json_node_get_value_type(val)) {
      tree->set_value(json_node_get_double(val));
    } else if (G_TYPE_STRING == json_node_get_value_type(val)) {
      tree->set_value(json_node_get_string(val));
    }
  }
}

InfoTree::ptr deserialize(const std::string& serialized) {
  auto res = InfoTree::make();

  JsonParser* parser = json_parser_new();
  On_scope_exit { g_object_unref(parser); };
  GError* error = nullptr;
  json_parser_load_from_data(parser, serialized.c_str(), serialized.size(), &error);
  if (error != nullptr) {
    g_error_free(error);
    return InfoTree::ptr();
  }
  JsonNode* root_node = json_parser_get_root(parser);
  if (!root_node) {
    return res;
  }
  JsonReader* reader = json_reader_new(root_node);
  if (!reader) {
    return res;
  }
  On_scope_exit { g_object_unref(reader); };

  add_json_node(res.get(), reader);

  return res;
}

}  // namespace JSONSerializer
}  // namespace switcher
