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
 * The JSONBuilder class
 */

#include "json-builder.h"


namespace switcher
{

  JSONBuilder::JSONBuilder () :
    builder_ (json_builder_new ()),
    thread_safe_ ()
  {}    

  JSONBuilder::~JSONBuilder ()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    if (builder_ != nullptr && G_IS_OBJECT (builder_))
      g_object_unref (builder_);
  }


  void
  JSONBuilder::reset()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_reset (builder_);
  }

  void 
  JSONBuilder::begin_object ()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_begin_object (builder_);
  }

  void 
  JSONBuilder::end_object ()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_end_object (builder_);
  }
  
  void 
  JSONBuilder::begin_array ()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_begin_array (builder_);
  }

  void 
  JSONBuilder::add_string_value (const gchar *string_value)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_add_string_value (builder_, string_value);
  }

  void 
  JSONBuilder::add_double_value (gdouble double_value)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_add_double_value (builder_, double_value);
  }
  
  void 
  JSONBuilder::end_array ()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_end_array (builder_);
  }
  
  void 
  JSONBuilder::set_member_name (const gchar *member_name)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_set_member_name (builder_, member_name);
  }

  void 
  JSONBuilder::add_string_member (const gchar *member_name, const gchar *string_value)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_set_member_name (builder_, member_name);
    json_builder_add_string_value (builder_, string_value);
  }

  void 
  JSONBuilder::add_double_member (const gchar *member_name, gdouble double_value)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_set_member_name (builder_, member_name);
    json_builder_add_double_value (builder_, double_value);
  }

  void 
  JSONBuilder::add_int_member (const gchar *member_name, gint int_value)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_set_member_name (builder_, member_name);
    json_builder_add_int_value (builder_, int_value);
  }
  
  void 
  JSONBuilder::add_node_value (Node node_value)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_add_value (builder_, json_node_copy (node_value));
  }
  
  void 
  JSONBuilder::add_JsonNode_member (const gchar *member_name, JsonNode *JsonNode_value)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    json_builder_set_member_name (builder_, member_name);
    json_builder_add_value (builder_, JsonNode_value);
  }

  std::string 
  JSONBuilder::get_string (bool pretty)
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    JsonNode *node = json_builder_get_root (builder_);
    std::string res = get_string (node, pretty);
    json_node_free (node);
    return res;
  }
  
  std::string 
  JSONBuilder::get_string (Node root_node, bool pretty)
  {
    JsonGenerator *generator = json_generator_new ();
    gsize length = 0;
    if (pretty)
      json_generator_set_pretty (generator, TRUE);
    else
      json_generator_set_pretty (generator, FALSE);
    json_generator_set_root (generator, root_node);
    gchar *data = json_generator_to_data (generator, &length);
    std::string description (data);
    g_free (data);
    g_object_unref (generator);
    return description;
  }


  JsonNode *
  JSONBuilder::get_root ()
  {
    std::unique_lock<std::mutex> lock (thread_safe_);
    return json_builder_get_root (builder_);//FIXME this might be leaking
  }

  void 
  JSONBuilder::node_free (JsonNode *root_node)
  {
    json_node_free (root_node);
  }
}
