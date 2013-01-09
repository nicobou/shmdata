/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

/**
 * The JSONBuilder class
 */

#include "switcher/json-builder.h"


namespace switcher
{

  JSONBuilder::JSONBuilder ()
  {
    builder_ = json_builder_new ();
     // JsonBuilder *builder = json_builder_new ();
     // JsonNode *node;
     // JsonGenerator *generator;
     // gsize length;
     // gchar *data;
    
     // json_builder_begin_object (builder);
    
     // json_builder_set_member_name (builder, "depth1");
     // json_builder_begin_array (builder);
     // json_builder_add_int_value (builder, 1);
    
     // json_builder_begin_object (builder);
    
     // json_builder_set_member_name (builder, "depth2");
     // json_builder_begin_array (builder);
     // json_builder_add_int_value (builder, 3);
    
     // json_builder_begin_array (builder);
     // json_builder_add_null_value (builder);
     // json_builder_end_array (builder);
    
     // json_builder_add_string_value (builder, "after array");
     // json_builder_end_array (builder); /* depth2 */
    
     // json_builder_set_member_name (builder, "value2");
     // json_builder_add_boolean_value (builder, TRUE);
     // json_builder_end_object (builder);
    
     // json_builder_end_array (builder); /* depth1 */
    
     // json_builder_set_member_name (builder, "object1");
     // json_builder_begin_object (builder);
     // json_builder_end_object (builder);
    
     // json_builder_end_object (builder);
    
     // node = json_builder_get_root (builder);
     // g_object_unref (builder);
    
     // generator = json_generator_new ();
     // json_generator_set_root (generator, node);
     // data = json_generator_to_data (generator, &length);
    
     // g_print ("Builder complex: '%*s'\n", (int)length, data);
    
     // g_free (data);
     // json_node_free (node);
     // g_object_unref (generator);
  }    

  JSONBuilder::~JSONBuilder ()
  {
    g_object_unref (builder_);
  }

  void
  JSONBuilder::reset()
  {
    json_builder_reset (builder_);
  }

  void 
  JSONBuilder::begin_object ()
  {
    json_builder_begin_object (builder_);
  }

  void 
  JSONBuilder::end_object ()
  {
    json_builder_end_object (builder_);
  }
  
  void 
  JSONBuilder::begin_array ()
  {
    json_builder_begin_array (builder_);
  }

  void 
  JSONBuilder::add_string_value (const gchar *string_value)
  {
    json_builder_add_string_value (builder_, string_value);
  }
  
  void 
  JSONBuilder::end_array ()
  {
    json_builder_end_array (builder_);
  }
  
  void 
  JSONBuilder::set_member_name (const gchar *member_name)
  {
    json_builder_set_member_name (builder_, member_name);
  }


  void 
  JSONBuilder::add_string_member (const gchar *member_name, const gchar *string_value)
  {
    json_builder_set_member_name (builder_, member_name);
    json_builder_add_string_value (builder_, string_value);
  }

  std::string 
  JSONBuilder::get_string ()
  {
    JsonNode *node;
    JsonGenerator *generator;
    gsize length;
    gchar *data;
    
    node = json_builder_get_root (builder_);
    generator = json_generator_new ();
    json_generator_set_root (generator, node);
    data = json_generator_to_data (generator, &length);
    
    g_print ("%*s\n", (int)length, data);
    std::string description (data);
    
    g_free (data);
    json_node_free (node);
    g_object_unref (generator);

    return description;
  }

}
