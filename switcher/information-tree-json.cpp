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

#include "information-tree-json.h"
#include <json-glib/json-glib.h>
// #include <iostream>
// #include <string>
// #include <iterator>

namespace switcher { 
  namespace data {

     std::string 
     JSONSerializer::serialize (Tree::ptr tree)
     {
       JsonBuilder *json_builder = json_builder_new ();
       preorder_tree_walk<JsonBuilder *> (tree,
					  JSONSerializer::on_visiting_node,
					  JSONSerializer::on_node_visited,
					  json_builder);

       JsonNode *node = json_builder_get_root (json_builder);
       if (NULL == node)
	 {
	   g_object_unref (json_builder);
	   return std::string();
	 }
       JsonGenerator *generator = json_generator_new ();
       json_generator_set_pretty (generator, TRUE);
       json_generator_set_root (generator, node);
       gsize length = 0;
       gchar *data = json_generator_to_data (generator, &length);
       std::string result (data);
       g_free (data);
       g_object_unref (generator);
       g_object_unref (json_builder);
       return result;
     }

    void 
    JSONSerializer::on_visiting_node (std::string key, Any value, std::size_t n, JsonBuilder *builder)
    {
      if (1 < n)
	{ 
	  json_builder_begin_object (builder);
	  json_builder_set_member_name (builder, key.c_str ());
	  json_builder_begin_array (builder);
	  // if (value.not_null ())
	  //   {
	  //     json_builder_begin_object (builder);
	  //     json_builder_set_member_name (builder, "key_value");
	  //     json_builder_add_string_value (builder, Any::to_string (value).c_str ());
	  //     json_builder_end_object (builder);
	  //   }
	  return;
	}
      
      if (1 == n)
	{
	  json_builder_begin_object (builder);
	  json_builder_set_member_name (builder, key.c_str ());
	  //json_builder_add_string_value (builder, Any::to_string (value).c_str ());
	  return;
	}

      if (0 == n)
	{
	  json_builder_set_member_name (builder, key.c_str ());
	  json_builder_add_string_value (builder, Any::to_string (value).c_str ());
	  return;
	}
    }
    
    void 
    JSONSerializer::on_node_visited (std::string, Any, std::size_t n, JsonBuilder *builder)
    {
      if (1 < n)
	{
	  json_builder_end_array (builder);
	  json_builder_end_object (builder);
	  return;
	}
      if (1 == n)
	json_builder_end_object (builder);
    }
   
    // Tree::ptr 
    // JSONSerializer::deserialize (std::string &serialized)
    // {
     
    //   // JsonParser *parser = json_parser_new ();
    //   // GError *error = NULL;
    //   // json_parser_load_from_data (parser,
    //   // 				  serialized.c_str (),
    //   // 				  &error);
    //   // if (error != NULL)
    //   // 	{
    //   // 	  g_warning ("%s",error->message);
    //   // 	  g_object_unref(parser);
    //   // 	  g_error_free (error);
    //   // 	  return Tree::ptr ();
    //   // 	}
    //   return tree;
    // }
    
  } // end of "data" namespace 
}  // end of "switcher" namespace
