/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
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
 * The JSON Builder allows for building json strings
 */

#ifndef __SWITCHER_JSON_BUILDER_H__
#define __SWITCHER_JSON_BUILDER_H__


#include <memory>
#include <string>
#include <json-glib/json-glib.h>
#include "glib.h" 

namespace switcher
{

  class JSONBuilder 
  {
  public:
    typedef std::shared_ptr< JSONBuilder > ptr;
    typedef JsonNode * Node;
    JSONBuilder ();
    ~JSONBuilder ();

    void reset();
    void begin_object ();
    void end_object ();
    void begin_array ();
    void add_string_value (const gchar *string_value);
    void add_node_value (Node node_value);
    void end_array ();
    void set_member_name (const gchar *member_name);
    void add_string_member (const gchar *member_name, const gchar *string_value);
    void add_JsonNode_member (const gchar *member_name, Node JsonNode_value);
 
    std::string get_string (bool pretty);
    static std::string get_string (Node root_node, bool pretty);
    Node get_root ();// call node free when done if not used with add_node_value
    static void node_free (Node root_node);
  private:
    JsonBuilder *builder_;
    
  };

} // end of namespace



#endif // ifndef
