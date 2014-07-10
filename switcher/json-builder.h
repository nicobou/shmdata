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
 * The JSON Builder allows for building json strings
 */

#ifndef __SWITCHER_JSON_BUILDER_H__
#define __SWITCHER_JSON_BUILDER_H__


#include <memory>
#include <mutex>
#include <string>
#include <json-glib/json-glib.h>
#include "glib.h" 

namespace switcher
{

  class JSONBuilder 
  {
  public:
    typedef std::shared_ptr< JSONBuilder > ptr;
    typedef JsonNode *Node;
    JSONBuilder ();
    ~JSONBuilder ();
    JSONBuilder (const JSONBuilder &source) = delete;
    JSONBuilder& operator= (const JSONBuilder &source) = delete;

    void reset();
    void begin_object ();
    void end_object ();
    void begin_array ();
    void add_string_value (const gchar *string_value);
    void add_double_value (gdouble double_value);
    void add_node_value (Node node_value);
    void end_array ();
    void set_member_name (const gchar *member_name);
    void add_string_member (const gchar *member_name, const gchar *string_value);
    void add_double_member (const gchar *member_name, gdouble double_value);
    void add_int_member (const gchar *member_name, gint int_value);
    void add_JsonNode_member (const gchar *member_name, Node JsonNode_value);
 
    std::string get_string (bool pretty);
    static std::string get_string (Node root_node, bool pretty);
    Node get_root ();// call node free when done if not used with add_node_value
    static void node_free (Node root_node);
  private:
    JsonBuilder *builder_;
    std::mutex thread_safe_;
    
  };

} // end of namespace



#endif // ifndef
