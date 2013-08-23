/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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




#ifndef __SWITCHER_METHOD_H__
#define __SWITCHER_METHOD_H__

#include <gst/gst.h>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <stdarg.h>
#include "json-builder.h"
#include <tuple>

namespace switcher
{
  
  class Method
  {
  public:
    typedef std::shared_ptr<Method> ptr;
    typedef GType return_type;
    typedef std::vector<GType> args_types;
    typedef std::vector<std::tuple<std::string,std::string,std::string> > args_doc;
    typedef void * method_ptr;

    Method ();
    ~Method ();
    bool set_method (method_ptr method,
		     return_type return_type,
		     args_types arg_types, 
		     gpointer user_data);
    GValue invoke (std::vector<std::string> args);
    uint get_num_of_value_args();
    void set_description (std::string long_name,
			  std::string method_name,
			  std::string short_description,
			  std::string return_description,
			  args_doc arg_description);
    std::string get_description (); //json formated description

    //helper methods, use NULL sentinel
    static args_types make_arg_type_description (GType arg_type, ...);//use G_TYPE_NONE if no arg
    static args_doc make_arg_description (const char *first_arg_long_name, ...);
    
    //Building complex json descriptions incuding this
    JSONBuilder::Node get_json_root_node ();

  private:
    GClosure *closure_;
    GType return_type_;
    args_types arg_types_; 
    uint num_of_value_args_;
    JSONBuilder::ptr json_description_;
    static void destroy_data (gpointer  data,
			      GClosure *closure);
    
  };

}  // end of namespace

#endif // ifndef
