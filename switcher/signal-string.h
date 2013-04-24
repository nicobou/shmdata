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


#ifndef __SWITCHER_SIGNAL_STRING_H__
#define __SWITCHER_SIGNAL_STRING_H__

#include <gst/gst.h>  
#include <memory>  
#include <vector>  
#include <string>  
#include "switcher/json-builder.h" 

namespace switcher 
{ 
  
  class Signal
  { 
  public: 
    typedef std::shared_ptr<Signal> ptr; 
    typedef std::vector<GType> args_types;
    typedef std::vector<std::pair<std::string,std::string> > args_doc;

    Signal (); 
    
    bool set_gobject_signame (GObject *object, std::string signame);

    void set_description (std::string method_name,
			  std::string short_description,
			  args_doc arg_description);
    std::string get_description (); 

    //helper methods, use NULL sentinel
    static args_types make_arg_type_description (GType arg_type, ...);//use G_TYPE_NONE if no arg
    static args_doc make_arg_description (char *first_arg_name, ...);
    JSONBuilder::Node get_json_root_node (); 
    
  private: 
    GObject *object_; 
    guint id_;
    args_types arg_types_; 
    JSONBuilder::ptr json_description_; 
    void make_description(); 
  }; 
  
}  // end of namespace 

#endif // ifndef
