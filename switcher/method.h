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




#ifndef __SWITCHER_METHOD_H__
#define __SWITCHER_METHOD_H__

#include <gst/gst.h>
#include <tr1/memory>
#include <vector>
#include <string>

namespace switcher
{
  
  class Method
  {
  public:
    typedef std::tr1::shared_ptr<Method> ptr;

    Method ();
    ~Method ();
    void set_method (void *method, 
		     std::vector<GType> arg_types, 
		     gpointer user_data);
    bool invoke (std::vector<std::string> args);
    bool invoke (std::vector<std::string> args,
		 std::vector<void *> entity_args);
    uint get_num_of_value_args();
    uint get_num_of_pointer_args();

  private:
    GClosure *closure_;
    std::vector<GType> arg_types_; 
    uint num_of_value_args_;
    uint num_of_pointer_args_;

    static void destroy_data (gpointer  data,
			      GClosure *closure);
    
  };

}  // end of namespace

#endif // ifndef
