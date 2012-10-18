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
 * The Quiddity command
 */


#ifndef __SWITCHER_QUIDDITY_COMMAND_H__
#define __SWITCHER_QUIDDITY_COMMAND_H__

#include <string>
#include <vector>
#include <tr1/memory>

namespace switcher
{
 
  class QuiddityCommand
  {
  public:
    typedef std::tr1::shared_ptr<QuiddityCommand> ptr;
    
    enum command 
    {
      get_classes, 
      get_quiddities, 
      create, 
      create_nick_named,
      remove,
      get_properties_description,
      get_property_description,
      set_property,
      get_property,
      get_methods_description,
      get_method_description,
      invoke
    };
    command name_;
    std::vector<std::string> args_;
    std::vector<std::string> vector_arg_;
    std::vector<std::string> exec_return_;

    void make (command name); 
    void make (command name, 
	       std::string arg0); 
    void make (command name, 
	       std::string arg0, 
	       std::string arg1); 
    void make (command name, 
	       std::string arg0, 
	       std::string arg1, 
	       std::string arg2); 
    void make (command name, 
	       std::string arg0, 
	       std::string arg1, 
	       std::vector<std::string> vector_arg); 
   
  private:
    void clear();
    
  };
  
} // end of namespace

#endif // ifndef
