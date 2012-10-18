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
 * The Quiddity command class
 */

#include "switcher/quiddity-command.h"

namespace switcher
{

  void
  QuiddityCommand::clear()
  {
    args_.clear ();
    vector_arg_.clear ();
    exec_return_.clear ();
  }

  void 
  QuiddityCommand::make (command name)
  {
    clear ();
    name_ = name;
  } 
  
  void 
  QuiddityCommand::make (command name, 
			 std::string arg0)
  {
    clear ();
    name_ = name;
    args_.push_back (arg0);
  } 
  
  void 
  QuiddityCommand::make (command name, 
			 std::string arg0, 
			 std::string arg1)
  {
    clear();
    name_ = name;
    args_.push_back (arg0);
    args_.push_back (arg1);
  } 
  
  void 
  QuiddityCommand::make (command name, 
	       std::string arg0, 
	       std::string arg1, 
	       std::string arg2)
  {
    clear ();
    name_ = name;
    args_.push_back (arg0);
    args_.push_back (arg1);
    args_.push_back (arg2);
  } 

  void 
  QuiddityCommand::make (command name, 
			 std::string arg0, 
			 std::string arg1, 
			 std::vector<std::string> vector_arg)
  {
    clear ();
    name_ = name;
    args_.push_back (arg0);
    args_.push_back (arg1);
    vector_arg_ = vector_arg;
  } 
  

}
