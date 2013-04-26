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
 * The Quiddity command class
 */

#include "quiddity-command.h"

namespace switcher
{

  void
  QuiddityCommand::clear()
  {
    args_.clear ();
    vector_arg_.clear ();
    result_.clear ();
  }

  void 
  QuiddityCommand::set_name (command name)
  {
    clear ();
    name_ = name;
  } 
  
  void 
  QuiddityCommand::add_arg (std::string arg)
  {
    args_.push_back (arg);
  } 
 
  void 
  QuiddityCommand::set_vector_arg (std::vector<std::string> vector_arg)
  {
    vector_arg_ = vector_arg;
  } 

}
