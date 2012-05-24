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
 * The BaseEntity class
 */

#include "switcher/base-entity.h"

namespace switcher
{


  std::set<BaseEntity *> BaseEntity::entities_;

  BaseEntity::BaseEntity ()
  {
    entities_.insert(this);
    g_print ("call: BaseEntity constructor\n");
  }
  
  BaseEntity::~BaseEntity () { 
    entities_.erase(this);
    g_print ("call: BaseEntity destructor for %s\n",get_name().c_str());
    };

  std::string
  BaseEntity::get_name()
  {
    return name_;
  }

}
