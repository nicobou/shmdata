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
 * The Quiddity property subscriber
 */

#include "switcher/quiddity-property-subscriber.h"
#include "switcher/quiddity.h" 

namespace switcher
{
  
  void
  QuiddityPropertySubscriber::set_callback (Callback cb)
  {
    user_callback_ = cb;
  }
  bool 
  QuiddityPropertySubscriber::subscribe (Quiddity::ptr quid, 
					 std::string property_name)
  {
    
    return false;
  }
  
  bool 
  QuiddityPropertySubscriber::unsubscribe (Quiddity::ptr quid, 
					   std::string property_name)
  {
    return false;
  }
}
