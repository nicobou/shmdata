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

#include "categorizable.h"
#include "gst/gst.h"

namespace switcher
{
  Categorizable::Categorizable ()
  {
    category_ = "";
    position_weight_ = 0;
  }

  Categorizable::~Categorizable ()
  {}
  
  void 
  Categorizable::set_category (std::string category_name)
  {
    category_ = category_name;
  }
    
  void 
  Categorizable::set_position_weight (int position_weight)
  {
    position_weight_ =  position_weight;
  }

  std::string 
  Categorizable::get_category ()
  {
    return category_;
  }
  
  int 
  Categorizable::get_position_weight ()
  {
    return position_weight_;
  }

  bool
  Categorizable::compare_ptr (Categorizable::ptr first, 
			      Categorizable::ptr second)
  {
    return first->position_weight_ < second->position_weight_;
  }

  bool
  Categorizable::compare (Categorizable first, 
			  Categorizable second)
  {
    return first.position_weight_ < second.position_weight_;
  }
}
