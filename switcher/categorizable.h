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


#ifndef __SWITCHER_CATEGORIZABLE_H__
#define __SWITCHER_CATEGORIZABLE_H__

#include <string>
#include <memory>

namespace switcher
{

  class Categorizable
  {
  public:
    typedef std::shared_ptr<Categorizable> ptr;

    Categorizable ();
    virtual ~Categorizable ();
    //default category is ""
    void set_category (std::string category_name);
    //default position weight is 0
    void set_position_weight (int position_weight);

    std::string get_category ();
    int get_position_weight ();

    static bool compare_ptr (Categorizable::ptr first, 
			     Categorizable::ptr second);

    static bool compare (Categorizable first, 
			 Categorizable second);


  private:
    std::string category_;
    int position_weight_;
  };

}  // end of namespace

#endif // ifndef
