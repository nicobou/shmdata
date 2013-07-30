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

/**
 * The StringMap template 
 */

#ifndef __SWITCHER_STRING_MAP_H__
#define __SWITCHER_STRING_MAP_H__

#include <string>
#include <map>
#include <vector>
#include <memory>

namespace switcher
{

  template <class T>
    class StringMap
    {
    public:
      typedef std::shared_ptr< StringMap<T> > ptr;

      bool insert (const std::string key, T value);
      bool replace (const std::string key, T value);
      bool remove (const std::string key);
      bool contains (const std::string key);
      unsigned int size ();
      T lookup (const std::string key);
      std::vector<std::string> get_keys ();
      std::vector<T> get_values ();
      std::map<std::string, T> get_map ();
      void clear ();
      
    private:
      std::map<std::string, T> map_;
    };

} // end of namespace

  //including the actual code here
#include "string-map_spec.h" 


#endif // ifndef
