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
 * The HashTable class (keys are string, values are void *)
 */


#ifndef __SWITCHER_HASH_TABLE_H__
#define __SWITCHER_HASH_TABLE_H__

#include <glib.h>
#include <tr1/memory>
#include <string>


namespace switcher
{

  class HashTable
  {
  public:
    typedef std::tr1::shared_ptr<HashTable> ptr;
    
    HashTable ();
    ~HashTable ();

    void insert (const std::string key, void *value);
    bool contains (const std::string key);
    unsigned int size ();
    void *lookup (const std::string key);
    void for_each (GHFunc function, void *user_data);

  private:
    GHashTable *table_;
  };
  
} // end of namespace

#endif // ifndef
