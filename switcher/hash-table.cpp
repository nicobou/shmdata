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
 * The Hash Table class
 */

#include "switcher/hash-table.h"


namespace switcher
{

  HashTable::HashTable () :
    table_ (g_hash_table_new_full ((GHashFunc)g_str_hash, 
				   (GEqualFunc)g_str_equal,
				   (GDestroyNotify)g_free, //freeing keys
				   NULL)) //not freeing values
  {}
  
  
  HashTable::~HashTable () 
  {
    g_hash_table_destroy (table_);
  }

  void
  HashTable::insert (const std::string key, 
		     void *value)
  {
    char* key_to_record = g_strdup (key.c_str());
    g_hash_table_insert (table_,
			 (gpointer)key_to_record,
			 (gpointer)value);
  }

  bool 
  HashTable::remove (const std::string key)
  {
    if (g_hash_table_remove (table_,(gconstpointer) key.c_str()))
      return true;
    else
      return false;
  }
  
  bool 
  HashTable::contains (const std::string key)
  {
    if ( g_hash_table_contains (table_, key.c_str()))
      return true;
    else
      return false;
  }

  unsigned int 
  HashTable::size ()
  {
    return (unsigned int) g_hash_table_size (table_);
  }


  void *
  HashTable::lookup (const std::string key)
  {
    gboolean res;
    gpointer value;
    res = g_hash_table_lookup_extended        (table_,
					       (gconstpointer) key.c_str(),
					       NULL, //origin key
					        &value);
    if (res && value == NULL)
      g_print ("warning: key %s has been found with a NULL value", key.c_str() );

    return (void *)value;
  }

  void
  HashTable::for_each (GHFunc function, 
		       void *user_data)
  {
    g_hash_table_foreach (table_,
			  function,
			  (gpointer)user_data);
  }


}
