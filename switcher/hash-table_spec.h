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
 * The Hash Table class
 */

// no includes here since this file is included from hash-table
// this separation is done in order to make hash-table.h easier to read 
// when using it  

namespace switcher
{
  template <typename T>
    HashTable<T>::HashTable () 
    {
      table_  = g_hash_table_new_full ((GHashFunc)g_str_hash, 
				       (GEqualFunc)g_str_equal,
				       (GDestroyNotify)g_free, //freeing keys
				       NULL); //not freeing values
    }
  
  template <typename T>
    HashTable<T>::~HashTable () 
    {
      g_hash_table_destroy (table_);
    }
  
  template <typename T>
    void
    HashTable<T>::insert (const std::string key, 
			  T *value)
    {
      char* key_to_record = g_strdup (key.c_str());
      g_hash_table_insert (table_,
			   (gpointer)key_to_record,
			   (gpointer)value);
    }
  
  template <typename T>
    bool 
    HashTable<T>::remove (const std::string key)
    {
      if (g_hash_table_remove (table_,(gconstpointer) key.c_str()))
	  return true;
      else
	return false;
    }
  
  template <typename T>
    bool 
    HashTable<T>::contains (const std::string key)
    {
      if ( g_hash_table_contains (table_, key.c_str()))
	return true;
      else
	return false;
    }
  
  template <typename T>
    unsigned int 
    HashTable<T>::size ()
    {
      return (unsigned int) g_hash_table_size (table_);
    }

  template <typename T>
    T *
    HashTable<T>::lookup (const std::string key)
    {
      gboolean res;
      gpointer value;
      res = g_hash_table_lookup_extended        (table_,
						 (gconstpointer) key.c_str(),
						 NULL, //origin key
						 &value);

      if (res && value == NULL)
	g_debug ("warning: key %s has been found with a NULL value", key.c_str() );
      
      return (T *)value;
    }
  
  template <typename T>
    std::vector<std::string> 
    HashTable<T>::get_keys ()
    {
      GList *list = g_hash_table_get_keys (table_);
      std::vector<std::string> keys;
	
      while (list != NULL)
	{
	  keys.push_back ((char *)list->data);
	  list = g_list_remove(list,list->data);
	}
	
      return keys;
    }
       
  template <typename T>
    std::vector<T *> 
    HashTable<T>::get_values ()
    {
      GList *list = g_hash_table_get_values (table_);
      std::vector<T *> values;
	
      while (list != NULL)
	{
	  values.push_back ((T *)list->data);
	  list = g_list_remove(list,list->data);
	}
      return values;
    }
  
  template <typename T>
    void
    HashTable<T>::for_each (GHFunc function, 
			    void *user_data)
    {
      g_hash_table_foreach (table_,
			    function,
			    (gpointer)user_data);
    }
    
    
}
