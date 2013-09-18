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
 * The StringMap 
 */

// no includes here since this file is included from string-map
// this separation is done in order to make hash-table.h easier to read 
// when using it  

namespace switcher
{
  
  template <typename T>
    bool
    StringMap<T>::insert (const std::string key, 
			  T value)
    {
      if (contains(key))
	return false;

      map_[key]=value;
      return true;
    }
  
  template <typename T>
    bool
    StringMap<T>::replace (const std::string key, 
			   T value)
    {
      if (is_disabled (key))
	{
	  map_disabled_[key] = value;
	  return true;
	}
      map_[key]=value;
      return true;
    }
   

  template <typename T>
    bool 
    StringMap<T>::remove (const std::string key)
    {
      if (is_enabled (key))
	{
	  map_.erase (key);
	  return true;
	}
      if (is_disabled (key))
	{
	  map_disabled_.erase (key);
	  return true;
	}
      return false;
    }
  
  template <typename T>
    bool 
    StringMap<T>::contains (const std::string key)
    {
      if (map_.count (key) == 0 && map_disabled_.count (key) == 0)
	return false;
      return true;
    }

  template <typename T>
    bool 
    StringMap<T>::is_enabled (const std::string key)
    {
      if (map_.count (key) == 0)
	return false;
      return true;
    }

  template <typename T>
    bool 
    StringMap<T>::is_disabled (const std::string key)
    {
      if (map_disabled_.count (key) == 0)
	return false;
      return true;
    }
 
  template <typename T>
    bool 
    StringMap<T>::enable (const std::string key)
    {
      if (is_enabled (key))
	return true;

      if (!is_disabled (key))
	return false;
      
      typename std::map<std::string, T>::iterator  it;
      it = map_disabled_.find (key);
      map_[it->first] = it->second;
      map_disabled_.erase (it);
      return true;
    }

  template <typename T>
    bool 
    StringMap<T>::disable (const std::string key)
    {
      if (is_disabled (key))
	return true;
      
      if (!is_enabled (key))
	return false;
      
      typename std::map<std::string, T>::iterator  it;
      it = map_.find (key);
      map_disabled_[it->first] = it->second;
      map_.erase (it);
      return true;
    }

  template <typename T>
    bool 
    StringMap<T>::set_enabled (const std::string key, bool enabling)
    {
      if (enabling)
	return enable (key);
      return disable (key);
    }
  
  
  template <typename T>
    unsigned int 
    StringMap<T>::size ()
    {
      return map_.size () + map_disabled_.size ();
    }

  template <typename T>
    unsigned int 
    StringMap<T>::size_enabled ()
    {
      return map_.size ();
    }

  template <typename T>
    unsigned int 
    StringMap<T>::size_disabled ()
    {
      return map_disabled_.size ();
    }

  template <typename T>
    void 
    StringMap<T>::clear ()
    {
      map_.clear ();
      map_disabled_.clear ();
      return;
    }

  template <typename T>
    void 
    StringMap<T>::clear_enabled ()
    {
      map_.clear ();
      return;
    }

  template <typename T>
    void 
    StringMap<T>::clear_disabled ()
    {
      map_disabled_.clear ();
      return;
    }

  template <typename T>
    T 
    StringMap<T>::lookup (const std::string key)
    {
      return map_.find(key)->second;
    }

  template <typename T>
    T 
    StringMap<T>::lookup_full (const std::string key)
    {
      if (is_enabled (key))
	return map_.find(key)->second;
      return map_disabled_.find (key)->second;
    }
  
  template <typename T>
    std::vector<std::string> 
    StringMap<T>::get_keys ()
    {
      std::vector<std::string> keys;
      
      typename std::map<std::string, T>::iterator it;
      for (it = map_.begin(); it != map_.end(); ++it)
	{
	  keys.push_back (it->first);
	}
      return keys;
    }

  template <typename T>
    std::vector<std::string> 
    StringMap<T>::get_disabled_keys ()
    {
      std::vector<std::string> keys;
      
      typename std::map<std::string, T>::iterator it;
      for (it = map_disabled_.begin(); it != map_disabled_.end(); ++it)
	{
	  keys.push_back (it->first);
	}
      return keys;
    }
       
  template <typename T> 
    typename std::vector<T>  
    StringMap<T>::get_values () 
    { 
      typename std::vector<T> values; 
      
      typename std::map<std::string, T>::iterator  it; 
      for (it = map_.begin(); it != map_.end(); it++) 
   	{ 
   	  values.push_back (it->second); 
   	} 
      
      return values; 
     } 

  template <typename T> 
    typename std::vector<T>  
    StringMap<T>::get_disabled_values () 
    { 
      typename std::vector<T> values; 
      
      typename std::map<std::string, T>::iterator  it; 
      for (it = map_disabled_.begin(); it != map_disabled_.end(); it++) 
   	{ 
   	  values.push_back (it->second); 
   	} 
      
      return values; 
     } 

  template <typename T> 
    typename std::map<std::string, T>  
    StringMap<T>::get_map () 
    { 
      return map_; 
    } 

  template <typename T> 
    typename std::map<std::string, T>  
    StringMap<T>::get_disabled_map () 
    { 
      return map_disabled_; 
     } 
    
}
