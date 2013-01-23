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
      map_[key]=value;
      return true;
    }
   

  template <typename T>
    bool 
    StringMap<T>::remove (const std::string key)
    {
      typename std::map<std::string, T>::iterator  it= map_.find(key);
      if( it == map_.end() ) 
	return false;
      map_.erase (key);
      return true;
    }
  
  template <typename T>
    bool 
    StringMap<T>::contains (const std::string key)
    {
      if (map_.count(key) == 0)
	return false;
      else
	return true;
    }
  
  template <typename T>
    unsigned int 
    StringMap<T>::size ()
    {
      return map_.size();
    }

  template <typename T>
    void 
    StringMap<T>::clear ()
    {
      map_.clear();
      return;
    }

  template <typename T>
    T 
    StringMap<T>::lookup (const std::string key)
    {
      typename std::map<std::string, T>::iterator  it= map_.find(key);
      /* if( it == map_.end() )  */
      /* 	return NULL; */
      
      return it->second;
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
    typename std::map<std::string, T>  
    StringMap<T>::get_map () 
    { 
      return map_; 
     } 
    
}
