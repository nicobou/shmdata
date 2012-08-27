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
 * the Abstract Life Manager class
 */

// no includes here since this file is included from abstract-life-manager.h
// this separation is done in order to make abstract-life-manager.h easier to read 
// when using it  

namespace switcher 
{
  
    template <typename T, typename Key>
    template <class U> void 
    AbstractFactory<T,  Key>::register_class (Key Id)
  {
    Creator<T>* Fn = (Creator<T>*)new DerivedCreator<U>();
    constructor_map_[Id] = Fn;
    constructor_names_.push_back (Id);
  }

  template <typename T, typename Key>
    std::vector<Key> 
    AbstractFactory<T, Key>::get_keys ()
  {
    return constructor_names_;
  }
    
  template <typename T, typename Key>
    bool 
    AbstractFactory<T, Key>::key_exists(Key Id)
    {
      return ( constructor_map_.find(Id) != constructor_map_.end() );
    }

  template <typename T, typename Key>
    std::tr1::shared_ptr<T> 
    AbstractFactory<T, Key>::create(Key Id)
  {
    std::tr1::shared_ptr<T> pointer;
	
    if ( constructor_map_.find(Id) != constructor_map_.end() ) 
      pointer.reset (constructor_map_[Id]->Create());
	
    return pointer;
  }

  template <typename T, typename Key>
    AbstractFactory<T, Key>::~AbstractFactory()
  {
    typename std::map<Key, Creator<T>*>::iterator i = constructor_map_.begin();
    while (i != constructor_map_.end())
      {
	delete (*i).second;
	++i;
      }
  }
 
} // end of namespace

