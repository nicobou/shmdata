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
 * the Abstract factory class
 */

// no includes here since this file is included from abstract-factory.h
// this separation is done in order to make abstract-factory.h easier to read 
// when using it  

namespace switcher 
{
  
  template <typename T, typename Key, typename Doc>
    template <class U> void 
    AbstractFactory<T,  Key, Doc>::register_class (Key Id, Doc doc)
  {
    Creator<T>* Fn = (Creator<T>*)new DerivedCreator<U>();
    constructor_map_[Id] = Fn;
    constructor_names_.push_back (Id);
    classes_documentation_[Id] = doc;
  }

  template <typename T, typename Key, typename Doc>
    std::vector<Key> 
    AbstractFactory<T, Key, Doc>::get_keys ()
  {
    return constructor_names_;
  }

  template <typename T, typename Key, typename Doc>
    std::vector<Doc> 
    AbstractFactory<T, Key, Doc>::get_classes_documentation ()
  {
    std::vector<Doc> tmp;
    typename std::map<Key, Doc>::iterator i = classes_documentation_.begin();
    while (i != classes_documentation_.end())
      {
	tmp.push_back ((*i).second);
	++i;
      }
    return tmp;
  }

  template <typename T, typename Key, typename Doc>
    Doc 
    AbstractFactory<T, Key, Doc>::get_class_documentation (Key id)
  {
    typename std::map<Key, Doc>::iterator iter = classes_documentation_.find(id); 
    if (iter == classes_documentation_.end())
      {
	Doc doc;
	return doc;
      }
    return iter->second;
  }
    
  template <typename T, typename Key, typename Doc>
    bool 
    AbstractFactory<T, Key, Doc>::key_exists(Key Id)
    {
      return ( constructor_map_.find(Id) != constructor_map_.end() );
    }

  template <typename T, typename Key, typename Doc>
    std::shared_ptr<T> 
    AbstractFactory<T, Key, Doc>::create(Key Id)
  {
    std::shared_ptr<T> pointer;
	
    if ( constructor_map_.find(Id) != constructor_map_.end() ) 
      pointer.reset (constructor_map_[Id]->Create());
	
    return pointer;
  }


  template <typename T, typename Key, typename Doc>
    AbstractFactory<T, Key, Doc>::~AbstractFactory()
  {
    typename std::map<Key, Creator<T>*>::iterator i = constructor_map_.begin();
    while (i != constructor_map_.end())
      {
	delete (*i).second;
	++i;
      }
  }
 
} // end of namespace

