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

#ifndef __SWITCHER_ABSTRACT_FACTORY_H__
#define __SWITCHER_ABSTRACT_FACTORY_H__


#include <vector>
#include <map>
#include "creator.h"

namespace switcher 
{

  template <class T, class Key, class Doc>
    class AbstractFactory
  {
  public:
    template <class U> void register_class (Key Id, Doc doc);
    std::vector<Key> get_keys ();
    std::vector<Doc> get_classes_documentation ();
    Doc get_class_documentation (Key Id);
    std::shared_ptr<T> create(Key Id);
    bool key_exists (Key Id);
    ~AbstractFactory();

  private:
    std::map<Key, Creator<T>*> constructor_map_;
    std::map<Key, Doc> classes_documentation_;
    std::vector<Key> constructor_names_;
  };
  
  
} // end of namespace

#include "abstract-factory_spec.h"

#endif // ifndef
