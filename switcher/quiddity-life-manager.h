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
 * The QuiddityLifeManager class wraps abstract factory for creating instace by name (birth) 
 * It wraps StringMap for managing instances (subsistence). 
 * Object destruction is managed through the use of std::tr1::shared_ptr
 */

#ifndef __SWITCHER_QUIDDITY_LIFE_MANAGER_H__
#define __SWITCHER_QUIDDITY_LIFE_MANAGER_H__


#include <tr1/memory>
#include "switcher/abstract-factory.h" 
#include "switcher/string-map.h"

#include "switcher/quiddity.h" 

namespace switcher
{
  class Quiddity;

  class QuiddityLifeManager
  {
  public:
    typedef std::tr1::shared_ptr< QuiddityLifeManager > ptr;
    
    QuiddityLifeManager();//will get name "default"
    QuiddityLifeManager(std::string);
    
    //info
    std::string get_name ();
    std::vector<std::string> get_classes ();
    bool class_exists (std::string class_name);
    std::vector<std::string> get_instances ();
    bool exists (std::string quiddity_name);
    
    //creation
    std::tr1::shared_ptr<Quiddity> create (std::string quiddity_class);
 
    //subsistence
    std::tr1::shared_ptr<Quiddity> get (std::string quiddity_name);
    
    //release base quiddity (destructed with the shared pointer)
    bool remove (std::string quiddity_name);

  private:
    std::string name_;
    void register_classes ();
    AbstractFactory<Quiddity, std::string, std::string> abstract_factory_;
    StringMap< std::tr1::shared_ptr<Quiddity> > quiddities_;
    };

} // end of namespace



#endif // ifndef
