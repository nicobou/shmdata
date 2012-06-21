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
 * The BaseEntity class
 */


#ifndef __SWITCHER_BASE_ENTITY_H__
#define __SWITCHER_BASE_ENTITY_H__

#include <string>
#include <vector>
#include <tr1/memory>
#include <map>
#include <gst/gst.h>
#include <set>
#include "switcher/property.h"

namespace switcher
{

  class BaseEntity
  {
  public:
    typedef std::tr1::shared_ptr<BaseEntity> ptr;

    BaseEntity ();
    virtual ~BaseEntity ();
    
    //virtual bool Get () = 0;
    std::string get_name ();
    void list_properties ();
    bool set_property (std::string name, std::string value);
    std::string get_property (std::string name);
    static std::vector<std::string> *get_entity_instance_names();

  private:
    static std::set<BaseEntity *> entities_;
    //properties are registered by derived class
    std::map<std::string, Property::ptr> properties_;

  protected:
    std::string name_;
    bool register_property (GObject *object, std::string name);
  };
  
} // end of namespace

#endif // ifndef
