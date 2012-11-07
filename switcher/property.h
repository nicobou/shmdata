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


#ifndef __SWITCHER_PROPERTY_H__
#define __SWITCHER_PROPERTY_H__

#include <gst/gst.h>
#include <memory>
#include <map>
#include <string>

namespace switcher
{
  
  class Property
  {
  public:
    typedef std::shared_ptr<Property> ptr;
    
    //this is when using an existing property
    void set_gobject_pspec (GObject *object, GParamSpec *pspec);
    void set (std::string value);
    std::string get ();
    std::string get_description ();
    void print ();

  private:
    GParamSpec *property_;
    GObject *object_;
    std::string json_description_;
    void add_json_object (const char *name, const char *value, bool put_comma);
    void make_description();
  };

}  // end of namespace

#endif // ifndef
