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


#ifndef __SWITCHER_PROPERTY_H__
#define __SWITCHER_PROPERTY_H__

#include <gst/gst.h>
#include <memory>
#include <map>
#include <string>
#include "json-builder.h"

namespace switcher
{
  
  class Property
  {
  public:
    typedef std::shared_ptr<Property> ptr;
    typedef void (*Callback) (GObject * gobject, GParamSpec * pspec, gpointer user_data);
    Property ();

    //this is when using an existing property
    void set_gobject_pspec (GObject *object, GParamSpec *pspec);
    void set (std::string value);
    std::string get ();
    bool subscribe (Callback cb, void *user_data);
    bool unsubscribe (Callback cb, void *user_data);
    static std::string parse_callback_args (GObject * gobject, GParamSpec * pspec);

    std::string get_description ();
    JSONBuilder::Node get_json_root_node ();

    void print ();

  private:
    GParamSpec *property_;
    GObject *object_;
    JSONBuilder::ptr json_description_;
    void make_description();
    std::map<std::pair<Callback, void *>, gulong> subscribed_handlers_;
  };

}  // end of namespace

#endif // ifndef
