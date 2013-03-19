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
 * The Quiddity property subscriber
 */

#include "switcher/quiddity-property-subscriber.h"
#include "switcher/quiddity.h" 

namespace switcher
{
  void 
  QuiddityPropertySubscriber::property_cb (GObject *gobject, GParamSpec *pspec, gpointer user_data)
  {
    PropertyData *prop = static_cast <PropertyData *>(user_data);
    
    g_print ("---------------- property callback: %s -- %s -- %s\n",  
	     prop->quiddity_name,
	     prop->property_name,
	     Property::parse_callback_args (gobject, pspec).c_str ()); 
  }

  void
  QuiddityPropertySubscriber::set_callback (Callback cb)
  {
    user_callback_ = cb;
  }
  bool 
  QuiddityPropertySubscriber::subscribe (Quiddity::ptr quid, 
					 std::string property_name)
  {
    if (user_callback_ == NULL)
      {
	g_warning ("cannot subscribe before setting a callback (%s %s)",
		   quid->get_nick_name ().c_str (),
		   property_name.c_str ());
	return false;
      }
    std::pair<std::string, std::string> cur_pair;
    cur_pair = std::make_pair (quid->get_nick_name (), property_name);
    if (prop_datas_.find (cur_pair) != prop_datas_.end ())
      {
	g_warning ("not subscribing twice the same property (%s %s)",
		   quid->get_nick_name ().c_str (),
		   property_name.c_str ());
	return false;
      }
    PropertyData *prop = new PropertyData ();
    prop->quiddity_name = g_strdup (quid->get_nick_name ().c_str ());
    prop->property_name = g_strdup (property_name.c_str ());
    prop->user_callback = user_callback_;
    if (quid->subscribe_property(property_name.c_str(), property_cb, prop))
      {
	prop_datas_[cur_pair] = prop;
	return true;
      }
    else
      return false;
  }
  
  bool 
  QuiddityPropertySubscriber::unsubscribe (Quiddity::ptr quid, 
					   std::string property_name)
  {
    return false;
  }
}
