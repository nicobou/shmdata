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

#include "quiddity-property-subscriber.h"
#include "quiddity.h" 
#include "quiddity-life-manager.h"

namespace switcher
{

  QuiddityPropertySubscriber::~QuiddityPropertySubscriber()
  {

    QuiddityLifeManager::ptr life_manager = life_manager_.lock ();
    if (!(bool)life_manager)
      return;

    PropDataMap::iterator it;
    for (it = prop_datas_.begin (); it != prop_datas_.end (); it++)
      {
     	Quiddity::ptr quid = life_manager->get_quiddity (it->second->quiddity_name);
	if ((bool)quid)
	  {
	    g_debug ("QuiddityPropertySubscriber: cleaning property not unsubscribed %s, %s, %s",
	    	     it->second->name,
	    	     it->second->quiddity_name,
    		     it->second->property_name);
     	    quid->unsubscribe_property (it->second->property_name, 
					property_cb,
					it->second);
     	    g_free (it->second->name);
     	    g_free (it->second->quiddity_name);
    	    g_free (it->second->property_name);
     	  }
      }
  }
  
  void 
  QuiddityPropertySubscriber::property_cb (GObject *gobject, GParamSpec *pspec, gpointer user_data)
  {
    PropertyData *prop = static_cast <PropertyData *>(user_data);
    
    // g_print ("---------------- property callback: %s -- %s -- %s -- %s\n",  
    // 	     prop->quiddity_name,
    // 	     prop->property_name,
    // 	     Property::parse_callback_args (gobject, pspec).c_str (),
    // 	     (gchar *)prop->user_data); 
    prop->user_callback (prop->name,
			 prop->quiddity_name, 
			 prop->property_name,
			 Property::parse_callback_args (gobject, pspec),
			 (gchar *)prop->user_data); 
  }

  void 
  QuiddityPropertySubscriber::set_life_manager (QuiddityLifeManager::ptr life_manager)
  {
    life_manager_ = life_manager;
  }
  
  void
  QuiddityPropertySubscriber::set_user_data (void *user_data)
  {
    user_data_ = user_data;
  }

  void
  QuiddityPropertySubscriber::set_name (const gchar *name)
  {
    name_ = g_strdup (name);
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
    prop->name = g_strdup (name_.c_str ());
    prop->quiddity_name = g_strdup (quid->get_nick_name ().c_str ());
    prop->property_name = g_strdup (property_name.c_str ());
    prop->user_callback = user_callback_;
    prop->user_data = user_data_;
    if (quid->subscribe_property(property_name.c_str(), property_cb, prop))
      {
	prop_datas_[cur_pair] = prop;
 	return true;
      }
    else
      {
	g_warning ("QuiddityPropertySubscriber: cannot subscribe to property");
	return false;
      }
  }
  
  bool 
  QuiddityPropertySubscriber::unsubscribe (Quiddity::ptr quid, 
					   std::string property_name)
  {
    std::pair<std::string, std::string> cur_pair;
    cur_pair = std::make_pair (quid->get_nick_name (), property_name);
    PropDataMap::iterator it = prop_datas_.find (cur_pair);
    if (it != prop_datas_.end ())
      {
	quid->unsubscribe_property (property_name, 
				    property_cb,
				    it->second);
	g_free (it->second->quiddity_name);
	g_free (it->second->property_name);
	prop_datas_.erase (cur_pair);
	return true;
      }
    g_warning ("not unsubscribing a not registered property (%s %s)",
	       quid->get_nick_name ().c_str (),
	       property_name.c_str ());
    return false;
  }

  std::vector<std::pair<std::string, std::string> > 
  QuiddityPropertySubscriber::list_subscribed_properties ()
  {
    std::vector<std::pair<std::string, std::string> > res;
    PropDataMap::iterator it;
    for (it = prop_datas_.begin (); it != prop_datas_.end (); it++)
      {
	res.push_back (it->first);
      }
    return res;
  }

}
