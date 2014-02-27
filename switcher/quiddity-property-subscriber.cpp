/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * The Quiddity property subscriber
 */

#include "quiddity-property-subscriber.h"
#include "quiddity.h" 
#include "quiddity-manager-impl.h"

namespace switcher
{
  QuiddityPropertySubscriber::QuiddityPropertySubscriber()
  {
    muted_ = false;
  }

  
  QuiddityPropertySubscriber::~QuiddityPropertySubscriber()
  {

    QuiddityManager_Impl::ptr manager = manager_impl_.lock ();
    if (!(bool)manager)
      return;

    for (auto &it : prop_datas_)
      {
     	Quiddity::ptr quid = manager->get_quiddity (it.second->quiddity_name);
    	if ((bool)quid)
    	  {
    	    g_debug ("QuiddityPropertySubscriber: cleaning property not unsubscribed %s, %s, %s",
    	    	     it.second->name,
    	    	     it.second->quiddity_name,
    		     it.second->property_name);
     	    quid->unsubscribe_property (it.second->property_name, 
    	    				property_cb,
    	    				it.second);
     	    g_free (it.second->name);
     	    g_free (it.second->quiddity_name);
    	    g_free (it.second->property_name);
     	  }
      }
  }

  void
  QuiddityPropertySubscriber::mute (bool muted)
  {
    muted_ = muted;
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
    if (!prop->property_subscriber->muted_)
      prop->user_callback (prop->name,
			   prop->quiddity_name, 
			   prop->property_name,
			   Property::parse_callback_args (gobject, pspec),
			   (gchar *)prop->user_data); 
  }

  void 
  QuiddityPropertySubscriber::set_manager_impl (QuiddityManager_Impl::ptr manager_impl)
  {
    manager_impl_ = manager_impl;
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
    prop->property_subscriber = this;
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
    //std::pair<std::string, std::string> cur_pair;
    auto cur_pair = std::make_pair (quid->get_nick_name (), property_name);
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

  bool 
  QuiddityPropertySubscriber::unsubscribe (Quiddity::ptr quid)
  {
    auto quid_name = quid->get_nick_name ();
    std::vector <std::pair<std::string, std::string>> entries_to_remove;
    for (auto& it: prop_datas_)
      if (it.first.first == quid_name)
	{
	  g_free (it.second->quiddity_name);
	  g_free (it.second->property_name);
	  entries_to_remove.push_back (it.first);
	}
    for (auto &it : entries_to_remove)
	prop_datas_.erase (it);
    return true;
  }
  
  std::vector<std::pair<std::string, std::string>> 
  QuiddityPropertySubscriber::list_subscribed_properties ()
  {
      std::vector<std::pair<std::string, std::string>>  res;
    for (auto &it : prop_datas_)
      res.push_back (it.first);
    return res;
  }

}
