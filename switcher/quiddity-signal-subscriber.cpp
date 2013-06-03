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
 * The Quiddity signal subscriber
 */

#include "quiddity-signal-subscriber.h"
#include "quiddity.h" 
#include "quiddity-manager-impl.h"

namespace switcher
{

  QuidditySignalSubscriber::~QuidditySignalSubscriber()
  {
    QuiddityManager_Impl::ptr manager = manager_impl_.lock ();
    if (!(bool)manager)
      return;

    SignalDataMap::iterator it;
    for (it = signal_datas_.begin (); it != signal_datas_.end (); it++)
      {
     	Quiddity::ptr quid = manager->get_quiddity (it->second->quiddity_name);
	if ((bool)quid)
	  {
	    g_debug ("QuidditySignalSubscriber: cleaning signal not unsubscribed %s, %s, %s",
	    	     it->second->name,
	    	     it->second->quiddity_name,
    		     it->second->signal_name);
     	    quid->unsubscribe_signal (it->second->signal_name, 
				      signal_cb,
				      it->second);
     	    g_free (it->second->name);
     	    g_free (it->second->quiddity_name);
    	    g_free (it->second->signal_name);
     	  }
      }
  }
  
  void 
  QuidditySignalSubscriber::signal_cb (std::vector <std::string> params, gpointer user_data)
  {
    SignalData *signal = static_cast <SignalData *>(user_data);
    
    // g_print ("---------------- signal callback: %s -- %s -- %s -- %s\n",  
    // 	     signal->quiddity_name,
    // 	     signal->signal_name,
    // 	     Signal::parse_callback_args (gobject, pspec).c_str (),
    // 	     (gchar *)signal->user_data); 
    signal->user_callback (signal->name,
			   signal->quiddity_name, 
			   signal->signal_name,
			   params,
			   (gchar *)signal->user_data); 
  }

  void 
  QuidditySignalSubscriber::set_manager_impl (QuiddityManager_Impl::ptr manager_impl)
  {
    manager_impl_ = manager_impl;
  }
  
  void
  QuidditySignalSubscriber::set_user_data (void *user_data)
  {
    user_data_ = user_data;
  }

  void
  QuidditySignalSubscriber::set_name (const gchar *name)
  {
    name_ = g_strdup (name);
  }

  void
  QuidditySignalSubscriber::set_callback (OnEmittedCallback cb)
  {
    user_callback_ = cb;
  }

  bool 
  QuidditySignalSubscriber::subscribe (Quiddity::ptr quid, 
				       std::string signal_name)
  {
    if (user_callback_ == NULL)
      {
	g_warning ("cannot subscribe before setting a callback (%s %s)",
		   quid->get_nick_name ().c_str (),
		   signal_name.c_str ());
	return false;
      }
    std::pair<std::string, std::string> cur_pair;
    cur_pair = std::make_pair (quid->get_nick_name (), signal_name);
    if (signal_datas_.find (cur_pair) != signal_datas_.end ())
      {
	g_warning ("not subscribing twice the same signal (%s %s)",
		   quid->get_nick_name ().c_str (),
		   signal_name.c_str ());
	return false;
      }
    SignalData *signal = new SignalData ();
    signal->name = g_strdup (name_.c_str ());
    signal->quiddity_name = g_strdup (quid->get_nick_name ().c_str ());
    signal->signal_name = g_strdup (signal_name.c_str ());
    signal->user_callback = user_callback_;
    signal->user_data = user_data_;
    if (quid->subscribe_signal(signal_name.c_str(), signal_cb, signal))
      {
	signal_datas_[cur_pair] = signal;
 	return true;
      }
    else
      {
	g_warning ("QuidditySignalSubscriber: cannot subscribe to signal");
	return false;
      }
  }
  
  bool 
  QuidditySignalSubscriber::unsubscribe (Quiddity::ptr quid, 
					   std::string signal_name)
  {
    std::pair<std::string, std::string> cur_pair;
    cur_pair = std::make_pair (quid->get_nick_name (), signal_name);
    SignalDataMap::iterator it = signal_datas_.find (cur_pair);
    if (it != signal_datas_.end ())
      {
	quid->unsubscribe_signal (signal_name, 
				    signal_cb,
				    it->second);
	g_free (it->second->quiddity_name);
	g_free (it->second->signal_name);
	signal_datas_.erase (cur_pair);
	return true;
      }
    g_warning ("not unsubscribing a not registered signal (%s %s)",
	       quid->get_nick_name ().c_str (),
	       signal_name.c_str ());
    return false;
  }

  bool 
  QuidditySignalSubscriber::unsubscribe (Quiddity::ptr quid)
  {
    std::string quid_name = quid->get_nick_name ();
    for (auto& it: signal_datas_)
      if (it.first.first == quid_name)
	{
	  g_free (it.second->quiddity_name);
	  g_free (it.second->signal_name);
	  signal_datas_.erase (it.first);
	}
    return true;
  }

  std::vector<std::pair<std::string, std::string> > 
  QuidditySignalSubscriber::list_subscribed_signals ()
  {
    std::vector<std::pair<std::string, std::string> > res;
    SignalDataMap::iterator it;
    for (it = signal_datas_.begin (); it != signal_datas_.end (); it++)
      {
	res.push_back (it->first);
      }
    return res;
  }

}
