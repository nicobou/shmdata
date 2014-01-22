/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include "switcher/quiddity-manager.h"
#include <vector>
#include <string>

static bool success;
static int signal_counter = 0;

void 
quiddity_created_removed_cb (std::string /*subscriber_name*/, 
			     std::string /*quiddity_name*/, 
			     std::string signal_name, 
			     std::vector<std::string> params, 
			     void */*user_data*/)
{
  g_message ("%s: %s", signal_name.c_str (), params[0].c_str ());
  signal_counter ++;
}

int
main ()
{
  success = false;
  
  {
    switcher::QuiddityManager::ptr manager = switcher::QuiddityManager::make_manager("testing_signals");  

    //make on-quiddity-created and on-quiddity-removed signals
    manager->create ("create_remove_spy", "create_remove_spy");
    manager->make_signal_subscriber ("signal_subscriber", quiddity_created_removed_cb, manager.get ());
    manager->subscribe_signal ("signal_subscriber","create_remove_spy","on-quiddity-created");
    manager->subscribe_signal ("signal_subscriber","create_remove_spy","on-quiddity-removed");
    
    manager->create ("videotestsrc","vid1");
    manager->create ("fakesink", "fake1");
    manager->create ("videotestsrc","vid2");
    manager->create ("fakesink", "fake2");

     // manager->create ("videotestsrc", "vid");
     // manager->subscribe_signal ("signal_subscriber","vid","on-property-added");
     // manager->subscribe_signal ("signal_subscriber","vid","on-property-removed");
     // manager->subscribe_signal ("signal_subscriber","vid","on-property-reinstalled");
     // manager->subscribe_signal ("signal_subscriber","vid","on-method-added");
     // manager->subscribe_signal ("signal_subscriber","vid","on-method-removed");
     // manager->set_property ("vid", "codec", "2");
     // manager->set_property ("vid", "started", "true");
    
    
    std::vector<std::string> subscribers = manager->list_signal_subscribers ();
    if (subscribers.size () != 1 
    	|| g_strcmp0 (subscribers.at(0).c_str (), "signal_subscriber") != 0)
      {
    	g_warning ("pb with list_signal_subscribers");
    	return 1;
      }
    
    std::vector<std::pair<std::string, std::string> > signals = 
      manager->list_subscribed_signals ("signal_subscriber");
    if(signals.size () != 2 
       || g_strcmp0 (signals.at(0).first.c_str (),  "create_remove_spy")
       || g_strcmp0 (signals.at(0).second.c_str (), "on-quiddity-created")
       || g_strcmp0 (signals.at(1).first.c_str (),  "create_remove_spy")
       || g_strcmp0 (signals.at(1).second.c_str (), "on-quiddity-removed"))
      {
    	g_warning ("pb with list_subscribed_signals");
    	return 1;
      }

    manager->remove ("create_remove_spy");

    signals = manager->list_subscribed_signals ("signal_subscriber");
    if(signals.size () != 0)
      {
    	g_warning ("pb with automatic unsubscribe at quiddity removal");
    	return 1;
      }
    // manager->unsubscribe_signal ("signal_subscriber","create_remove_spy","on-quiddity-created");
    // manager->unsubscribe_signal ("signal_subscriber","create_remove_spy","on-quiddity-removed");

    manager->remove_signal_subscriber ("signal_subscriber");
    
    // manager->unsubscribe_signal ("signal_subscriber","create_remove_spy","on-quiddity-created");
    // manager->unsubscribe_signal ("signal_subscriber","create_remove_spy","on-quiddity-removed");
  }

  if (signal_counter == 4) //4 creations has been asked
    success = true;

  if (success)
    return 0;
  else
    return 1;
}



