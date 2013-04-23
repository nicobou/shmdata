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

#include "switcher/quiddity-manager.h"
#include <vector>
#include <string>

static bool success;
static char *user_string = "hello world";

void 
mon_property_cb(std::string subscriber_name, 
		std::string quiddity_name, 
		std::string property_name, 
		std::string value, 
		void *user_data)
{
  if (g_strcmp0 (quiddity_name.c_str (), "vid") != 0)
    {
      g_warning ("quiddity name does not match, got %s instead of \"vid\"", 
		 quiddity_name.c_str ());
      return;
    }

  if (g_strcmp0 (property_name.c_str (), "pattern") != 0)
    {
      g_warning ("property name does not match, got %s instead of \"pattern\"", 
		 property_name.c_str ());
      return;
    }

  if (g_strcmp0 (value.c_str (), "Random (television snow)") != 0)
    {
      g_warning ("value does not match, got %s instead of Random (television snow)", 
		 value.c_str ());
      return;
    }

  if (g_strcmp0 ((char *)user_data, "hello world") != 0)
    {
      g_warning ("user_data name does not match, got %s instead of \"hello world\"", 
		 (char *)user_data);
      return;
    }

  success = true;
}


int
main (int argc,
      char *argv[])
{
  success = false;
  
  {
    switcher::QuiddityManager::ptr manager = switcher::QuiddityManager::make_manager("test_manager");  
    
    manager->create ("runtime");
    //setting auto_invoke for attaching to gst pipeline "pipeline0"
    std::vector<std::string> arg;
    arg.push_back ("pipeline0");
    manager->auto_invoke ("set_runtime",arg);
    
    manager->make_subscriber ("sub", mon_property_cb, (void *)user_string);
    manager->create ("videotestsrc","vid");
 
    manager->subscribe_property ("sub","vid","pattern");
    manager->subscribe_property ("sub","vid","text");
    
    std::vector<std::string> subscribers = manager->list_subscribers ();
    if (subscribers.size () != 1 
    	|| g_strcmp0 (subscribers.at(0).c_str (), "sub") != 0)
      {
    	g_warning ("pb with list_subscribers");
    	return 1;
      }
    
    std::vector<std::pair<std::string, std::string> > properties = 
      manager->list_subscribed_properties ("sub");
    if(properties.size () != 2 
       || g_strcmp0 (properties.at(0).first.c_str (),  "vid")
       || g_strcmp0 (properties.at(0).second.c_str (), "pattern")
       || g_strcmp0 (properties.at(1).first.c_str (),  "vid")
       || g_strcmp0 (properties.at(1).second.c_str (), "text"))
      {
    	g_warning ("pb with list_subscribed_properties");
    	return 1;
      }
    
    manager->set_property ("vid", "pattern", "1");
    
    manager->unsubscribe_property ("sub", "vid", "pattern");
    manager->remove_subscriber ("sub");
    manager->remove ("vid");
  }

  if (success)
    return 0;
  else
    return 1;
}



