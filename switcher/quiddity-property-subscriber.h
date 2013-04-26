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


#ifndef __SWITCHER_QUIDDITY_PROPERTY_SUBSCRIBER_H__
#define __SWITCHER_QUIDDITY_PROPERTY_SUBSCRIBER_H__

#include <string>
#include <memory>
#include <map>
#include <vector>
#include "property.h"

namespace switcher
{
  class Quiddity;
  class QuiddityLifeManager;

  class QuiddityPropertySubscriber
  {
  public:
    typedef std::shared_ptr<QuiddityPropertySubscriber> ptr;
    typedef void (*Callback)(std::string subscriber_name,
			     std::string quiddity_name,
			     std::string property_name,
			     std::string value,
			     void *user_data);
    ~QuiddityPropertySubscriber();

    void set_callback (Callback cb);
    void set_user_data (void *user_data);
    void set_name (const gchar *name);
    bool subscribe (std::shared_ptr <Quiddity> quid, 
		    std::string property_name);
    bool unsubscribe (std::shared_ptr <Quiddity> quid, 
		      std::string property_name);
    std::vector<std::pair<std::string, std::string> > list_subscribed_properties ();
    static void property_cb (GObject *gobject, GParamSpec *pspec, gpointer user_data);
    
    //life manager  initialization
    void set_life_manager (std::shared_ptr<QuiddityLifeManager> life_manager);

  private:
    Callback user_callback_;
    void *user_data_;
    std::string name_;
    std::weak_ptr<QuiddityLifeManager> life_manager_;

    typedef struct {
      gchar *name;
      gchar *quiddity_name; 
      gchar *property_name;
      Callback user_callback;
      void *user_data;
    } PropertyData;
    typedef std::map < std::pair<std::string, std::string>, PropertyData *> PropDataMap;
    PropDataMap prop_datas_;
  };
  
} // end of namespace

#endif // ifndef
