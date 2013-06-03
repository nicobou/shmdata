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


#ifndef __SWITCHER_QUIDDITY_SIGNAL_SUBSCRIBER_H__
#define __SWITCHER_QUIDDITY_SIGNAL_SUBSCRIBER_H__

#include <string>
#include <memory>
#include <map>
#include <vector>
#include "signal-string.h"

namespace switcher
{
  class Quiddity;
  class QuiddityManager_Impl;

  class QuidditySignalSubscriber
  {
  public:
    typedef std::shared_ptr<QuidditySignalSubscriber> ptr;
    typedef void (*OnEmittedCallback)(std::string subscriber_name,
				      std::string quiddity_name,
				      std::string signal_name,
				      std::vector<std::string> params,
				      void *user_data);
    ~QuidditySignalSubscriber();

    void set_callback (OnEmittedCallback cb);
    void set_user_data (void *user_data);
    void set_name (const gchar *name);
    bool subscribe (std::shared_ptr <Quiddity> quid, 
		    std::string signal_name);
    bool unsubscribe (std::shared_ptr <Quiddity> quid, 
		      std::string signal_name);
    bool unsubscribe (std::shared_ptr <Quiddity> quid);
    
    std::vector<std::pair<std::string, std::string> > list_subscribed_signals ();
    static void signal_cb (std::vector<std::string> params, gpointer user_data);
        
    //manager_impl initialization
    void set_manager_impl (std::shared_ptr<QuiddityManager_Impl> manager_impl);

  private:
    OnEmittedCallback user_callback_;
    void *user_data_;
    std::string name_;
    std::weak_ptr<QuiddityManager_Impl> manager_impl_;

    typedef struct {
      gchar *name;
      gchar *quiddity_name; 
      gchar *signal_name;
      OnEmittedCallback user_callback;
      void *user_data;
    } SignalData;
    typedef std::map < std::pair<std::string, std::string>, SignalData *> SignalDataMap;
    SignalDataMap signal_datas_;
  };
  
} // end of namespace

#endif // ifndef
