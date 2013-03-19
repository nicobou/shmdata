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
#include "switcher/quiddity.h" 


namespace switcher
{
  class Quiddity;
  
  class QuiddityPropertySubscriber
  {
  public:
    typedef std::shared_ptr<QuiddityPropertySubscriber> ptr;
    typedef void *(*Callback)(std::string quiddity_name,
			      std::string property_name,
			      std::string value);
    void set_callback (Callback cb);
    bool subscribe (std::shared_ptr <Quiddity> quid, 
		    std::string property_name);
    bool unsubscribe (std::shared_ptr <Quiddity> quid, 
		      std::string property_name);
    //unsubscribe all already subscribed properties of a quid
    //bool unsubscribe (Quiddity quid);

  private:
    Callback user_callback_;
    
  };
  
} // end of namespace

#endif // ifndef
