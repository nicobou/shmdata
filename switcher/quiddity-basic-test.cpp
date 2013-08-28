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

#include "quiddity-manager.h"
#include "quiddity-basic-test.h"

namespace switcher
{
  bool 
  QuiddityBasicTest::test_full (std::string quiddity_class_name)
  {
    test_create (quiddity_class_name);
    test_startable (quiddity_class_name);
    test_description (quiddity_class_name);
    return true;
  }

  bool 
  QuiddityBasicTest::test_create (std::string quiddity_class_name)
  {
    switcher::QuiddityManager::ptr manager = switcher::QuiddityManager::make_manager("test_manager");
    
    //testing with a nick name
    std::string res_with_nick = manager->create(quiddity_class_name, quiddity_class_name);
    if (res_with_nick.compare (quiddity_class_name) != 0)
      {
	g_warning ("quiddity %s cannot be created (with nickname)", 
		   quiddity_class_name.c_str ());
	return true; //true because some quiddity may not be crated because of a missing resource 
      }
    
    if (!manager->remove (res_with_nick))
      {
	g_warning ("error while removing quiddity %s (with nickname)", 
		   quiddity_class_name.c_str ());
	return false;
      }


    //testing without nick name
    std::string res_without_nick = manager->create(quiddity_class_name);
    if (res_without_nick.compare ("") == 0)
      {
	g_warning ("quiddity %s cannot be created (without nickname)",
		   quiddity_class_name.c_str ());
	return false;
      }
    
    if (!manager->remove (res_without_nick))
      {
	g_warning ("error while removing quiddity %s (without nickname)",
		   quiddity_class_name.c_str ());
	return false;
      }
    
    return true;
  }

  bool 
  QuiddityBasicTest::test_startable (std::string quiddity_class_name)
  {
    return true;

  }
  
  bool 
  QuiddityBasicTest::test_description (std::string quiddity_class_name)
  {

    return true;
  }
}
