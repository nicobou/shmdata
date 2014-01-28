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
#include "switcher/quiddity-basic-test.h"
#include <vector>
#include <string>
#include <iostream>
int
main (int /*argc*/,
      char */*argv*/[])
{
  bool success = true;
  switcher::QuiddityManager::ptr manager = switcher::QuiddityManager::make_manager("test_manager");  
  std::vector<std::string> classes = manager->get_classes ();
  
  for (auto &it: classes)
    {  
      //std::cout << class_name << std::endl; 
      if (!switcher::QuiddityBasicTest::test_create (manager, it))
	success = false;
      //std::cout << res << std::endl;
    }
  
  if (success)
    return 0;
  else
    return 1;
}



