/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include "switcher/controller.h"
#include "switcher/runtime.h"
#include "switcher/base-entity.h"
#include <iostream>


int
main (int argc,
      char *argv[])
{

  using namespace switcher;
  
  std::cout <<"Hello World\n";
  
  //Register
  Factory<BaseEntity, std::string> temp;
  temp.Register("runtime", (Creator<BaseEntity>*)new DerivedCreator<Runtime>);
  
  //Pointer to base interface
    BaseEntity* pBase = 0;

    //Create and call
    pBase = temp.Create("runtime");
    printf("Runtime %u\n", pBase->Get());
    delete pBase;

  
  std::cout <<"Bye World\n";
  
  return 0;
}
