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

/**
 * the Abstract Life Manager class
 */

#ifndef __SWITCHER_CREATOR_H__
#define __SWITCHER_CREATOR_H__


#include <vector>
#include "switcher/string-map.h"

namespace switcher 
{

  template <class T, class Arg>
    class Creator
    {
    public:
      virtual ~Creator(){}
      virtual T* Create() = 0;
      virtual T* Create(Arg arg) = 0;

    };
  

  template <class T, class Arg>
    class DerivedCreator : public Creator<T,Arg>
  {
  public:
    T* Create()
    {
      return new T;
    }
    T* Create(Arg arg)
    {
      return new T(arg);
    }

  };
  
} // end of namespace
 
#endif // ifndef
