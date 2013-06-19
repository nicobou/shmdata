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
 * the Abstract creator class
 */

#ifndef __SWITCHER_CREATOR_H__
#define __SWITCHER_CREATOR_H__

#include <vector>

namespace switcher 
{

  template <class T>
    class Creator
    {
    public:
      virtual ~Creator(){}
      virtual T* Create() = 0;
    };
  

  template <class T>
    class DerivedCreator : public Creator<T>
  {
  public:
    T* Create()
    {
      return new T;
    }
  };
  
} // end of namespace
 
#endif // ifndef
