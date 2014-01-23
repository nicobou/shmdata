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

  template <class T>
    class CustomDerivedCreator : public Creator<T>
  {
  public:
    T* Create()
    {
      return (*custom_create_) ();
    }
    T * (*custom_create_) ();
    
    CustomDerivedCreator () :
       custom_create_ ()
    {}
};
  
} // end of namespace
 
#endif // ifndef
