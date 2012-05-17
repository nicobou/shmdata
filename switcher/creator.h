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
 * the AbstractFactory class
 */

#ifndef __SWITCHER_CREATOR_H__
#define __SWITCHER_CREATOR_H__

#include "memory.h"

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
  
  template <class T, class Key>
    class Factory
  {
  public:
    template <class U>
    void Register(Key Id)
    {
      Creator<T>* Fn = (Creator<T>*)new DerivedCreator<U>();
      FunctionMap[Id] = Fn;
    }
    
    std::tr1::shared_ptr<T> Create(Key Id)
    {
      std::tr1::shared_ptr<T> pointer (FunctionMap[Id]->Create());
      return pointer;
    }
    
    ~Factory()
      {
        typename std::map<Key, Creator<T>*>::iterator i = FunctionMap.begin();
        while (i != FunctionMap.end())
	  {
            delete (*i).second;
            ++i;
	  }
      }

  private:
    std::map<Key, Creator<T>*> FunctionMap;
  };


} // end of namespace

#endif // ifndef
