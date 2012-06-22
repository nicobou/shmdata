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

#include "switcher/base-entity-manager.h"
#include "switcher/video-test-source.h"
#include "switcher/ctrl-server.h"
#include "switcher/runtime.h"


 namespace switcher
 {

   BaseEntityManager::BaseEntityManager()
    {
      //registering base entity to make available
      abstract_factory_.Register<Runtime> ("runtime");
      abstract_factory_.Register<VideoTestSource> ("videotestsource");
      // abstract_factory_.Register<CtrlServer> ("controlserver");
    }

    BaseEntityManager::~BaseEntityManager()
    {

    }
  
    std::vector<std::string> 
    BaseEntityManager::get_list_of_creatable_entities ()
    {
      return abstract_factory_.getList ();
    }

    BaseEntity::ptr
    BaseEntityManager::create_entity (std::string entity_class)
    {
      return abstract_factory_.Create (entity_class);
    }

 }
