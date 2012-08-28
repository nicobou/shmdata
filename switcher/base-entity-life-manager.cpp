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

#include "switcher/base-entity-life-manager.h"
#include "switcher/base-entity.h" 

//the base entities to manage (line sorted)
#include "switcher/audio-test-source.h"
#include "switcher/ctrl-server.h"
#include "switcher/gconf-video-sink.h"
#include "switcher/gconf-video-source.h"
#include "switcher/runtime.h"
#include "switcher/video-test-source.h"
#include "switcher/xvimagesink.h"

namespace switcher
{

  BaseEntityLifeManager::BaseEntityLifeManager()
  {
    //registering base entity to make available (line sorted)
    abstract_factory_.register_class<AudioTestSource> ("audiotestsource");
    abstract_factory_.register_class<CtrlServer> ("controlserver");
    abstract_factory_.register_class<GconfVideoSink> ("gconfvideosink");
    abstract_factory_.register_class<GconfVideoSource> ("gconfvideosource");
    abstract_factory_.register_class<Runtime> ("runtime");
    abstract_factory_.register_class<VideoTestSource> ("videotestsource");
    abstract_factory_.register_class<Xvimagesink> ("xvimagesink");
  }

  std::vector<std::string> 
  BaseEntityLifeManager::get_classes ()
  {
    return abstract_factory_.get_keys ();
  }

  bool 
  BaseEntityLifeManager::class_exists (std::string class_name)
  {
    return abstract_factory_.key_exists (class_name);
  }

  BaseEntity::ptr 
  BaseEntityLifeManager::create (std::string entity_class)
  {
    BaseEntity::ptr entity = abstract_factory_.create (entity_class);
    g_print ("create_entity %p %p\n",&entity,entity.get());
    if (entity.get() != NULL)
      {
	//hashed_->insert (entity->get_name(),&entity);
	entities_.insert (entity->get_name(),entity);
      }
    return entity;
  }

  std::vector<std::string> 
  BaseEntityLifeManager::get_instances ()
  {
    return entities_.get_keys();
  }

  BaseEntity::ptr 
  BaseEntityLifeManager::get (std::string entity_name)
  {
    return entities_.lookup (entity_name);
  }

  bool 
  BaseEntityLifeManager::exists (std::string entity_name)
  {
    return entities_.contains (entity_name);
  }

  bool 
  BaseEntityLifeManager::remove (std::string entity_name)
  {
    return entities_.remove (entity_name);
  }
  
} // end of namespace
