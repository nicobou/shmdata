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

#include "switcher/base-entity-documentation.h"
#include "switcher/base-entity-life-manager.h"
#include "switcher/base-entity.h" 

//the base entities to manage (line sorted)
#include "switcher/aac.h"
#include "switcher/audio-test-source.h"
#include "switcher/ctrl-server.h"
#include "switcher/gconf-audio-sink.h"
#include "switcher/gconf-audio-source.h"
#include "switcher/gconf-video-sink.h"
#include "switcher/gconf-video-source.h"
#include "switcher/h264.h"
#include "switcher/runtime.h"
#include "switcher/video-test-source.h"
#include "switcher/xvimagesink.h"

namespace switcher
{

  BaseEntityLifeManager::BaseEntityLifeManager()
  {
    //registering base entity to make available
    abstract_factory_.register_class<AAC> (AAC::get_documentation().get_class_name (), 
					   AAC::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<AudioTestSource> (AudioTestSource::get_documentation().get_class_name (), 
						       AudioTestSource::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<CtrlServer> (CtrlServer::get_documentation().get_class_name (), 
						  CtrlServer::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<GconfAudioSink> (GconfAudioSink::get_documentation().get_class_name (), 
						      GconfAudioSink::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<GconfAudioSource> (GconfAudioSource::get_documentation().get_class_name (), 
							GconfAudioSource::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<GconfVideoSink> (GconfVideoSink::get_documentation().get_class_name (), 
						      GconfVideoSink::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<GconfVideoSource> (GconfVideoSource::get_documentation().get_class_name (),
							GconfVideoSource::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<H264> (H264::get_documentation().get_class_name (), 
					    H264::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<Runtime> (Runtime::get_documentation().get_class_name (), 
					       Runtime::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<VideoTestSource> (VideoTestSource::get_documentation().get_class_name (),
						       VideoTestSource::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<Xvimagesink> (Xvimagesink::get_documentation().get_class_name (),
						   Xvimagesink::get_documentation().get_json_documentation ());
  }

  std::vector<std::string> 
  BaseEntityLifeManager::get_classes ()
  {
    //return abstract_factory_.get_keys ();
    return abstract_factory_.get_classes_documentation ();
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
