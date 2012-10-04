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

#include "switcher/quiddity-documentation.h"
#include "switcher/quiddity-life-manager.h"
#include "switcher/quiddity.h" 

//the base quiddities to manage (line sorted)
#include "switcher/aac.h"
#include "switcher/audio-test-source.h"
#include "switcher/ctrl-server.h"
#include "switcher/gconf-audio-sink.h"
#include "switcher/gconf-audio-source.h"
#include "switcher/gconf-video-sink.h"
#include "switcher/gconf-video-source.h"
#include "switcher/h264.h"
#include "switcher/rtpsession.h"
#include "switcher/runtime.h"
#include "switcher/video-test-source.h"
#include "switcher/xvimagesink.h"

namespace switcher
{

  QuiddityLifeManager::QuiddityLifeManager() :
    name_ ("default")
  {
    register_classes ();
  }

  QuiddityLifeManager::QuiddityLifeManager(std::string name) :
    name_ (name)
  {
    register_classes ();
  }

  std::string
  QuiddityLifeManager::get_name ()
  {
    return name_; 
  }
  
  void
  QuiddityLifeManager::register_classes ()
  {
    //registering quiddities
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
    abstract_factory_.register_class<RtpSession> (RtpSession::get_documentation().get_class_name (), 
						  RtpSession::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<Runtime> (Runtime::get_documentation().get_class_name (), 
					       Runtime::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<VideoTestSource> (VideoTestSource::get_documentation().get_class_name (),
						       VideoTestSource::get_documentation().get_json_documentation ());
    abstract_factory_.register_class<Xvimagesink> (Xvimagesink::get_documentation().get_class_name (),
						   Xvimagesink::get_documentation().get_json_documentation ());
    
  }

  std::vector<std::string> 
  QuiddityLifeManager::get_classes ()
  {
    //return abstract_factory_.get_keys ();
    return abstract_factory_.get_classes_documentation ();
  }

  bool 
  QuiddityLifeManager::class_exists (std::string class_name)
  {
    return abstract_factory_.key_exists (class_name);
  }

  Quiddity::ptr 
  QuiddityLifeManager::create (std::string quiddity_class, QuiddityLifeManager::ptr life_manager)
  {
    Quiddity::ptr quiddity = abstract_factory_.create (quiddity_class, life_manager);
    g_print ("create_quiddity %p %p\n",&quiddity,quiddity.get());
    if (quiddity.get() != NULL)
      {
     	quiddities_.insert (quiddity->get_name(),quiddity);
      }
    return quiddity;
  }

  std::vector<std::string> 
  QuiddityLifeManager::get_instances ()
  {
    return quiddities_.get_keys();
  }

  Quiddity::ptr 
  QuiddityLifeManager::get (std::string quiddity_name)
  {
    return quiddities_.lookup (quiddity_name);
  }

  bool 
  QuiddityLifeManager::exists (std::string quiddity_name)
  {
    return quiddities_.contains (quiddity_name);
  }

  bool 
  QuiddityLifeManager::remove (std::string quiddity_name)
  {
    return quiddities_.remove (quiddity_name);
  }
  
} // end of namespace
