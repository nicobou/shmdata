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
#include "switcher/udpsink.h"
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
    abstract_factory_.register_class<UDPSink> (UDPSink::get_documentation().get_class_name (), 
					       UDPSink::get_documentation().get_json_documentation ());

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

  std::string 
  QuiddityLifeManager::create (std::string quiddity_class, QuiddityLifeManager::ptr life_manager)
  {
     if(!class_exists (quiddity_class))
      return "";
    
     Quiddity::ptr quiddity = abstract_factory_.create (quiddity_class, life_manager);
     g_print ("%s: create_quiddity %p %s\n",name_.c_str(), &quiddity,quiddity->get_name ().c_str());
     if (quiddity.get() != NULL)
       {
	 quiddities_.insert (quiddity->get_name(),quiddity);
	 quiddities_nick_names_.insert (quiddity->get_nick_name (),quiddity->get_name());
       }
     return quiddity->get_nick_name ();
  }

  std::string 
  QuiddityLifeManager::create (std::string quiddity_class, std::string nick_name, QuiddityLifeManager::ptr life_manager)
  {
    if(!class_exists (quiddity_class))
      return "";

    Quiddity::ptr quiddity = abstract_factory_.create (quiddity_class, life_manager);
    g_print ("%s: create_quiddity %p %s\n",name_.c_str(), &quiddity,quiddity->get_name ().c_str());
    if (quiddity.get() != NULL)
      {
	if (!nick_name.empty () && !quiddities_nick_names_.contains (nick_name))
	  quiddity->set_nick_name (nick_name);
	else
	  g_print ("QuiddityLifeManager::create: nick name %s not valid, using %s\n",
		   nick_name.c_str (),
		   quiddity->get_name().c_str ());
     	quiddities_.insert (quiddity->get_name(),quiddity);
	quiddities_nick_names_.insert (quiddity->get_nick_name (),quiddity->get_name());
      }
    return quiddity->get_nick_name ();
  }

  std::vector<std::string> 
  QuiddityLifeManager::get_instances ()
  {
    return quiddities_nick_names_.get_keys();
  }

  Quiddity::ptr 
  QuiddityLifeManager::get_quiddity (std::string quiddity_name)
  {
     if (!exists (quiddity_name))
      {
	g_printerr ("quiddity %s not found, cannot provide ptr\n",quiddity_name.c_str());
	Quiddity::ptr empty_quiddity_ptr;
	return empty_quiddity_ptr;
      }
    return quiddities_.lookup (quiddities_nick_names_.lookup (quiddity_name));
  }

  bool 
  QuiddityLifeManager::exists (std::string quiddity_name)
  {
    return quiddities_nick_names_.contains (quiddity_name);
  }

  bool 
  QuiddityLifeManager::remove (std::string quiddity_name)
  {
    quiddities_.remove (quiddities_nick_names_.lookup (quiddity_name));
    return quiddities_nick_names_.remove (quiddity_name);
  }

  std::string 
  QuiddityLifeManager::get_properties_description (std::string quiddity_name)
  {

    if (!exists (quiddity_name))
      {
	g_printerr ("quiddity %s not found, cannot get description of properties\n",quiddity_name.c_str());
	return "";
      }
    return (get_quiddity (quiddity_name))->get_properties_description ();
  }

  std::string 
  QuiddityLifeManager::get_property_description (std::string quiddity_name, std::string property_name)
  {

    if (!exists (quiddity_name))
      {
	g_printerr ("quiddity %s not found, cannot get description of properties\n",quiddity_name.c_str());
	return "";
      }
    return (get_quiddity (quiddity_name))->get_property_description (property_name);
  }

  bool
  QuiddityLifeManager::set_property (std::string quiddity_name,
				 std::string property_name,
				 std::string property_value)
  {
    if (!exists (quiddity_name))
      {
	g_printerr ("quiddity %s not found, cannot set property\n",quiddity_name.c_str());
	return false;
      }
    return (get_quiddity (quiddity_name))->set_property(property_name.c_str(),property_value.c_str());
  }

  std::string
  QuiddityLifeManager::get_property (std::string quiddity_name,
				 std::string property_name)
  {
    if (!exists (quiddity_name))
      {
	g_printerr ("quiddity %s not found, cannot get property\n",quiddity_name.c_str());
	return "error, quiddity not found";
      }
    return (get_quiddity (quiddity_name))->get_property(property_name.c_str());
  }

  bool 
  QuiddityLifeManager::invoke (std::string quiddity_name, 
			       std::string function_name,
			       std::vector<std::string> args)
  {
    //g_print ("   QuiddityLifeManager::quiddity_invoke_method %s %s, arg size %d\n",quiddity_name.c_str(), function_name.c_str(), args.size ());
    if (!exists (quiddity_name))
      {
	g_printerr ("quiddity %s not found, cannot invoke\n",quiddity_name.c_str());
	return false;
      }
    Quiddity::ptr quiddity = get_quiddity (quiddity_name);

    int num_val = quiddity->method_get_num_value_args(function_name);

    if (num_val == -1) 
      {
	g_printerr ("function %s not found, cannot invoke\n",function_name.c_str());
	return false;
      }

    if ((int)args.size () != num_val)
      {
	g_printerr ("invoking %s/%s, number of arguments does not match\n",quiddity_name.c_str(),function_name.c_str());
	return false;
      }
    
    return quiddity->invoke_method (function_name, args);
  } 

  std::string
  QuiddityLifeManager::get_methods_description (std::string quiddity_name)
  {
    if (!exists (quiddity_name))
      {
	g_printerr ("quiddity %s not found, cannot get description of methods\n",quiddity_name.c_str());
	return "error, quiddity not found";
      }
     
    return (get_quiddity (quiddity_name))->get_methods_description ();
  }

  std::string
  QuiddityLifeManager::get_method_description (std::string quiddity_name, std::string method_name)
  {
    if (!exists (quiddity_name))
      {
	g_printerr ("quiddity %s not found, cannot get description of methods\n",quiddity_name.c_str());
	return "error, quiddity not found";
      }
     
    return (get_quiddity (quiddity_name))->get_method_description (method_name);
  }

  
} // end of namespace
