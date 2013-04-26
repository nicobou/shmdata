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

#include "quiddity-documentation.h"
#include "quiddity-life-manager.h"
#include "quiddity.h" 

//removing shmdata 
#include <gio/gio.h>

//the base quiddities to manage (line sorted)
#include "aac.h"
#include "aravis-genicam.h"
#include "audio-test-source.h"
#include "decodebin2.h"
#include "deinterleave.h"
#include "fake-shmdata-writer.h"
#include "fakesink.h"
#include "file-sdp.h"
#include "gst-parse-to-bin-src.h"
#include "gst-video-parse-to-bin-src.h"
#include "h264.h"
#include "http-sdp.h"
#include "logger.h"
#include "osc-ctrl-server.h"
#include "pulse-sink.h"
#include "rtp-session.h"
#include "runtime.h"
#include "shmdata-to-file.h"
#include "shmdata-from-gdp-file.h"
#include "soap-ctrl-server.h"
#include "udpsink.h"
#include "uridecodebin.h"
#include "uris.h"
#include "video-rate.h"
#include "video-test-source.h"
#include "vorbis.h"
#include "xvimagesink.h"

namespace switcher
{


  QuiddityLifeManager::ptr 
  QuiddityLifeManager::make_life_manager ()
  {
    QuiddityLifeManager::ptr manager(new QuiddityLifeManager);
    return manager;
  }
  
  QuiddityLifeManager::ptr 
  QuiddityLifeManager::make_life_manager (std::string name)
  {
    QuiddityLifeManager::ptr manager(new QuiddityLifeManager(name));
    return manager;
  }

  QuiddityLifeManager::QuiddityLifeManager() :
    name_ ("default")
  {
    remove_shmdata_sockets ();
    register_classes ();
    classes_doc_.reset (new JSONBuilder ());
    make_classes_doc ();
  }
  
  QuiddityLifeManager::QuiddityLifeManager(std::string name) :
    name_ (name)
  {
    remove_shmdata_sockets ();
    register_classes ();
    classes_doc_.reset (new JSONBuilder ());
    make_classes_doc ();
  }

  QuiddityLifeManager::~QuiddityLifeManager()
  {
    g_debug ("~QuiddityLifeManager");
  }
  

  void
  QuiddityLifeManager::remove_shmdata_sockets ()
  {
    
    GFile *shmdata_dir = g_file_new_for_commandline_arg (Quiddity::get_socket_dir().c_str ());

    gchar *shmdata_prefix = g_strconcat (Quiddity::get_socket_name_prefix ().c_str (), 
					 name_.c_str (), 
					 "_",
					 NULL);
    
    gboolean res;
    GError *error;
    GFileEnumerator *enumerator;
    GFileInfo *info;
    GFile *descend;
    char *relative_path;
    
    error = NULL;
    enumerator =
      g_file_enumerate_children (shmdata_dir, "*",
				 G_FILE_QUERY_INFO_NOFOLLOW_SYMLINKS, NULL,
				 &error);
    if (! enumerator)
      return;
    error = NULL;
    info = g_file_enumerator_next_file (enumerator, NULL, &error);
    while ((info) && (!error))
      {
	descend = g_file_get_child (shmdata_dir, g_file_info_get_name (info));
	//g_assert (descend != NULL);
	relative_path = g_file_get_relative_path (shmdata_dir, descend);

	
	
	error = NULL;
	
	if (g_str_has_prefix (relative_path, shmdata_prefix))
	  {
	    g_warning ("deleting previous shmdata socket (%s)", g_file_get_path (descend));
	    res = g_file_delete (descend, NULL, &error);
	    if(res != TRUE)
	      g_warning ("socket cannot be deleted");
	  }
	
	g_object_unref (descend);
	error = NULL;
	info = g_file_enumerator_next_file (enumerator, NULL, &error);
      }
    if (error != NULL)
      g_debug ("error not NULL");
    
    error = NULL;
    res = g_file_enumerator_close (enumerator, NULL, &error);
    if (res != TRUE)
      g_debug ("QuiddityLifeManager: file enumerator not properly closed");
    if (error != NULL)
      g_debug ("error not NULL");
    
    g_object_unref (shmdata_dir);
    g_free (shmdata_prefix);
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
    abstract_factory_.register_class<AAC> (AAC::doc_.get_class_name (), 
					   AAC::doc_.get_json_root_node ());
    abstract_factory_.register_class<AudioTestSource> (AudioTestSource::doc_.get_class_name (), 
      						       AudioTestSource::doc_.get_json_root_node ());
    abstract_factory_.register_class<AravisGenicam> (AravisGenicam::doc_.get_class_name (), 
      						     AravisGenicam::doc_.get_json_root_node ());
    abstract_factory_.register_class<Decodebin2> (Decodebin2::doc_.get_class_name (), 
						  Decodebin2::doc_.get_json_root_node ());
    abstract_factory_.register_class<Deinterleave> (Deinterleave::doc_.get_class_name (), 
						    Deinterleave::doc_.get_json_root_node ());
    abstract_factory_.register_class<FakeShmdataWriter> (FakeShmdataWriter::doc_.get_class_name (), 
							 FakeShmdataWriter::doc_.get_json_root_node ());
    abstract_factory_.register_class<FakeSink> (FakeSink::doc_.get_class_name (), 
						FakeSink::doc_.get_json_root_node ());
    abstract_factory_.register_class<FileSDP> (FileSDP::doc_.get_class_name (), 
					       FileSDP::doc_.get_json_root_node ());
    abstract_factory_.register_class<GstParseToBinSrc> (GstParseToBinSrc::doc_.get_class_name (),
      							GstParseToBinSrc::doc_.get_json_root_node ());
    abstract_factory_.register_class<GstVideoParseToBinSrc> (GstVideoParseToBinSrc::doc_.get_class_name (),
							     GstVideoParseToBinSrc::doc_.get_json_root_node ());
    abstract_factory_.register_class<H264> (H264::doc_.get_class_name (), 
      					    H264::doc_.get_json_root_node ());
    abstract_factory_.register_class<HTTPSDP> (HTTPSDP::doc_.get_class_name (), 
      					       HTTPSDP::doc_.get_json_root_node ());
    abstract_factory_.register_class<Logger> (Logger::doc_.get_class_name (), 
      					       Logger::doc_.get_json_root_node ());
    abstract_factory_.register_class<OscCtrlServer> (OscCtrlServer::doc_.get_class_name (), 
						     OscCtrlServer::doc_.get_json_root_node ());
    abstract_factory_.register_class<PulseSink> (PulseSink::doc_.get_class_name (), 
      						 PulseSink::doc_.get_json_root_node ());
    abstract_factory_.register_class<RtpSession> (RtpSession::doc_.get_class_name (), 
      						  RtpSession::doc_.get_json_root_node ());
    abstract_factory_.register_class<Runtime> (Runtime::doc_.get_class_name (), 
      					       Runtime::doc_.get_json_root_node ());
    abstract_factory_.register_class<ShmdataFromGDPFile> (ShmdataFromGDPFile::doc_.get_class_name (), 
							  ShmdataFromGDPFile::doc_.get_json_root_node ());
    abstract_factory_.register_class<ShmdataToFile> (ShmdataToFile::doc_.get_class_name (), 
						     ShmdataToFile::doc_.get_json_root_node ());
    abstract_factory_.register_class<SoapCtrlServer> (SoapCtrlServer::doc_.get_class_name (), 
     						      SoapCtrlServer::doc_.get_json_root_node ());
    abstract_factory_.register_class<UDPSink> (UDPSink::doc_.get_class_name (), 
      					       UDPSink::doc_.get_json_root_node ());
    abstract_factory_.register_class<Uridecodebin> (Uridecodebin::doc_.get_class_name (), 
      						    Uridecodebin::doc_.get_json_root_node ());
    abstract_factory_.register_class<Uris> (Uris::doc_.get_class_name (), 
					    Uris::doc_.get_json_root_node ());
    abstract_factory_.register_class<VideoRate> (VideoRate::doc_.get_class_name (),
						 VideoRate::doc_.get_json_root_node ());
    abstract_factory_.register_class<VideoTestSource> (VideoTestSource::doc_.get_class_name (),
       						       VideoTestSource::doc_.get_json_root_node ());
    abstract_factory_.register_class<Vorbis> (Vorbis::doc_.get_class_name (),
					      Vorbis::doc_.get_json_root_node ());
    abstract_factory_.register_class<Xvimagesink> (Xvimagesink::doc_.get_class_name (),
      						   Xvimagesink::doc_.get_json_root_node ());
  }

  std::vector<std::string> 
  QuiddityLifeManager::get_classes ()
  {
    //return abstract_factory_.get_classes_documentation ();
    return abstract_factory_.get_keys ();
  }

  void
  QuiddityLifeManager::make_classes_doc ()
  {
    std::vector<JSONBuilder::Node> docs = abstract_factory_.get_classes_documentation ();
    classes_doc_->reset ();
    classes_doc_->begin_object ();
    classes_doc_->set_member_name ("classes");
    classes_doc_->begin_array ();
    for(std::vector<JSONBuilder::Node>::iterator it = docs.begin(); it != docs.end(); ++it) 
      classes_doc_->add_node_value (*it);
    classes_doc_->end_array ();
    classes_doc_->end_object ();
  }

  std::string 
  QuiddityLifeManager::get_classes_doc ()
  {
    return classes_doc_->get_string (true);
  }
  
  std::string 
  QuiddityLifeManager::get_class_doc (std::string class_name)
  {
    if (abstract_factory_.key_exists (class_name))
      {
	JSONBuilder::Node doc = abstract_factory_.get_class_documentation (class_name);
	return JSONBuilder::get_string (doc, true);
      }
    else
      return "{ \"error\":\"class not found\" }";
  }

  bool 
  QuiddityLifeManager::class_exists (std::string class_name)
  {
    return abstract_factory_.key_exists (class_name);
  }


  bool 
  QuiddityLifeManager::init_quiddity (Quiddity::ptr quiddity)
  {
    quiddity->set_life_manager (shared_from_this());
    if (!quiddity->init ())
      return false;
    // g_critical ("QuiddityLifeManager: intialization of %s (%s) return false",
    // 	       quiddity->get_name ().c_str (),
    // 	       quiddity->get_documentation ().get_class_name ().c_str ());
    quiddities_.insert (quiddity->get_name(),quiddity);
    quiddities_nick_names_.insert (quiddity->get_nick_name (),quiddity->get_name());
    return true;
  }

  std::string 
  QuiddityLifeManager::create (std::string quiddity_class)
  {
     if(!class_exists (quiddity_class))
      return "";
    
     Quiddity::ptr quiddity = abstract_factory_.create (quiddity_class);
     if (quiddity.get() != NULL)
       if (!init_quiddity (quiddity))
	 {
	   g_warning ("initialization of %s failled",quiddity_class.c_str ());
	   return "";
	 }
     
     g_message ("(%s) quiddity %s created (%s)",
		name_.c_str(), 
		quiddity->get_nick_name ().c_str (), 
		quiddity->get_name ().c_str ());

     return quiddity->get_nick_name ();
  }

  std::string 
  QuiddityLifeManager::create (std::string quiddity_class, std::string nick_name)
  {
    if(!class_exists (quiddity_class))
      return "";

    Quiddity::ptr quiddity = abstract_factory_.create (quiddity_class);

    if (quiddity.get() != NULL)
      {
	if (!nick_name.empty () && !quiddities_nick_names_.contains (nick_name))
	  quiddity->set_nick_name (nick_name);
	else
	  g_warning ("QuiddityLifeManager::create: nick name %s not valid, using %s",
		   nick_name.c_str (),
		   quiddity->get_name().c_str ());

	if (!init_quiddity (quiddity))
	  {
	    g_warning ("initialization of %s with name %s failled\n",
		       quiddity_class.c_str (), nick_name.c_str ());
	    
	    return "";
	  }

     g_message ("(%s) quiddity %s created (%s)",
		name_.c_str(), 
		quiddity->get_nick_name ().c_str (), 
		quiddity->get_name ().c_str ());

      }
    return quiddity->get_nick_name ();
  }

  std::vector<std::string> 
  QuiddityLifeManager::get_instances ()
  {
    return quiddities_nick_names_.get_keys();
  }

  std::string 
  QuiddityLifeManager::get_quiddities_description ()
  {
    //FIXME get json doc from the quiddity class and make an array of it 
    JSONBuilder::ptr descr (new JSONBuilder ());
    descr->reset ();
    descr->begin_object();
    descr->set_member_name ("quiddities");
    descr->begin_array();
    std::vector<std::string> quids = get_instances (); 
    for(std::vector<std::string>::iterator it = quids.begin(); it != quids.end(); ++it) 
      {
	descr->begin_object();
	std::shared_ptr<Quiddity> quid = get_quiddity (*it);
	descr->add_string_member ("name", quid->get_nick_name().c_str ());
	descr->add_string_member ("class", quid->get_documentation().get_class_name().c_str ());
	descr->end_object();

      }

    descr->end_array();
    descr->end_object ();

    return descr->get_string(true);
  }

  std::string 
  QuiddityLifeManager::get_quiddity_description (std::string nick_name)
  {
    if (!quiddities_nick_names_.contains (nick_name))
      return "{ \"error\":\"quiddity not found\"}";

    JSONBuilder::ptr descr (new JSONBuilder ());
    descr->reset ();
    descr->begin_object();
    descr->add_string_member ("name", nick_name.c_str ());
    descr->add_string_member ("class", quiddities_.lookup(quiddities_nick_names_.lookup (nick_name))->get_documentation().get_class_name().c_str ());
    descr->end_object ();

    return descr->get_string(true);
  }

  Quiddity::ptr 
  QuiddityLifeManager::get_quiddity (std::string quiddity_name)
  {
     if (!exists (quiddity_name))
      {
	g_critical ("quiddity %s not found, cannot provide ptr",quiddity_name.c_str());
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
    if (exists (quiddity_name))
      quiddities_.remove (quiddities_nick_names_.lookup (quiddity_name));
    
    if (quiddities_nick_names_.remove (quiddity_name))
      {  
        g_message ("(%s) quiddity removed (%s)",name_.c_str(), quiddity_name.c_str());
	return true;
      }
    g_warning ("(%s) quiddity %s not found for removing",name_.c_str(), quiddity_name.c_str());
    return false; 
  }

  std::string 
  QuiddityLifeManager::get_properties_description (std::string quiddity_name)
  {

    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get description of properties",quiddity_name.c_str());
	return "";
      }
    return (get_quiddity (quiddity_name))->get_properties_description ();
  }

  std::string 
  QuiddityLifeManager::get_property_description (std::string quiddity_name, std::string property_name)
  {

    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get description of properties",quiddity_name.c_str());
	return "";
      }
    return (get_quiddity (quiddity_name))->get_property_description (property_name);
  }

  std::string 
  QuiddityLifeManager::get_properties_description_by_class (std::string class_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create (class_name);
    if (g_strcmp0 (quid_name.c_str (), "") == 0)
      return "{\"error\":\"cannot get property because the class cannot be instanciated\"}";
    std::string descr = get_properties_description (quid_name);
    remove (quid_name);
    return descr;
  }
  
  std::string
  QuiddityLifeManager::get_property_description_by_class (std::string class_name, 
							  std::string property_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create (class_name);
    if (g_strcmp0 (quid_name.c_str (), "") == 0)
      return "{\"error\":\"cannot get property because the class cannot be instanciated\"}";
    std::string descr = get_property_description (quid_name, property_name);
    remove (quid_name);
    return descr;
  }


  bool
  QuiddityLifeManager::set_property (std::string quiddity_name,
				 std::string property_name,
				 std::string property_value)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot set property",quiddity_name.c_str());
	return false;
      }
    return (get_quiddity (quiddity_name))->set_property(property_name.c_str(),property_value.c_str());
  }

 //higher level subscriber
  bool 
  QuiddityLifeManager::make_subscriber (std::string subscriber_name,
					QuiddityPropertySubscriber::Callback cb,
					void *user_data)
  {
    if (property_subscribers_.contains (subscriber_name))
      {
	g_warning ("QuiddityLifeManager, a subscriber named %s already exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    
    QuiddityPropertySubscriber::ptr subscriber;
    subscriber.reset (new QuiddityPropertySubscriber());
    subscriber->set_life_manager (shared_from_this());
    subscriber->set_name (subscriber_name.c_str ());
    subscriber->set_user_data (user_data);
    subscriber->set_callback (cb);
    property_subscribers_.insert (subscriber_name, subscriber);
    return true; 
  }

  bool
  QuiddityLifeManager::remove_subscriber (std::string subscriber_name)
  {
    if (!property_subscribers_.contains (subscriber_name))
      {
	g_warning ("QuiddityLifeManager, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    return property_subscribers_.remove (subscriber_name);
  }
  
  bool 
  QuiddityLifeManager::subscribe_property (std::string subscriber_name,
					   std::string quiddity_name,
					   std::string property_name)
  {
    if (!property_subscribers_.contains (subscriber_name))
      {
	g_warning ("QuiddityLifeManager, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot subscribe to property",quiddity_name.c_str());
	return false;
      }
    return property_subscribers_.lookup(subscriber_name)->subscribe (get_quiddity (quiddity_name), property_name);
  }
  
  bool 
  QuiddityLifeManager::unsubscribe_property (std::string subscriber_name,
					     std::string quiddity_name,
					     std::string property_name)
  {
    if (!property_subscribers_.contains (subscriber_name))
      {
	g_warning ("QuiddityLifeManager, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot subscribe to property",quiddity_name.c_str());
	return false;
      }
    return property_subscribers_.lookup(subscriber_name)->unsubscribe (get_quiddity (quiddity_name), property_name);
  }

  std::vector<std::string> 
  QuiddityLifeManager::list_subscribers ()
  {
    return property_subscribers_.get_keys ();
  }

    std::vector<std::pair<std::string, std::string> > 
    QuiddityLifeManager::list_subscribed_properties (std::string subscriber_name)
    {
      if (!property_subscribers_.contains (subscriber_name))
	{
	  g_warning ("QuiddityLifeManager, a subscriber named %s does not exists\n",
		     subscriber_name.c_str ());
	  std::vector<std::pair<std::string, std::string> > empty;
	  return empty;
	}
      
      return property_subscribers_.lookup(subscriber_name)->list_subscribed_properties ();
    }

    std::string 
    QuiddityLifeManager::list_subscribers_json ()
    {
      return "{\"error\":\"to be implemented\"}";//FIXME (list_subscriber_json)
    }
  
    std::string 
    QuiddityLifeManager::list_subscribed_properties_json (std::string subscriber_name)
    {
      std::vector<std::pair<std::string, std::string> > subscribed_props =
	list_subscribed_properties (subscriber_name);

      JSONBuilder *doc = new JSONBuilder ();
      doc->reset ();
      doc->begin_object ();
      doc->set_member_name ("subscribed properties");
      doc->begin_array ();
      for(std::vector<std::pair<std::string, std::string> >::iterator it = subscribed_props.begin(); 
	  it != subscribed_props.end(); ++it) 
	{
	  doc->begin_object ();
	  doc->add_string_member ("quiddity", it->first.c_str ());
	  doc->add_string_member ("property", it->second.c_str ());
	  doc->end_object ();
	}
      doc->end_array ();
      doc->end_object ();
      
      return doc->get_string(true);
    }

  //lower level subscriber
  bool
  QuiddityLifeManager::subscribe_property_glib (std::string quiddity_name,
						std::string property_name,
						Property::Callback cb, 
						void *user_data)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot subscribe to property",quiddity_name.c_str());
	return false;
      }
    return (get_quiddity (quiddity_name))->subscribe_property(property_name.c_str(),
							      cb,
							      user_data);
  }

  bool
  QuiddityLifeManager::unsubscribe_property_glib (std::string quiddity_name,
						  std::string property_name,
						  Property::Callback cb, 
						  void *user_data)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot unsubscribe to property",quiddity_name.c_str());
	return false;
      }
    return (get_quiddity (quiddity_name))->unsubscribe_property(property_name.c_str(),
								cb,
								user_data);
  }


  std::string
  QuiddityLifeManager::get_property (std::string quiddity_name,
				 std::string property_name)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get property",quiddity_name.c_str());
	return "error, quiddity not found";
      }
    return (get_quiddity (quiddity_name))->get_property(property_name.c_str());
  }

  bool 
  QuiddityLifeManager::invoke (std::string quiddity_name, 
			       std::string method_name,
			       std::vector<std::string> args)
  {
    //g_debug ("QuiddityLifeManager::quiddity_invoke_method %s %s, arg size %d",quiddity_name.c_str(), function_name.c_str(), args.size ());
    
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot invoke",quiddity_name.c_str());
	return false;
      }
    Quiddity::ptr quiddity = get_quiddity (quiddity_name);

    //FIXME implement and use "quiddity->has_method(method_name)" 
    int num_val = quiddity->method_get_num_value_args(method_name);
    if (num_val == -1) 
      {
	g_warning ("method %s not found, cannot invoke",method_name.c_str());
	return false;
      }

    return quiddity->invoke_method (method_name, args);
  } 

  std::string
  QuiddityLifeManager::get_methods_description (std::string quiddity_name)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get description of methods",quiddity_name.c_str());
	return "error, quiddity not found";
      }
     
    return (get_quiddity (quiddity_name))->get_methods_description ();
  }


  std::string
  QuiddityLifeManager::get_method_description (std::string quiddity_name, std::string method_name)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get description of methods",quiddity_name.c_str());
	return "error, quiddity not found";
      }
     
    return (get_quiddity (quiddity_name))->get_method_description (method_name);
  }

  std::string
  QuiddityLifeManager::get_methods_description_by_class (std::string class_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create (class_name);
    std::string descr = get_methods_description (quid_name);
    remove (quid_name);
    return descr;

  }

  std::string
  QuiddityLifeManager::get_method_description_by_class (std::string class_name, 
							std::string method_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create (class_name);
    std::string descr = get_method_description (quid_name, method_name);
    remove (quid_name);
    return descr;

  }

  
} // end of namespace
