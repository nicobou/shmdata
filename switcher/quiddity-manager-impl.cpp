/*
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

//#include <gmodule.h>

#include "quiddity-documentation.h"
#include "quiddity-manager-impl.h"
#include "quiddity.h" 

//removing shmdata 
#include <gio/gio.h>

//the quiddities to manage (line sorted)
#include "aravis-genicam.h"
#include "audio-test-source.h"
#include "create-remove-spy.h"
#include "decodebin2.h"
#include "deinterleave.h"
#include "fake-shmdata-writer.h"
#include "fakesink.h"
#include "file-sdp.h"
#include "gst-parse-to-bin-src.h"
#include "gst-video-parse-to-bin-src.h"
#include "http-sdp.h"
#include "http-sdp-dec.h"
#include "jack-audio-source.h"
#include "jack-sink.h"
#include "logger.h"
#include "property-mapper.h"
#include "rtp-session.h"
#include "shmdata-to-file.h"
#include "shmdata-from-gdp-file.h"
#include "udpsink.h"
#include "uridecodebin.h"
#include "string-dictionary.h"
#include "video-test-source.h"
#include "xvimagesink.h"

namespace switcher
{

  QuiddityManager_Impl::ptr 
  QuiddityManager_Impl::make_manager ()
  {
    QuiddityManager_Impl::ptr manager(new QuiddityManager_Impl("default"));
    return manager;
  }
  
  QuiddityManager_Impl::ptr 
  QuiddityManager_Impl::make_manager (const std::string &name)
  {
    QuiddityManager_Impl::ptr manager(new QuiddityManager_Impl(name));
    return manager;
  }

  QuiddityManager_Impl::QuiddityManager_Impl(const std::string &name) :
      plugins_ (),
      name_ (name),
      abstract_factory_ (),
      quiddities_ (),
      quiddities_nick_names_ (),
      property_subscribers_ (),
      signal_subscribers_ (),
      classes_doc_ (new JSONBuilder ()),
      creation_hook_ (nullptr),
      removal_hook_ (nullptr),
      creation_hook_user_data_ (nullptr),
      removal_hook_user_data_ (nullptr),
      quiddity_created_counter_ (0), 
      thread_ (),
      main_context_ (nullptr),
      mainloop_ (nullptr) 
  {
    init_gmainloop ();
    remove_shmdata_sockets (); 
    register_classes ();
    make_classes_doc ();
  }

  QuiddityManager_Impl::~QuiddityManager_Impl()
  {
    g_main_loop_quit (mainloop_);
    g_main_context_unref (main_context_);
    if (thread_.joinable ())
      thread_.join ();      
  }

  void
  QuiddityManager_Impl::remove_shmdata_sockets ()
  {
   
    GFile *shmdata_dir = g_file_new_for_commandline_arg (Quiddity::get_socket_dir().c_str ());

    gchar *shmdata_prefix = g_strconcat (Quiddity::get_socket_name_prefix ().c_str (), 
					 name_.c_str (), 
					 "_",
					 nullptr);
    
    gboolean res;
    GError *error;
    GFileEnumerator *enumerator;
    GFileInfo *info;
    GFile *descend;
    char *relative_path;
    
    error = nullptr;
    enumerator =
      g_file_enumerate_children (shmdata_dir, "*",
				 G_FILE_QUERY_INFO_NOFOLLOW_SYMLINKS, nullptr,
				 &error);
    if (! enumerator)
      return;
    error = nullptr;
    info = g_file_enumerator_next_file (enumerator, nullptr, &error);
    while ((info) && (!error))
      {
	descend = g_file_get_child (shmdata_dir, g_file_info_get_name (info));
	//g_assert (descend != nullptr);
	relative_path = g_file_get_relative_path (shmdata_dir, descend);
	
	error = nullptr;
	
	if (g_str_has_prefix (relative_path, shmdata_prefix))
	  {
	    g_debug ("deleting previous shmdata socket (%s)", g_file_get_path (descend));
	    res = g_file_delete (descend, nullptr, &error);
	    if(res != TRUE)
	      g_warning ("previous switcher file \"%s\" cannot be deleted", g_file_get_path (descend));
	  }
	
	g_object_unref (descend);
	error = nullptr;
	info = g_file_enumerator_next_file (enumerator, nullptr, &error);
      }
    if (error != nullptr)
      g_debug ("error not nullptr");
    
    error = nullptr;
    res = g_file_enumerator_close (enumerator, nullptr, &error);
    if (res != TRUE)
      g_debug ("QuiddityManager_Impl: file enumerator not properly closed");
    if (error != nullptr)
      g_debug ("error not nullptr");
    
    g_object_unref (shmdata_dir);
    g_free (shmdata_prefix);
  }
  
  std::string
  QuiddityManager_Impl::get_name ()
  {
    return name_; 
  }
  
  void
  QuiddityManager_Impl::register_classes ()
  {
    //registering quiddities
    abstract_factory_.register_class<AudioTestSource> (AudioTestSource::switcher_doc_.get_class_name (), 
      						       AudioTestSource::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<AravisGenicam> (AravisGenicam::switcher_doc_.get_class_name (), 
      						     AravisGenicam::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<CreateRemoveSpy> (CreateRemoveSpy::switcher_doc_.get_class_name (), 
						       CreateRemoveSpy::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<Decodebin2> (Decodebin2::switcher_doc_.get_class_name (), 
						  Decodebin2::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<Deinterleave> (Deinterleave::switcher_doc_.get_class_name (), 
						    Deinterleave::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<FakeShmdataWriter> (FakeShmdataWriter::switcher_doc_.get_class_name (), 
							 FakeShmdataWriter::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<FakeSink> (FakeSink::switcher_doc_.get_class_name (), 
						FakeSink::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<FileSDP> (FileSDP::switcher_doc_.get_class_name (), 
					       FileSDP::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<GstParseToBinSrc> (GstParseToBinSrc::switcher_doc_.get_class_name (),
      							GstParseToBinSrc::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<GstVideoParseToBinSrc> (GstVideoParseToBinSrc::switcher_doc_.get_class_name (),
							     GstVideoParseToBinSrc::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<HTTPSDP> (HTTPSDP::switcher_doc_.get_class_name (), 
      					       HTTPSDP::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<HTTPSDPDec> (HTTPSDPDec::switcher_doc_.get_class_name (), 
      					       HTTPSDPDec::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<JackAudioSource> (JackAudioSource::switcher_doc_.get_class_name (), 
						       JackAudioSource::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<JackSink> (JackSink::switcher_doc_.get_class_name (), 
						JackSink::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<Logger> (Logger::switcher_doc_.get_class_name (), 
      					       Logger::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<PropertyMapper> (PropertyMapper::switcher_doc_.get_class_name (), 
						      PropertyMapper::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<RtpSession> (RtpSession::switcher_doc_.get_class_name (), 
      						  RtpSession::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<ShmdataFromGDPFile> (ShmdataFromGDPFile::switcher_doc_.get_class_name (), 
      							  ShmdataFromGDPFile::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<ShmdataToFile> (ShmdataToFile::switcher_doc_.get_class_name (), 
      						     ShmdataToFile::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<StringDictionary> (StringDictionary::switcher_doc_.get_class_name (), 
							StringDictionary::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<UDPSink> (UDPSink::switcher_doc_.get_class_name (), 
       					       UDPSink::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<Uridecodebin> (Uridecodebin::switcher_doc_.get_class_name (), 
						    Uridecodebin::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<VideoTestSource> (VideoTestSource::switcher_doc_.get_class_name (),
						       VideoTestSource::switcher_doc_.get_json_root_node ());
    abstract_factory_.register_class<Xvimagesink> (Xvimagesink::switcher_doc_.get_class_name (),
       						   Xvimagesink::switcher_doc_.get_json_root_node ());
  }

  std::vector<std::string> 
  QuiddityManager_Impl::get_classes ()
  {
    //return abstract_factory_.get_classes_documentation ();
    return abstract_factory_.get_keys ();
  }

  void
  QuiddityManager_Impl::make_classes_doc ()
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
  QuiddityManager_Impl::get_classes_doc ()
  {
    return classes_doc_->get_string (true);
  }
  
  std::string 
  QuiddityManager_Impl::get_class_doc (std::string class_name)
  {
    if (abstract_factory_.key_exists (class_name))
      return JSONBuilder::get_string (abstract_factory_.get_class_documentation (class_name), true);

    return "{ \"error\":\"class not found\" }";
  }

  bool 
  QuiddityManager_Impl::class_exists (std::string class_name)
  {
    return abstract_factory_.key_exists (class_name);
  }


  void 
  QuiddityManager_Impl::give_name_if_unnamed (Quiddity::ptr quiddity)
  {
    //if no name was given, give one, should eventually be the prefered way
    if (g_strcmp0 (quiddity->get_name().c_str (), "") == 0)
      {
	gchar *name = g_strdup_printf ("%s%u",
				       quiddity->get_documentation ().get_class_name ().c_str (),
				       quiddity_created_counter_);
	quiddity->set_name (name);
	g_free (name);
      }
    quiddity_created_counter_++;
  }

  bool 
  QuiddityManager_Impl::init_quiddity (Quiddity::ptr quiddity)
  {
    quiddity->set_manager_impl (shared_from_this());

    give_name_if_unnamed (quiddity);

    if (!quiddity->init ())
      return false;

    quiddities_[quiddity->get_name()] = quiddity;
    quiddities_nick_names_[quiddity->get_nick_name ()] = quiddity->get_name();
    
    if (creation_hook_ != nullptr)
      (*creation_hook_) (quiddity->get_nick_name (), creation_hook_user_data_);

    return true;
  }

  //for use of the "get description by class" methods 
  std::string 
  QuiddityManager_Impl::create_without_hook (std::string quiddity_class)
  {
    if(!class_exists (quiddity_class))
      return "";
    
    Quiddity::ptr quiddity = abstract_factory_.create (quiddity_class);
    if (quiddity.get() != nullptr)
      {
	quiddity->set_manager_impl (shared_from_this());
	give_name_if_unnamed (quiddity);
	
	if (!quiddity->init ())
	  return "{\"error\":\"cannot init quiddity class\"}";

	quiddities_[quiddity->get_name()] = quiddity;
	quiddities_nick_names_ [quiddity->get_nick_name ()] = quiddity->get_name();
      }
    return quiddity->get_nick_name ();
  }

  std::string 
  QuiddityManager_Impl::create (std::string quiddity_class)
  {
     if(!class_exists (quiddity_class))
      return "";
    
     Quiddity::ptr quiddity = abstract_factory_.create (quiddity_class);
     if (quiddity.get() != nullptr)
       if (!init_quiddity (quiddity))
	 {
	   g_debug ("initialization of %s failled",quiddity_class.c_str ());
	   return "";
	 }
     return quiddity->get_nick_name ();
  }

  std::string 
  QuiddityManager_Impl::create (std::string quiddity_class, 
				std::string nick_name)
  {
    if(!class_exists (quiddity_class))
      return "";
    
    auto it = quiddities_nick_names_.find (nick_name);
    if (quiddities_nick_names_.end () != it)
      return "";
    
    Quiddity::ptr quiddity = abstract_factory_.create (quiddity_class);

    if (quiddity.get() != nullptr)
      {
	if (!nick_name.empty ())
	  quiddity->set_nick_name (nick_name);
	else
	  {
	    g_debug ("QuiddityManager_Impl::create: nick name \"%s\" not valid",
		     nick_name.c_str ());
	    return "";
	  }

	if (!init_quiddity (quiddity))
	  {
	    g_debug ("initialization of %s with name %s failled",
		     quiddity_class.c_str (), nick_name.c_str ());
	    
	    return "";
	  }
      }
    return quiddity->get_nick_name ();
  }

  bool 
  QuiddityManager_Impl::rename (std::string nick_name, 
				std::string new_nick_name)
  {
    auto it = quiddities_nick_names_.find (nick_name);
    if (quiddities_nick_names_.end () != it)
      {
	g_debug ("cannot rename because no quiddity is nick named %s",
		 nick_name.c_str ());
	return false;
      }
    
    it = quiddities_nick_names_.find (new_nick_name);
    if (quiddities_nick_names_.end () != it)
      {
	g_debug ("cannot rename because %s is already taken",
		 new_nick_name.c_str ());
	return false;
      }
       
    Quiddity::ptr temp = get_quiddity (nick_name);
    temp->set_nick_name (new_nick_name);
    quiddities_nick_names_[new_nick_name] = temp->get_name();
    quiddities_nick_names_.erase (nick_name);
    return true;
  }


  std::vector<std::string> 
  QuiddityManager_Impl::get_instances ()
  {
    std::vector<std::string> res;
    for (auto &it : quiddities_nick_names_)
      res.push_back (it.first);
    return res;
  }
  
  std::string 
  QuiddityManager_Impl::get_quiddities_description ()
  {
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
	descr->add_string_member ("class", quid->get_documentation ().get_class_name ().c_str ());
	descr->add_string_member ("category", quid->get_documentation ().get_category ().c_str ());
	descr->add_string_member ("long name", quid->get_documentation ().get_long_name ().c_str ());
	descr->add_string_member ("description", quid->get_documentation().get_description().c_str ());
	descr->add_string_member ("license", quid->get_documentation ().get_license ().c_str ()); 
	descr->add_string_member ("author", quid->get_documentation ().get_author ().c_str ());
	descr->end_object();

      }

    descr->end_array();
    descr->end_object ();

    return descr->get_string(true);
  }

  std::string 
  QuiddityManager_Impl::get_quiddity_description (std::string nick_name)
  {
    auto it = quiddities_nick_names_.find (nick_name);
    if (quiddities_nick_names_.end () == it)
      return "{ \"error\":\"quiddity not found\"}";

    JSONBuilder::ptr descr (new JSONBuilder ());
    descr->reset ();
    descr->begin_object();
    descr->add_string_member ("name", nick_name.c_str ());
    //FIXME should use json node
    descr->add_string_member ("class", quiddities_[quiddities_nick_names_[nick_name]]->get_documentation ().get_class_name ().c_str ());
    descr->add_string_member ("category", quiddities_[quiddities_nick_names_[nick_name]]->get_documentation ().get_category ().c_str ());
    descr->add_string_member ("long name", quiddities_[quiddities_nick_names_[nick_name]]->get_documentation ().get_long_name ().c_str ());
    descr->add_string_member ("description", quiddities_[quiddities_nick_names_[nick_name]]->get_documentation().get_description().c_str ());
    descr->add_string_member ("license", quiddities_[quiddities_nick_names_[nick_name]]->get_documentation ().get_license ().c_str ()); 
    descr->add_string_member ("author", quiddities_[quiddities_nick_names_[nick_name]]->get_documentation ().get_author ().c_str ());
    descr->end_object ();
    return descr->get_string(true);
  }

  Quiddity::ptr 
  QuiddityManager_Impl::get_quiddity (std::string quiddity_name)
  {
     if (!exists (quiddity_name))
      {
	g_debug ("quiddity %s not found, cannot provide ptr",quiddity_name.c_str());
	Quiddity::ptr empty_quiddity_ptr;
	return empty_quiddity_ptr;
      }
    return quiddities_[quiddities_nick_names_[quiddity_name]];
  }

  bool 
  QuiddityManager_Impl::exists (std::string quiddity_name)
  {
    return (quiddities_nick_names_.end () != quiddities_nick_names_.find  (quiddity_name));
  }

  //for use of "get description by class" methods only
  bool 
  QuiddityManager_Impl::remove_without_hook (std::string quiddity_name)
  {
    if (!exists (quiddity_name))
      {
	g_debug ("(%s) quiddity %s not found for removing",name_.c_str(), quiddity_name.c_str());
	return false; 
      }
    for (auto &it : property_subscribers_)
      it.second->unsubscribe (get_quiddity (quiddity_name));
    for (auto &it : signal_subscribers_)
      it.second->unsubscribe (get_quiddity (quiddity_name));
    quiddities_.erase (quiddities_nick_names_[quiddity_name]);
    quiddities_nick_names_.erase (quiddity_name);
    return true;
  }

  bool 
  QuiddityManager_Impl::remove (std::string quiddity_name)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("(%s) quiddity %s not found for removing",name_.c_str(), quiddity_name.c_str());
	return false; 
      }
    for (auto &it : property_subscribers_)
      it.second->unsubscribe (get_quiddity (quiddity_name));
    for (auto &it : signal_subscribers_)
      it.second->unsubscribe (get_quiddity (quiddity_name));
    quiddities_.erase (quiddities_nick_names_[quiddity_name]);
    quiddities_nick_names_.erase (quiddity_name);
    if (removal_hook_ != nullptr)
      (*removal_hook_) (quiddity_name.c_str (), removal_hook_user_data_);
    return true;
  }

  std::string 
  QuiddityManager_Impl::get_properties_description (std::string quiddity_name)
  {

    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get description of properties",quiddity_name.c_str());
	return "";
      }
    return (get_quiddity (quiddity_name))->get_properties_description ();
  }

  std::string 
  QuiddityManager_Impl::get_property_description (std::string quiddity_name, std::string property_name)
  {

    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get description of properties",quiddity_name.c_str());
	return "";
      }
    return (get_quiddity (quiddity_name))->get_property_description (property_name);
  }

  std::string 
  QuiddityManager_Impl::get_properties_description_by_class (std::string class_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create_without_hook (class_name);
    if (g_strcmp0 (quid_name.c_str (), "") == 0)
      return "{\"error\":\"cannot get property because the class cannot be instanciated\"}";
    std::string descr = get_properties_description (quid_name);
    remove_without_hook (quid_name);
    return descr;
  }
  
  std::string
  QuiddityManager_Impl::get_property_description_by_class (std::string class_name, 
							  std::string property_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create_without_hook (class_name);
    if (g_strcmp0 (quid_name.c_str (), "") == 0)
      return "{\"error\":\"cannot get property because the class cannot be instanciated\"}";
    std::string descr = get_property_description (quid_name, property_name);
    remove_without_hook (quid_name);
    return descr;
  }


  bool
  QuiddityManager_Impl::set_property (std::string quiddity_name,
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
  QuiddityManager_Impl::make_property_subscriber (std::string subscriber_name,
					QuiddityPropertySubscriber::Callback cb,
					void *user_data)
  {
    if (property_subscribers_.end () != property_subscribers_.find (subscriber_name))
      {
	g_warning ("QuiddityManager_Impl, a subscriber named %s already exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    
    QuiddityPropertySubscriber::ptr subscriber;
    subscriber.reset (new QuiddityPropertySubscriber());
    subscriber->set_manager_impl (shared_from_this());
    subscriber->set_name (subscriber_name.c_str ());
    subscriber->set_user_data (user_data);
    subscriber->set_callback (cb);
    property_subscribers_[subscriber_name] = subscriber;
    return true; 
  }

  bool
  QuiddityManager_Impl::remove_property_subscriber (std::string subscriber_name)
  {
    auto it = property_subscribers_.find (subscriber_name);
    if (property_subscribers_.end () == it)
      {
	g_warning ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    property_subscribers_.erase (it);
    return true;
  }
  
  bool 
  QuiddityManager_Impl::subscribe_property (std::string subscriber_name,
					   std::string quiddity_name,
					   std::string property_name)
  {
    auto it = property_subscribers_.find (subscriber_name);
    if (property_subscribers_.end () == it)
      {
	g_warning ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot subscribe to property",quiddity_name.c_str());
	return false;
      }
    return property_subscribers_[subscriber_name]->subscribe (get_quiddity (quiddity_name), property_name);
  }
  
  bool 
  QuiddityManager_Impl::unsubscribe_property (std::string subscriber_name,
					      std::string quiddity_name,
					      std::string property_name)
  {
    auto it = property_subscribers_.find (subscriber_name);
    if (property_subscribers_.end () == it)
      {
	g_warning ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot subscribe to property",quiddity_name.c_str());
	return false;
      }
    return property_subscribers_[subscriber_name]->unsubscribe (get_quiddity (quiddity_name), property_name);
  }
  
  std::vector<std::string> 
  QuiddityManager_Impl::list_property_subscribers ()
  {
    std::vector<std::string> res;
    for (auto &it : property_subscribers_)
      res.push_back (it.first);
     return res;
  }

    std::vector<std::pair<std::string, std::string> > 
    QuiddityManager_Impl::list_subscribed_properties (std::string subscriber_name)
    {
      auto it = property_subscribers_.find (subscriber_name);
      if (property_subscribers_.end () == it)
	{
	  g_warning ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
		     subscriber_name.c_str ());
	  std::vector<std::pair<std::string, std::string> > empty;
	  return empty;
	}
      return property_subscribers_[subscriber_name]->list_subscribed_properties ();
    }

    std::string 
    QuiddityManager_Impl::list_property_subscribers_json ()
    {
      return "{\"error\":\"to be implemented\"}";//FIXME (list_property_subscriber_json)
    }
  
    std::string 
    QuiddityManager_Impl::list_subscribed_properties_json (std::string subscriber_name)
    {
      auto subscribed_props = list_subscribed_properties (subscriber_name);

      JSONBuilder *doc = new JSONBuilder ();
      doc->reset ();
      doc->begin_object ();
      doc->set_member_name ("subscribed properties");
      doc->begin_array ();
      for(auto &it : subscribed_props) 
	{
	  doc->begin_object ();
	  doc->add_string_member ("quiddity", it.first.c_str ());
	  doc->add_string_member ("property", it.second.c_str ());
	  doc->end_object ();
	}
      doc->end_array ();
      doc->end_object ();
      
      return doc->get_string(true);
    }

  //lower level subscriber
  bool
  QuiddityManager_Impl::subscribe_property_glib (std::string quiddity_name,
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
  QuiddityManager_Impl::unsubscribe_property_glib (std::string quiddity_name,
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
  QuiddityManager_Impl::get_property (std::string quiddity_name,
				 std::string property_name)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get property",quiddity_name.c_str());
	return "{\"error\":\"quiddity not found\"}";
      }
    return (get_quiddity (quiddity_name))->get_property(property_name.c_str());
  }

  bool
  QuiddityManager_Impl::has_property (std::string quiddity_name, 
				      std::string property_name)
  {
      if (!exists (quiddity_name))
      {
	g_debug ("quiddity %s not found",quiddity_name.c_str());
	return false;
      }
      Quiddity::ptr quiddity = get_quiddity (quiddity_name);
      
      return quiddity->has_property (property_name);
  }

  bool
  QuiddityManager_Impl::has_method (std::string quiddity_name, 
				    std::string method_name)
  {
      if (!exists (quiddity_name))
      {
	g_debug ("quiddity %s not found",quiddity_name.c_str());
	return false;
      }
      Quiddity::ptr quiddity = get_quiddity (quiddity_name);
      
      return quiddity->has_method (method_name);
  }

  bool 
  QuiddityManager_Impl::invoke (const std::string quiddity_name, 
				const std::string method_name,
				std::string **return_value,
				const std::vector<std::string> args)
  {
    //g_debug ("QuiddityManager_Impl::quiddity_invoke_method %s %s, arg size %d",quiddity_name.c_str(), function_name.c_str(), args.size ());
    
    if (!exists (quiddity_name))
      {
	g_debug ("quiddity %s not found, cannot invoke",quiddity_name.c_str());
	if (return_value != nullptr)
	  *return_value = new std::string ("");
	return false;
      }
    Quiddity::ptr quiddity = get_quiddity (quiddity_name);

    if (!quiddity->has_method (method_name)) 
      {
	g_debug ("method %s not found, cannot invoke",method_name.c_str());
	if (return_value != nullptr)
	  *return_value = new std::string ("");
	return false;
      }

    bool res = quiddity->invoke_method (method_name, return_value, args);
    return res;
  } 

  std::string
  QuiddityManager_Impl::get_methods_description (std::string quiddity_name)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get description of methods",quiddity_name.c_str());
	return "{\"error\":\"quiddity not found\"}";
      }
     
    return (get_quiddity (quiddity_name))->get_methods_description ();
  }


  std::string
  QuiddityManager_Impl::get_method_description (std::string quiddity_name, std::string method_name)
  {
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get description of methods",quiddity_name.c_str());
	return "{\"error\":\"quiddity not found\"}";
      }
     
    return (get_quiddity (quiddity_name))->get_method_description (method_name);
  }

  std::string
  QuiddityManager_Impl::get_methods_description_by_class (std::string class_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create_without_hook (class_name);
    std::string descr = get_methods_description (quid_name);
    remove_without_hook (quid_name);
    return descr;
  }

  std::string
  QuiddityManager_Impl::get_method_description_by_class (std::string class_name, 
							std::string method_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create_without_hook (class_name);
    std::string descr = get_method_description (quid_name, method_name);
    remove_without_hook (quid_name);
    return descr;

  }

  //*** signals
  std::string 
  QuiddityManager_Impl::get_signals_description (std::string quiddity_name)
  {

    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get signals description",quiddity_name.c_str());
	return "";
      }
    return (get_quiddity (quiddity_name))->get_signals_description ();
  }

  std::string 
  QuiddityManager_Impl::get_signal_description (std::string quiddity_name, std::string signal_name)
  {

    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot get signal description",quiddity_name.c_str());
	return "";
      }
    return (get_quiddity (quiddity_name))->get_signal_description (signal_name);
  }

  std::string 
  QuiddityManager_Impl::get_signals_description_by_class (std::string class_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create_without_hook (class_name);
    if (g_strcmp0 (quid_name.c_str (), "") == 0)
      return "{\"error\":\"cannot get signal description because the class cannot be instanciated\"}";
    std::string descr = get_signals_description (quid_name);
    remove_without_hook (quid_name);
    return descr;
  }
  
  std::string
  QuiddityManager_Impl::get_signal_description_by_class (std::string class_name, 
							 std::string signal_name)
  {
    if (!class_exists (class_name))
      return "{\"error\":\"class not found\"}";
    std::string quid_name = create_without_hook (class_name);
    if (g_strcmp0 (quid_name.c_str (), "") == 0)
      return "{\"error\":\"cannot get signal because the class cannot be instanciated\"}";
    std::string descr = get_signal_description (quid_name, signal_name);
    remove_without_hook (quid_name);
    return descr;
  }

 //higher level subscriber

  void
  QuiddityManager_Impl::mute_signal_subscribers (bool muted)
  {
    for (auto &it : signal_subscribers_)
      it.second->mute (muted);
  }

  void
  QuiddityManager_Impl::mute_property_subscribers (bool muted)
  {
    for (auto &it : property_subscribers_)
      it.second->mute (muted);
  }

  bool 
  QuiddityManager_Impl::make_signal_subscriber (std::string subscriber_name,
					       QuidditySignalSubscriber::OnEmittedCallback cb,
					       void *user_data)
  {
    auto it = signal_subscribers_.find (subscriber_name);
    if (signal_subscribers_.end () != it)
      {
	g_warning ("QuiddityManager_Impl, a subscriber named %s already exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    
    QuidditySignalSubscriber::ptr subscriber;
    subscriber.reset (new QuidditySignalSubscriber());
    subscriber->set_manager_impl (shared_from_this());
    subscriber->set_name (subscriber_name.c_str ());
    subscriber->set_user_data (user_data);
    subscriber->set_callback (cb);
    signal_subscribers_[subscriber_name] = subscriber;
    return true; 
  }

  bool
  QuiddityManager_Impl::remove_signal_subscriber (std::string subscriber_name)
  {
    auto it = signal_subscribers_.find (subscriber_name);
    if (signal_subscribers_.end () == it)
      {
	g_warning ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    signal_subscribers_.erase (it);
    return true;
  }
  
  bool 
  QuiddityManager_Impl::subscribe_signal (std::string subscriber_name,
					 std::string quiddity_name,
					 std::string signal_name)
  {
    auto it = signal_subscribers_.find (subscriber_name);
    if (signal_subscribers_.end () == it)
      {
	g_warning ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot subscribe to signal",quiddity_name.c_str());
	return false;
      }
    return signal_subscribers_[subscriber_name]->subscribe (get_quiddity (quiddity_name), signal_name);
  }
  
  bool 
  QuiddityManager_Impl::unsubscribe_signal (std::string subscriber_name,
					     std::string quiddity_name,
					     std::string signal_name)
  {
    auto it = signal_subscribers_.find (subscriber_name);
    if (signal_subscribers_.end () == it)
      {
	g_warning ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	return false;
      }
    if (!exists (quiddity_name))
      {
	g_warning ("quiddity %s not found, cannot subscribe to signal",quiddity_name.c_str());
	return false;
      }
    return signal_subscribers_[subscriber_name]->unsubscribe (get_quiddity (quiddity_name), signal_name);
  }

  std::vector<std::string> 
  QuiddityManager_Impl::list_signal_subscribers ()
  {
    std::vector<std::string> res;
    for (auto &it : signal_subscribers_)
      res.push_back (it.first);
    return res;
  }

    std::vector<std::pair<std::string, std::string> > 
    QuiddityManager_Impl::list_subscribed_signals (std::string subscriber_name)
    {
    auto it = signal_subscribers_.find (subscriber_name);
    if (signal_subscribers_.end () == it)
      {
	g_warning ("QuiddityManager_Impl, a subscriber named %s does not exists\n",
		   subscriber_name.c_str ());
	std::vector<std::pair<std::string, std::string> > empty;
	return empty;
      }
      
      return signal_subscribers_[subscriber_name]->list_subscribed_signals ();
    }

    std::string 
    QuiddityManager_Impl::list_signal_subscribers_json ()
    {
      return "{\"error\":\"to be implemented\"}";//FIXME (list_signal_subscriber_json)
    }
  
    std::string 
    QuiddityManager_Impl::list_subscribed_signals_json (std::string subscriber_name)
    {
      auto subscribed_sigs = list_subscribed_signals (subscriber_name);

      JSONBuilder *doc = new JSONBuilder ();
      doc->reset ();
      doc->begin_object ();
      doc->set_member_name ("subscribed signals");
      doc->begin_array ();
      for(auto &it : subscribed_sigs) 
	{
	  doc->begin_object ();
	  doc->add_string_member ("quiddity", it.first.c_str ());
	  doc->add_string_member ("signal", it.second.c_str ());
	  doc->end_object ();
	}
      doc->end_array ();
      doc->end_object ();
      
      return doc->get_string(true);
    }

  bool 
  QuiddityManager_Impl::set_created_hook (quiddity_created_hook hook, 
					  void *user_data)
  {
    if (creation_hook_ != nullptr)
      return false;
    creation_hook_ = hook;
    creation_hook_user_data_ = user_data;
    return true;
  }
  
  bool 
  QuiddityManager_Impl::set_removed_hook (quiddity_removed_hook hook,
					  void *user_data)
  {
    if (removal_hook_ != nullptr)
      return false;
    removal_hook_ = hook;
    removal_hook_user_data_ = user_data;
    return true;
  }
  
  void 
  QuiddityManager_Impl::reset_create_remove_hooks ()
  {
    creation_hook_ = nullptr;
    removal_hook_ = nullptr;
    creation_hook_user_data_ = nullptr;
    removal_hook_user_data_ = nullptr;
  }


  void
  QuiddityManager_Impl::init_gmainloop ()
  {
    if (! gst_is_initialized ())
      gst_init (nullptr,nullptr);
    
    main_context_ = g_main_context_new ();
    mainloop_ = g_main_loop_new (main_context_, FALSE);
    //mainloop_ = g_main_loop_new (nullptr, FALSE);
    GstRegistry *registry;
    registry = gst_registry_get_default();
    //TODO add option for scanning a path
    gst_registry_scan_path (registry, "/usr/local/lib/gstreamer-0.10/");
    thread_ = std::thread (&QuiddityManager_Impl::main_loop_thread, this);
  }
  
  void
  QuiddityManager_Impl::main_loop_thread ()
  {
    g_main_loop_run (mainloop_);
  }
  
  GMainContext *
  QuiddityManager_Impl::get_g_main_context ()
  {
    return main_context_;
  } 
  
   bool 
   QuiddityManager_Impl::load_plugin (const char *filename)
   {

     PluginLoader::ptr plugin (new PluginLoader ());
     
     if (!plugin->load (filename))
       return false;
     
     std::string class_name = plugin->get_class_name ();

     //close the old one if exists
     auto it = plugins_.find (class_name);
     if (plugins_.end () != it)
       {
	 g_debug ("closing old plugin for reloading (class: %s)",
		  class_name.c_str ());
	 close_plugin (class_name);
       }
     
     abstract_factory_.register_class_with_custom_factory (class_name,
     							   plugin->get_json_root_node (),
     							   plugin->create_,
     							   plugin->destroy_);
     plugins_[class_name] = plugin;
     return true;
   }

  void
  QuiddityManager_Impl::close_plugin (const std::string class_name)
  {
    abstract_factory_.unregister_class (class_name);
    plugins_.erase (class_name);
  }

  bool 
  QuiddityManager_Impl::scan_directory_for_plugins (const char *directory_path)
  {
    GFile *dir = g_file_new_for_commandline_arg (directory_path);
    gboolean res;
    GError *error;
    GFileEnumerator *enumerator;
    GFileInfo *info;
    GFile *descend;
    char *absolute_path;
    error = nullptr;
    enumerator =
      g_file_enumerate_children (dir, "*",
				 G_FILE_QUERY_INFO_NOFOLLOW_SYMLINKS, 
				 nullptr,
				 &error);
    if (! enumerator)
      return false;;
    error = nullptr;
    info = g_file_enumerator_next_file (enumerator, nullptr, &error);
    while ((info) && (!error))
      {
	descend = g_file_get_child (dir, g_file_info_get_name (info));
	absolute_path = g_file_get_path (descend);//g_file_get_relative_path (dir, descend);
	//trying to load the module 
	if (g_str_has_suffix (absolute_path, ".so") || g_str_has_suffix (absolute_path, ".dylib"))
	  {
	    g_debug ("loading module %s", absolute_path);
	    load_plugin (absolute_path);
	  }
	g_free (absolute_path);
	g_object_unref (descend);
	info = g_file_enumerator_next_file (enumerator, nullptr, &error);
      }
    error = nullptr;
    res = g_file_enumerator_close (enumerator, nullptr, &error);
    if (res != TRUE)
      g_debug ("scanning dir: file enumerator not properly closed");
    if (error != nullptr)
      g_debug ("scanning dir: error not nullptr");
    g_object_unref (dir);

    classes_doc_.reset (new JSONBuilder ());
    make_classes_doc ();
        
    return true;
  }

  std::string
  QuiddityManager_Impl::get_info (const std::string &nick_name, const std::string &path)
  {
    auto it = quiddities_nick_names_.find (nick_name);
    if (quiddities_nick_names_.end () == it)
      return "{ \"error\":\"quiddity not found\"}";
    return quiddities_[quiddities_nick_names_[nick_name]]->get_info (path);
  }

} // end of namespace
