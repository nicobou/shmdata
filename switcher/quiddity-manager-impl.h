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
 * The QuiddityManagerImpl class wraps abstract factory for creating instace by name (birth) 
 * It wraps StringMap for managing instances (subsistence). 
 * Object destruction is managed through the use of std::shared_ptr
 */

#ifndef __SWITCHER_QUIDDITY_MANAGER_IMPL_H__
#define __SWITCHER_QUIDDITY_MANAGER_IMPL_H__


#include <memory>
#include "abstract-factory.h" 
#include "string-map.h"
#include "quiddity.h" 
#include "json-builder.h"
#include "quiddity-property-subscriber.h"
#include "quiddity-signal-subscriber.h"

namespace switcher
{
  class Quiddity;
  class QuiddityPropertySubscriber;
  class QuidditySignalSubscriber;

  class QuiddityManager_Impl : public std::enable_shared_from_this<QuiddityManager_Impl>
    {
    public:
      typedef std::shared_ptr< QuiddityManager_Impl > ptr;
      typedef void (*quiddity_created_hook) (std::string nick_name, void *user_data);
      typedef void (*quiddity_removed_hook) (std::string nick_name, void *user_data);

      static QuiddityManager_Impl::ptr make_manager ();//will get name "default"
      static QuiddityManager_Impl::ptr make_manager (std::string name);
      //void reboot ();
      ~QuiddityManager_Impl();
    
      //**** info about manager
      std::string get_name ();
      std::vector<std::string> get_classes ();//vector of class names
      std::vector<std::string> get_instances ();//vector of instance names
      //doc (json formatted)
      std::string get_classes_doc ();
      std::string get_class_doc (std::string class_name);
      std::string get_quiddity_description (std::string quiddity_name);
      std::string get_quiddities_description ();
      bool class_exists (std::string class_name);
      bool exists (std::string quiddity_name);
      
      //**** creation/remove/get
      std::string create (std::string quiddity_class);
      std::string create (std::string quiddity_class, 
			  std::string nick_name);
      bool remove (std::string quiddity_name);
      std::shared_ptr<Quiddity> get_quiddity (std::string quiddity_name);
      // only one hook is allowed now, 
      // it is used by the quiddity manager-spy-create-remove
      // for converting creating removal into signals
      bool set_created_hook (quiddity_created_hook hook, void *user_data);
      bool set_removed_hook (quiddity_removed_hook hook, void *user_data);
      void reset_create_remove_hooks ();
      
      //**** properties
      //doc (json formatted)
      std::string get_properties_description (std::string quiddity_name);
      std::string get_property_description (std::string quiddity_name, 
					    std::string property_name);
      //following "by_class" methods provide properties available after creation only
      std::string get_properties_description_by_class (std::string class_name); 
      std::string get_property_description_by_class (std::string class_name, 
						     std::string property_name); 
      //set & get
      bool set_property (std::string quiddity_name,
			 std::string property_name,
			 std::string property_value);
      std::string get_property (std::string quiddity_name, 
				std::string property_name);

      //high level property subscriber
      bool make_property_subscriber (std::string subscriber_name,
				     void (*callback)(std::string subscriber_name,
						      std::string quiddity_name,
						      std::string property_name,
						      std::string value,
						      void *user_data),
				     void *user_data);
      bool remove_property_subscriber (std::string subscriber_name);
      bool subscribe_property (std::string subscriber_name,
			       std::string quiddity_name,
			       std::string property_name);
      bool unsubscribe_property (std::string subscriber_name,
				 std::string quiddity_name,
				 std::string property_name);
      //signal utils
      std::vector<std::string> 
	list_property_subscribers ();
      std::vector<std::pair<std::string, std::string> > 
	list_subscribed_properties (std::string subscriber_name);
      std::string 
	list_property_subscribers_json ();
      std::string 
	list_subscribed_properties_json (std::string subscriber_name);

      //low level subscribe
      bool subscribe_property_glib (std::string quiddity_name,
				    std::string property_name,
				    Property::Callback cb, 
				    void *user_data);
      bool unsubscribe_property_glib (std::string quiddity_name,
				      std::string property_name,
				      Property::Callback cb, 
				      void *user_data);//the same as called with subscribe

      //**** methods 
      //doc (json formatted)
      std::string get_methods_description (std::string quiddity_name); 
      std::string get_method_description (std::string quiddity_name, 
					  std::string method_name);
      //following "by_class" methods provide properties available after creation only
      std::string get_methods_description_by_class (std::string class_name); 
      std::string get_method_description_by_class (std::string class_name, 
						   std::string method_name);
      //invoke
      bool invoke (std::string quiddity_name, 
		   std::string method_name,
		   std::vector<std::string> args);  
      
      bool has_method (std::string quiddity_name, 
		       std::string method_name);

      //**** signals
      //doc (json formatted)
      std::string get_signals_description (std::string quiddity_name);
      std::string get_signal_description (std::string quiddity_name, 
					  std::string signal_name);
      //following "by_class" methods provide properties available after creation only, 
      //avoiding possible properties created dynamically
      std::string get_signals_description_by_class (std::string class_name); 
      std::string get_signal_description_by_class (std::string class_name, 
						   std::string signal_name); 
      //high level signal subscriber
      bool make_signal_subscriber (std::string subscriber_name,
				   void (*callback)(std::string subscriber_name,
						    std::string quiddity_name,
						    std::string property_name,
						    std::vector<std::string> params,
						    void *user_data),
				   void *user_data);
      bool remove_signal_subscriber (std::string subscriber_name);
      bool subscribe_signal (std::string subscriber_name,
			     std::string quiddity_name,
			     std::string signal_name);
      bool unsubscribe_signal (std::string subscriber_name,
			       std::string quiddity_name,
			       std::string signal_name);

      std::vector<std::string> 
	list_signal_subscribers ();
      std::vector<std::pair<std::string, std::string> > 
	list_subscribed_signals (std::string subscriber_name);
      std::string 
	list_signal_subscribers_json ();
      std::string 
	list_subscribed_signals_json (std::string subscriber_name);

      //mainloop
      GMainContext * get_g_main_context();


      //for use of "get description by class" 
      //and from quiddity that creates other quiddity in the same manager
      std::string create_without_hook (std::string quiddity_class);
      bool remove_without_hook (std::string quiddity_name);

    private:
      void init (std::string name);
      QuiddityManager_Impl();//will get name "default"
      QuiddityManager_Impl(std::string);
      void make_classes_doc ();
      std::string name_;
      void register_classes ();
      AbstractFactory< Quiddity, std::string, JSONBuilder::Node> abstract_factory_;
      StringMap< std::shared_ptr<Quiddity> > quiddities_;
      StringMap< std::string > quiddities_nick_names_;
      StringMap< std::shared_ptr <QuiddityPropertySubscriber> > property_subscribers_;
      StringMap< std::shared_ptr <QuidditySignalSubscriber> > signal_subscribers_;
      bool init_quiddity (std::shared_ptr<Quiddity> quiddity);
      void remove_shmdata_sockets ();
      JSONBuilder::ptr classes_doc_;
      quiddity_created_hook creation_hook_;
      quiddity_removed_hook removal_hook_;
      void *creation_hook_user_data_;
      void *removal_hook_user_data_;

      //gmainloop 
      GThread *thread_; //this runs the main loop 
      GMainContext *main_context_;
      GMainLoop *mainloop_; 
      void init_gmainloop (); 
      static gpointer main_loop_thread (gpointer user_data); 

    };

} // end of namespace



#endif // ifndef
