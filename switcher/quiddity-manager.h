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

/**
 * The QuiddityManager class
 */

#ifndef __SWITCHER_QUIDDITY_MANAGER_H__
#define __SWITCHER_QUIDDITY_MANAGER_H__

#include <vector>
#include <map>
#include <string>
#include <mutex>
#include <condition_variable>
#include <thread>
#include "quiddity-manager-impl.h"
#include "quiddity-command.h"
#include "quiddity-manager-wrapper.h"
#include <stdarg.h>

namespace switcher 
{ 
  class QuiddityManager : public std::enable_shared_from_this<QuiddityManager>
    //FIXME add const for method args
    { 
    public: 
      typedef std::shared_ptr<QuiddityManager> ptr; 
      typedef std::vector<QuiddityCommand::ptr> CommandHistory; 
      typedef void (*PropCallback)(std::string subscriber_name,
				   std::string quiddity_name,
				   std::string property_name,
				   std::string value,
				   void *user_data);
      typedef void (*SignalCallback)(std::string subscriber_name,
				     std::string quiddity_name,
				     std::string property_name,
				     std::vector<std::string> params,
				     void *user_data);
      typedef std::map<std::string, 
	std::pair<QuiddityManager::PropCallback, 
	void *> > PropCallbackMap;
      typedef std::map<std::string, 
	std::pair<QuiddityManager::SignalCallback, 
	void *> > SignalCallbackMap;

      static QuiddityManager::ptr make_manager (std::string name);
      ~QuiddityManager(); 
      QuiddityManager *operator=(const QuiddityManager &) = delete;
      QuiddityManager (const QuiddityManager &) = delete;

      std::string get_name ();
      void reboot ();

      //*************** command history ***********************************************************
      bool save_command_history (const char *file_path);
      CommandHistory get_command_history_from_file (const char *file_path);
      std::vector<std::string> get_property_subscribers_names (QuiddityManager::CommandHistory histo);
      std::vector<std::string> get_signal_subscribers_names (QuiddityManager::CommandHistory histo);
      void play_command_history (QuiddityManager::CommandHistory histo,
				 QuiddityManager::PropCallbackMap *prop_cb_data,
				 QuiddityManager::SignalCallbackMap *sig_cb_data,
				 bool mute_property_and_signal_subscribers);
      void reset_command_history (bool remove_created_quiddities);//FIXME maybe implement undo and remove this  arg

      //************** plugins *******************************************************************
      bool scan_directory_for_plugins (std::string directory);

      //***************** inspect ****************************************************************
      std::vector<std::string> get_classes (); //know which quiddities can be created
      std::vector<std::string> get_quiddities (); //know instances
      // doc (json formatted) 
      std::string get_classes_doc ();
      std::string get_class_doc (std::string class_name);
      std::string get_quiddity_description (std::string quiddity_name);
      std::string get_quiddities_description ();
      // create/remove/rename
      std::string create (std::string class_name); //returns the name
      std::string create (std::string class_name, 
			  std::string nick_name); // &?= chars are not allowed in nicknames
      bool remove (std::string quiddity_name);
      bool rename (std::string nick_name, std::string new_nick_name);

      //****************** informations **********************************************************
      std::string get_info (const std::string &nick_name, const std::string &path);
      
      //****************** properties ************************************************************
      //doc (json formatted)
      std::string get_properties_description (std::string quiddity_name);
      std::string get_property_description (std::string quiddity_name, 
					    std::string property_name);
      //following "by_class" methods provide properties available after creation only, 
      //avoiding possible properties created dynamically
      std::string get_properties_description_by_class (std::string class_name); 
      std::string get_property_description_by_class (std::string class_name, 
						     std::string property_name); 
      //set & get
      bool set_property (std::string quiddity_name,
			 std::string property_name,
			 std::string property_value);
     
      std::string get_property (std::string quiddity_name, 
				std::string property_name);

      bool has_property (const std::string quiddity_name,
			 const std::string property_name);

      //property subscribtion
      bool make_property_subscriber (std::string subscriber_name,
				     QuiddityManager::PropCallback callback,
				     void *user_data);
      bool remove_property_subscriber (std::string subscriber_name);
      bool subscribe_property (std::string subscriber_name,
			       std::string quiddity_name,
			       std::string property_name);
      bool unsubscribe_property (std::string subscriber_name,
				 std::string quiddity_name,
				 std::string property_name);
      std::vector<std::string> 
	list_property_subscribers ();
      std::vector<std::pair<std::string, std::string> > 
	list_subscribed_properties (std::string subscriber_name);
      //json //FIXME implement
      std::string 
	list_property_subscribers_json ();
      std::string 
	list_subscribed_properties_json (std::string subscriber_name);
     
     
      //LOWER LEVEL subscription
      //This is how to subscribe and get property values when changed:
      /* static gchar *hello = "hello"; */
      /* void prop_cb (GObject *gobject, GParamSpec *pspec, gpointer user_data) */
      /*   g_print ("---------------- property callback: %s -- %s\n",  */
      /* 		(gchar *)user_data,  */
      /* 		switcher::Property::parse_callback_args (gobject, pspec).c_str ()); */
      /* //testing property */
      /* manager->create ("videotestsrc","vid"); */
      /* manager->subscribe_property ("vid", "pattern", prop_cb, hello); */

      bool subscribe_property_glib (std::string quiddity_name,
				    std::string name,
				    Property::Callback cb, 
				    void *user_data);
      bool unsubscribe_property_glib (std::string quiddity_name,
				      std::string name,
				      Property::Callback cb, 
				      void *user_data); //the same called with subscribe
     
     
      //*********************** methods 
      //doc (json formatted)
      std::string get_methods_description (std::string quiddity_name);
      std::string get_method_description (std::string quiddity_name, 
					  std::string method_name);
      //following "by_class" methods provide properties available after creation only
      std::string get_methods_description_by_class (std::string class_name); 
      std::string get_method_description_by_class (std::string class_name, 
						   std::string method_name);
      //invoke
      bool invoke (const std::string quiddity_name, 
		   const std::string method_name,
		   std::string **return_value,
		   const std::vector<std::string> args);  
      bool invoke_va (const gchar *quiddity_name,
		      const gchar *method_name,
		      std::string **return_value,
		      ...);
      
      bool has_method (const std::string quiddity_name,
		       const std::string method_name);
    
      //************************ signals 
      //doc (json formatted)
      std::string get_signals_description (std::string quiddity_name);
      std::string get_signal_description (std::string quiddity_name, 
					  std::string signal_name);
      //following "by_class" methods provide properties available after creation only, 
      //avoiding possible properties created dynamically
      std::string get_signals_description_by_class (std::string class_name); 
      std::string get_signal_description_by_class (std::string class_name, 
						   std::string signal_name); 
    
      bool make_signal_subscriber (std::string subscriber_name,
				   /* void (*callback)(std::string subscriber_name, */
				   /* 		    std::string quiddity_name, */
				   /* 		    std::string signal_name, */
				   /* 		    std::vector<std::string> params, */
				   /* 		    void *user_data) */
				   QuiddityManager::SignalCallback callback,
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
      //json //FIXME implement or remove
      std::string 
	list_signal_subscribers_json ();
      std::string 
	list_subscribed_signals_json (std::string subscriber_name);
    
    private: 
      QuiddityManager_Impl::ptr manager_impl_; //may be shared with others for automatic quiddity creation 
      std::string name_;
      //running commands in sequence 
      QuiddityCommand::ptr command_;
      std::mutex seq_mutex_;
      GAsyncQueue *command_queue_;
      std::thread invocation_thread_;
      //invokation in gmainloop
      std::condition_variable execution_done_cond_; //sync current thread and gmainloop  
      std::mutex execution_done_mutex_; //sync current thread and gmainloop  
      //history
      CommandHistory command_history_;
      gint64 history_begin_time_; //monotonic time, in microseconds


      QuiddityManager() = delete;
      QuiddityManager(std::string name); 
      //auto invoke and init
      void auto_init (std::string quiddity_name);
      void command_lock ();
      void command_unlock ();
      std::string seq_invoke (QuiddityCommand::command command, ...);
      void init_command_sync(); 
      void clear_command_sync(); 
      void invocation_thread ();
      static gboolean execute_command (gpointer user_data);//gmainloop source callback
      void invoke_in_thread ();
    }; 

} // end of namespace 

#endif  




