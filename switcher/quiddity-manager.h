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
 * The QuiddityManager class
 */

#ifndef __SWITCHER_QUIDDITY_MANAGER_H__
#define __SWITCHER_QUIDDITY_MANAGER_H__

#include <vector>
#include <map>
#include <string>
#include "quiddity-life-manager.h"
#include "quiddity-command.h"
#include "quiddity-manager-wrapper.h"
#include <stdarg.h>

 namespace switcher 
 { 
   class QuiddityManager : public std::enable_shared_from_this<QuiddityManager>
   { 
   public: 
     typedef std::shared_ptr<QuiddityManager> ptr; 

     static QuiddityManager::ptr make_manager ();//will get name "default"
     static QuiddityManager::ptr make_manager (std::string name);
     ~QuiddityManager(); 
     std::string get_name ();
     

     // life manager
     std::vector<std::string> get_classes (); //know which quiddities can be created
     std::vector<std::string> get_quiddities (); //know instances
     // doc (json formatted) 
     std::string get_classes_doc ();
     std::string get_class_doc (std::string class_name);
     std::string get_quiddity_description (std::string quiddity_name);
     std::string get_quiddities_description ();
     // create & remove
     std::string create (std::string class_name); //returns the name
     std::string create (std::string class_name, 
			 std::string nick_name); // &?= chars are not allowed in nicknames
     bool remove (std::string quiddity_name);


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

     //property subscribtion
     bool make_subscriber (std::string subscriber_name,
			   void (*callback)(std::string subscriber_name,
					    std::string quiddity_name,
					    std::string property_name,
					    std::string value,
					    void *user_data),
			   void *user_data);
     bool remove_subscriber (std::string subscriber_name);
     bool subscribe_property (std::string subscriber_name,
			      std::string quiddity_name,
			      std::string property_name);
     bool unsubscribe_property (std::string subscriber_name,
				std::string quiddity_name,
				std::string property_name);
     std::vector<std::string> 
       list_subscribers ();
     std::vector<std::pair<std::string, std::string> > 
       list_subscribed_properties (std::string subscriber_name);
     //json //FIXME implement or remove
     std::string 
       list_subscribers_json ();
     std::string 
       list_subscribed_properties_json (std::string subscriber_name);
     
     
     //LOWER LEVEL subscription
     //This is how to subscribe and get property values when changed:
     /* static gchar *coucou = "coucou"; */
     /* void prop_cb (GObject *gobject, GParamSpec *pspec, gpointer user_data) */
     /*   g_print ("---------------- property callback: %s -- %s\n",  */
     /* 		(gchar *)user_data,  */
     /* 		switcher::Property::parse_callback_args (gobject, pspec).c_str ()); */
     /* //testing property */
     /* manager->create ("videotestsrc","vid"); */
     /* manager->subscribe_property ("vid", "pattern", prop_cb, coucou); */

     bool subscribe_property_glib (std::string quiddity_name,
				   std::string name,
				   Property::Callback cb, 
				   void *user_data);
     bool unsubscribe_property_glib (std::string quiddity_name,
				     std::string name,
				     Property::Callback cb, 
				     void *user_data); //the same called with subscribe
     
     
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
    bool invoke_va (const gchar *quiddity_name,
		    ...);
     
     // will invoke the given method after quiddity creation, 
     //if method exists (only one method)
     bool auto_invoke  (std::string method_name,
			std::vector<std::string> args);  

   private: 
     QuiddityManager();//will get name "default"
     QuiddityManager(std::string name); 
     QuiddityLifeManager::ptr life_manager_; //may be shared with others for automatic quiddity creation 
     std::string name_;

     //auto invoke and init
     void auto_init (std::string quiddity_name);
     std::string auto_invoke_method_name_;
     std::vector<std::string> auto_invoke_args_;

     //gmainloop
     static bool gmainloop_initialized_;
     GThread *thread_; //runing the main loop
     GMainLoop *mainloop_;
     void init_gmainloop ();
     static gpointer main_loop_thread (gpointer user_data);

     //running commands in the gmain_loop context
     static gpointer gmainloop_run (gpointer user_data);//thread for the loop
     void invoke_in_gmainloop();
     QuiddityCommand command_;
     GCond *exec_cond_; //sync current thread and gmainloop
     GMutex *exec_mutex_; //sync current thread and gmainloop
     void init_command_sync(); 
     void clear_command_sync(); 

     //ensure sequential invokation
     GMutex *seq_mutex_; 
     std::string seq_invoke (QuiddityCommand::command command, ...);
   }; 

 } // end of namespace 

#endif  




