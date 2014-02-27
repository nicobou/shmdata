/*
 * This file is part of switcher-gsoap.
 *
 * switcher-gsoap is free software: you can redistribute it and/or modify
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

#include "soap-ctrl-server.h"
#include "webservices/control.nsmap"
//hacking gsoap bug for ubuntu 13.10
#ifdef WITH_IPV6
#define SOAPBINDTO "::"
#else
# define SOAPBINDTO NULL
#endif


namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(SoapCtrlServer,
				       "Switcher Web Controler (SOAP)",
				       "control server", 
				       "getting switcher controled through SOAP webservices",
				       "GPL",
				       "SOAPcontrolServer",
				       "Nicolas Bouillot");
    
  SoapCtrlServer::SoapCtrlServer () :
    soap_ (),
    port_ (8080),
    quit_server_thread_ (false),
    service_ (NULL), 
    socket_ (),
    thread_ ()
  {}

  bool
  SoapCtrlServer::init ()
  {
    soap_init(&soap_);
    //release port
    soap_.connect_flags = SO_LINGER; 
    soap_.accept_flags = SO_LINGER;
    soap_.accept_timeout =  100 * -1000; //100ms
    soap_.fget = SoapCtrlServer::http_get;
    
    install_method ("Set Port",
		    "set_port", 
		    "set the port used by the soap server", 
		    "success or fail",
		    Method::make_arg_description ("Port",
						  "port",
						  "the port to bind",
						  NULL),
		    (Method::method_ptr) &set_port_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_INT, NULL),
     		    this);
   
    return true;
  }

  SoapCtrlServer::~SoapCtrlServer ()
  {
    stop ();
  }

  std::shared_ptr<QuiddityManager>
  SoapCtrlServer::get_quiddity_manager ()
  {
    return manager_.lock ();
  }


 int 
 SoapCtrlServer::http_get (struct soap *soap)
 {
   std::string rtpsession_name;
   std::string destination_name;
   
   if (g_str_has_prefix (soap->path, "/sdp"))
     {
       gchar **query = g_strsplit_set (soap->path, "?",-1);
       
       if (query[1] == NULL)
	 return 404;

       gchar **query_vars = g_strsplit_set (query[1], "&",-1);
       int i=0;
       while (query_vars [i] != NULL)
	 {
	   gchar **var = g_strsplit_set (query_vars [i], "=",-1);
	   if (g_strcmp0 (var[0],"rtpsession") == 0 && var[1] != NULL)
	     {
	       rtpsession_name.clear ();
	       rtpsession_name.append (var[1]);
	     }
	   else if (g_strcmp0 (var[0],"destination") == 0 && var[1] != NULL)
	     {
	       destination_name.clear ();
	       destination_name.append (var[1]);
	     }	   
	   g_strfreev(var);
	   i++;
	 }
       g_strfreev(query_vars);
       g_strfreev(query);
       
       SoapCtrlServer *ctrl_server = (SoapCtrlServer *) soap->user;
       QuiddityManager::ptr manager;
       if (ctrl_server != NULL)
	 manager = ctrl_server->get_quiddity_manager ();

       if (!(bool) manager)
	 return 404;

       std::vector<std::string> arg;
       arg.push_back (destination_name);
       if (!manager->invoke (rtpsession_name, "write_sdp_file", NULL, arg))
	   return 404;
       
       //sending file to client
       std::string sdp_file = get_socket_dir ();
       sdp_file.append ("/");
       sdp_file.append (get_socket_name_prefix ());
       sdp_file.append(manager->get_name()+"_"+rtpsession_name+"_"+destination_name+".sdp");
       gchar *sdp_contents;
       gsize file_length;
       g_file_get_contents (sdp_file.c_str (), 
			    &sdp_contents, 
			    &file_length, 
			    NULL); //not getting errors

       soap_response(soap, SOAP_FILE); 
       soap->http_content = "application/x-sdp";
       soap_send_raw(soap, sdp_contents, file_length);
       g_free (sdp_contents);
       return soap_end_send(soap);
     }
   return 404;
 }

  gboolean
  SoapCtrlServer::set_port_wrapped (gint port, gpointer user_data)
  {
    SoapCtrlServer *context = static_cast<SoapCtrlServer*>(user_data);
    if (context->set_port (port))
      return TRUE;
    else
      return FALSE;
  }

  bool 
  SoapCtrlServer::set_port (int port)
  {
    port_ = port;
    return start ();
  }

  bool 
  SoapCtrlServer::start ()
  {
    soap_.user = (void *)this;
    quit_server_thread_ = false;
    service_ = new controlService (soap_);
    socket_ = service_->bind (SOAPBINDTO, port_, 100 /* BACKLOG */);
    if (!soap_valid_socket (socket_))
      {
	service_->soap_print_fault (stderr);
	delete service_;
	service_ = NULL;
	return false;
      }

    thread_ = std::thread (&SoapCtrlServer::server_thread, this);
    return true;
  }


  // gboolean
  // SoapCtrlServer::stop_wrapped (gpointer user_data)
  // {
  //   SoapCtrlServer *context = static_cast<SoapCtrlServer*>(user_data);
  //   context->stop ();
  //   return TRUE;
  // }

  bool
  SoapCtrlServer::stop ()
  {
    quit_server_thread_ = true;
    if (thread_.joinable ())
      thread_.join ();      
    soap_closesocket (socket_);
    soap_destroy(&soap_);
    soap_end(&soap_);
    soap_done(&soap_);
    if (NULL != service_)
      delete service_;
    return true;
  }
  
  void
  SoapCtrlServer::server_thread ()
  {
    // /* run iterative server on port until fatal error */
    // if (context->service_->run(context->port_))
    //   { context->service_->soap_stream_fault(std::cerr);
    //   }
    // return NULL;

    //for (int i = 1; ; i++)
    while(!quit_server_thread_)
      { 
	SOAP_SOCKET s = service_->accept();
	if (!soap_valid_socket(s))
	  { 
	    if (service_->errnum)
	      service_->soap_print_fault (stderr);
	    else
	      {
		//g_debug ("SOAP server timed out");
	      }
	  }
	else
	  {
	    // g_debug ("client request %d accepted on socket %d, client IP is %d.%d.%d.%d", 
	    // 	    i, s, 
	    // 	    (int)(service_->ip>>24)&0xFF, 
	    // 	    (int)(service_->ip>>16)&0xFF, 
	    // 	    (int)(service_->ip>>8)&0xFF, 
	    // 	    (int)service_->ip&0xFF);
	    controlService *tcontrol = service_->copy();
	    if (service_->errnum)
	      service_->soap_print_fault(stderr);
	    tcontrol->serve();
	    delete tcontrol;
	  }
      }
  }

}//end of SoapCtrlServer class


/**********************************************
 * below is the implementation of the service *
 **********************************************/

int
controlService::get_factory_capabilities(std::vector<std::string> *result){//FIXME rename that to get_classes
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();
    
  if (ctrl_server == NULL || !(bool)manager)
    {
      char *s = (char*)soap_malloc(this, 1024);
      g_debug ("controlService::get_factory_capabilities: cannot get manager from SoapCtrlServer (NULL)");
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">controlService::get_factory_capabilities: cannot get manager (NULL)</error>");
      return soap_senderfault("error in get_factory_capabilities", s);
    }
  
  *result = manager->get_classes ();
  
  return SOAP_OK;
}

int
controlService::get_classes_doc(std::string *result){
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();
    
  if (ctrl_server == NULL || !(bool)manager)
    {
      char *s = (char*)soap_malloc(this, 1024);
      g_debug ("controlService::get_classes_doc: cannot get manager from SoapCtrlServer (NULL)");
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">controlService::get_factory_capabilities: cannot get manager (NULL)</error>");
      return soap_senderfault("error in get_classes_doc", s);
    }
  
  *result = manager->get_classes_doc ();
  
  return SOAP_OK;
}

int
controlService::get_quiddity_description(std::string quiddity_name, std::string *result){
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();
    
  if (ctrl_server == NULL || !(bool)manager)
    {
      char *s = (char*)soap_malloc(this, 1024);
      g_debug ("controlService::get_class_doc: cannot get manager from SoapCtrlServer (NULL)");
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">controlService::get_factory_capabilities: cannot get manager (NULL)</error>");
      return soap_senderfault("error in get_class_doc", s);
    }
  
  *result = manager->get_quiddity_description (quiddity_name);
  
  return SOAP_OK;
}

int
controlService::get_quiddities_description (std::string *result){
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();
    
  if (ctrl_server == NULL || !(bool)manager)
    {
      char *s = (char*)soap_malloc(this, 1024);
      g_debug ("controlService::get_quiddities_description: cannot get manager from SoapCtrlServer (NULL)");
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">controlService::get_quiddities_description: cannot get manager (NULL)</error>");
      return soap_senderfault("error in get_classes_doc", s);
    }
  
  *result = manager->get_quiddities_description ();
  
  return SOAP_OK;
}

int
controlService::get_class_doc(std::string class_name, std::string *result){
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();
    
  if (ctrl_server == NULL || !(bool)manager)
    {
      char *s = (char*)soap_malloc(this, 1024);
      g_debug ("controlService::get_class_doc: cannot get manager from SoapCtrlServer (NULL)");
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">controlService::get_factory_capabilities: cannot get manager (NULL)</error>");
      return soap_senderfault("error in get_class_doc", s);
    }
  
  *result = manager->get_class_doc (class_name);
  
  return SOAP_OK;
}

int
controlService::get_quiddity_names(std::vector<std::string> *result)
{
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_quiddities ();
  
  return SOAP_OK;
}

int
controlService::get_properties_description (std::string quiddity_name,
					    std::string *result)
{
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_properties_description (quiddity_name);

  return SOAP_OK;
}

int
controlService::get_properties_description_by_class (std::string class_name,
						     std::string *result)
{
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_properties_description_by_class (class_name);

  return SOAP_OK;
}

int
controlService::get_property_description (std::string quiddity_name,
					  std::string property_name,
					  std::string *result)
{
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_property_description (quiddity_name, property_name);

  return SOAP_OK;
}

int
controlService::get_property_description_by_class (std::string class_name,
						   std::string property_name,
						   std::string *result)
{
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_property_description_by_class (class_name, property_name);

  return SOAP_OK;
}


int
controlService::set_property (std::string quiddity_name, 
			      std::string property_name,
			      std::string property_value)
{
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  manager->set_property (quiddity_name, property_name, property_value);

  return send_set_property_empty_response(SOAP_OK);
}


int
controlService::get_property (std::string quiddity_name, 
			      std::string property_name,
			      std::string *result)
{
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

 *result = manager->get_property (quiddity_name, property_name);
  
  return SOAP_OK;
}


int
controlService::create_quiddity (std::string quiddity_class, 
			       std::string *result)
{
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

   std::string name = manager->create (quiddity_class);
   if (name != "")
     {
       *result = name; 
     }
   else 
     { 
       char *s = (char*)soap_malloc(this, 1024);
       sprintf(s, "%s cannot be created, see switcher logs", quiddity_class.c_str());
       return soap_senderfault("Quiddity creation error", s);
     }
  return SOAP_OK;
}

int
controlService::create_named_quiddity (std::string quiddity_class, 
				       std::string nick_name,
				       std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();


   std::string name = manager->create (quiddity_class, nick_name);
   if (name != "")
     {
       *result = name; 
     }
   else 
     { 
       char *s = (char*)soap_malloc(this, 1024);
       sprintf(s, "%s cannot be created, see switcher logs", quiddity_class.c_str());
       return soap_senderfault("Quiddity creation error", s);
     }
  return SOAP_OK;
}

int
controlService::rename_quiddity (std::string nick_name, 
				 std::string new_nick_name,
				 std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  if (manager->rename (nick_name, new_nick_name))
    *result = "true"; 
   else 
     *result = "false";
  return SOAP_OK;
}

int
controlService::delete_quiddity (std::string quiddity_name)
{
  using namespace switcher;
  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();
 
  if (manager->remove (quiddity_name))
    return send_set_property_empty_response(SOAP_OK);
  else
    {
      char *s = (char*)soap_malloc(this, 1024);
      sprintf(s, "%s is not found, not deleting", quiddity_name.c_str());
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">%s is not found, not deleting</error>", quiddity_name.c_str());
      return send_set_property_empty_response(soap_senderfault("Quiddity creation error", s));
    }
}


int
controlService::invoke_method (std::string quiddity_name,
			       std::string method_name,
			       std::vector<std::string> args,
			       std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();
  std::string *invocation_result;
  if (manager->invoke (quiddity_name, method_name, &invocation_result, args))
    {
      *result = *invocation_result;
      return SOAP_OK;
    }
  else
    {
      char *s = (char*)soap_malloc(this, 1024);
      sprintf(s, "invoking %s/%s returned false", quiddity_name.c_str(), method_name.c_str());
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">invoking %s/%s returned false</error>", quiddity_name.c_str(), method_name.c_str());
      return soap_senderfault("Method invocation error", s);
    }
}


int
controlService::get_methods_description (std::string quiddity_name,
					 std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_methods_description (quiddity_name);
  return SOAP_OK;
}

int
controlService::get_method_description (std::string quiddity_name,
					std::string method_name,
					std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_method_description (quiddity_name, method_name);
  return SOAP_OK;
}

int
controlService::get_methods_description_by_class (std::string class_name,
						  std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_methods_description_by_class (class_name);
  return SOAP_OK;
}

int
controlService::get_method_description_by_class (std::string class_name,
						 std::string method_name,
						 std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_method_description_by_class (class_name, method_name);
  return SOAP_OK;
}

int
controlService::get_signals_description (std::string quiddity_name,
					 std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_signals_description (quiddity_name);
  return SOAP_OK;
}

int
controlService::get_signal_description (std::string quiddity_name,
					std::string signal_name,
					std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_signal_description (quiddity_name, signal_name);
  return SOAP_OK;
}

int
controlService::get_signals_description_by_class (std::string class_name,
						  std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_signals_description_by_class (class_name);
  return SOAP_OK;
}

int
controlService::get_signal_description_by_class (std::string class_name,
						 std::string signal_name,
						 std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  *result = manager->get_signal_description_by_class (class_name, signal_name);
  return SOAP_OK;
}


int
controlService::save (std::string file_name,
		      std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  if (manager->save_command_history (file_name.c_str ()))
    *result = "true";
  else
    *result = "false";
  return SOAP_OK;
}

int
controlService::load (std::string file_name,
		      std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  manager->reset_command_history(true);

  switcher::QuiddityManager::CommandHistory histo = 
    manager->get_command_history_from_file (file_name.c_str ());
   if (histo.empty ())
    {
      *result = "false";
      return SOAP_OK;
    }
   manager->play_command_history (histo, NULL, NULL, true); 
  *result = "true";
  return SOAP_OK;
}

int
controlService::run (std::string file_name,
		     std::string *result)
{
  using namespace switcher;

  SoapCtrlServer *ctrl_server = (SoapCtrlServer *) this->user;
  QuiddityManager::ptr manager;
  if (ctrl_server != NULL)
    manager = ctrl_server->get_quiddity_manager ();

  switcher::QuiddityManager::CommandHistory histo = 
    manager->get_command_history_from_file (file_name.c_str ());
   if (histo.empty ())
    {
      *result = "false";
      return SOAP_OK;
    }
   manager->play_command_history (histo, NULL, NULL, true); 
   *result = "true";
  return SOAP_OK;
}


