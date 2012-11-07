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

#include "switcher/ctrl-server.h"
#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()
#include "switcher/webservices/control.nsmap"

namespace switcher
{
  QuiddityDocumentation CtrlServer::doc_ ("control", "SOAPcontrolServer",
					  "SOAPcontrolServer allows for managing switcher through SOAP webservices");
    
  CtrlServer::CtrlServer()
  { 
    make_ctrlserver ();
  }

  CtrlServer::CtrlServer(QuiddityLifeManager::ptr life_manager)
  { 
    life_manager_ = life_manager;
    make_ctrlserver ();
  }

  void
  CtrlServer::make_ctrlserver ()
  {
    port_ = 8080;
    soap_init(&soap_);
    //release port
    soap_.connect_flags = SO_LINGER; 
    soap_.accept_flags = SO_LINGER;
    
    soap_.fget = CtrlServer::http_get;
    
    //TODO find a better name for CtrlServer
    srand(time(0));
    set_name (g_strdup_printf ("ctrlserver%d",rand() % 1024));
   
  
  }

  CtrlServer::~CtrlServer ()
  {
    stop ();
  }


 int 
 CtrlServer::http_get (struct soap *soap)
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
       
       QuiddityManager *manager = (QuiddityManager *) soap->user;       
       std::vector<std::string> arg;
       arg.push_back (destination_name);
       if (!manager->invoke (rtpsession_name,"write_sdp_file",arg))
	   return 404;
       
       //sending file to client
       std::string sdp_file = get_file_name_suffix ();
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


  QuiddityDocumentation 
  CtrlServer::get_documentation ()
  {
    return doc_;
  }
  

  void
  CtrlServer::set_quiddity_manager (QuiddityManager::ptr manager)
  {
    soap_.user = (void *)manager.get ();
  }
  
  void 
  CtrlServer::set_port (int port)
  {
    port_ = port;
  }

  void 
  CtrlServer::start ()
  {
    quit_server_thread_ = false;
    service_ = new controlService (soap_);
    SOAP_SOCKET m = service_->bind(NULL, port_, 100 /* BACKLOG */);
    if (!soap_valid_socket(m))
	service_->soap_print_fault(stderr);
    //g_print("Socket connection successful %d\n", m);
    
    thread_ = g_thread_new ("CtrlServer", GThreadFunc(server_thread), this);
  }

  void
  CtrlServer::stop ()
  {
    quit_server_thread_ = true;
    //g_thread_join (thread_);
  }
  
  gpointer
  CtrlServer::server_thread (gpointer user_data)
  {
    CtrlServer *context = static_cast<CtrlServer*>(user_data);
    
    // /* run iterative server on port until fatal error */
    // if (context->service_->run(context->port_))
    //   { context->service_->soap_stream_fault(std::cerr);
    //   }
    // return NULL;

    //for (int i = 1; ; i++)
    while(!context->quit_server_thread_)
      { 
	SOAP_SOCKET s = context->service_->accept();
	if (!soap_valid_socket(s))
	  { if (context->service_->errnum)
	      context->service_->soap_print_fault(stderr);
	    else
	      g_printerr("SOAP server timed out\n");	/* should really wait for threads to terminate, but 24hr timeout should be enough ... */
	    break;
	  }
	// g_print ("client request %d accepted on socket %d, client IP is %d.%d.%d.%d\n", 
	// 	    i, s, 
	// 	    (int)(context->service_->ip>>24)&0xFF, 
	// 	    (int)(context->service_->ip>>16)&0xFF, 
	// 	    (int)(context->service_->ip>>8)&0xFF, 
	// 	    (int)context->service_->ip&0xFF);
	controlService *tcontrol = context->service_->copy();
	tcontrol->serve();
	delete tcontrol;
      }
    if (context->service_)
      delete context->service_;
    return NULL;
  }

}//end of CtrlServer class


// below is the implementation of the service
int 
controlService::add(double a, double b, double *result)
{ *result = a + b;
  return SOAP_OK;
} 

int 
controlService::sub(double a, double b, double *result)
{ *result = a - b;
  return SOAP_OK;
} 

int 
controlService::mul(double a, double b, double *result)
{ *result = a * b;
  return SOAP_OK;
} 

int 
controlService::div(double a, double b, double *result)
{ if (b)
    *result = a / b;
  else
    { char *s = (char*)soap_malloc(this, 1024);
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">Can't divide %f by %f</error>", a, b);
      return soap_senderfault("Division by zero", s);
    }
  return SOAP_OK;
} 

int 
controlService::pow(double a, double b, double *result)
{ *result = ::pow(a, b);
  if (soap_errno == EDOM)	/* soap_errno is like errno, but compatible with Win32 */
    { char *s = (char*)soap_malloc(this, 1024);
      sprintf(s, "Can't take the power of %f to %f", a, b);
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">Can't take power of %f to %f</error>", a, b);
      return soap_senderfault("Power function domain error", s);
    }
  return SOAP_OK;
}

int
controlService::get_factory_capabilities(std::vector<std::string> *result){
  using namespace switcher;
  
  QuiddityManager *manager = (QuiddityManager *) this->user;
  if (this->user == NULL)
    {
      char *s = (char*)soap_malloc(this, 1024);
      g_printerr ("controlService::get_factory_capabilities: cannot get manager (NULL)\n");
      sprintf(s, "<error xmlns=\"http://tempuri.org/\">controlService::get_factory_capabilities: cannot get manager (NULL)</error>");
      return soap_senderfault("error in get_factory_capabilities", s);
    }

  *result = manager->get_classes ();

  return SOAP_OK;
}

int
controlService::get_quiddity_names(std::vector<std::string> *result){
  using namespace switcher;
  
  QuiddityManager *manager = (QuiddityManager *) this->user;
  *result = manager->get_quiddities ();
  
  return SOAP_OK;
}

int
controlService::get_properties_description (std::string quiddity_name,
					    std::string *result)
{
  using namespace switcher;
  
  QuiddityManager *manager = (QuiddityManager *) this->user;
  *result = manager->get_properties_description (quiddity_name);

  return SOAP_OK;
}

int
controlService::get_property_description (std::string quiddity_name,
					  std::string property_name,
					  std::string *result)
{
  using namespace switcher;
  
  QuiddityManager *manager = (QuiddityManager *) this->user;
  *result = manager->get_property_description (quiddity_name, property_name);

  return SOAP_OK;
}


int
controlService::set_property (std::string quiddity_name, 
			      std::string property_name,
			      std::string property_value)
{
  using namespace switcher;
  
  QuiddityManager *manager = (QuiddityManager *) this->user;
  manager->set_property (quiddity_name, property_name, property_value);

  return send_set_property_empty_response(SOAP_OK);
}


int
controlService::get_property (std::string quiddity_name, 
			      std::string property_name,
			      std::string *result)
{
  using namespace switcher;
  
  QuiddityManager *manager = (QuiddityManager *) this->user;
  *result = manager->get_property (quiddity_name, property_name);
  
  return SOAP_OK;
}


int
controlService::create_quiddity (std::string quiddity_class, 
			       std::string *result)
{
  using namespace switcher;
  
   QuiddityManager *manager = (QuiddityManager *) this->user;

   std::string name = manager->create (quiddity_class);
   if (name != "")
     {
       *result = name; 
     }
   else 
     { 
       char *s = (char*)soap_malloc(this, 1024);
       sprintf(s, "%s is not an available class", quiddity_class.c_str());
       sprintf(s, "<error xmlns=\"http://tempuri.org/\">%s is not an available class</error>", quiddity_class.c_str());
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
  
   QuiddityManager *manager = (QuiddityManager *) this->user;

   std::string name = manager->create (quiddity_class, nick_name);
   if (name != "")
     {
       *result = name; 
     }
   else 
     { 
       char *s = (char*)soap_malloc(this, 1024);
       sprintf(s, "%s is not an available class", quiddity_class.c_str());
       sprintf(s, "<error xmlns=\"http://tempuri.org/\">%s is not an available class</error>", quiddity_class.c_str());
       return soap_senderfault("Quiddity creation error", s);
     }
  return SOAP_OK;
}

int
controlService::delete_quiddity (std::string quiddity_name)
{
  using namespace switcher;
  QuiddityManager *manager = (QuiddityManager *) this->user;
  
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
			       bool *result)
{
  using namespace switcher;
  QuiddityManager *manager = (QuiddityManager *) this->user;

  *result = manager->invoke (quiddity_name, method_name, args);

  if (*result)
    return SOAP_OK;
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
  QuiddityManager *manager = (QuiddityManager *) this->user;

  *result = manager->get_methods_description (quiddity_name);
  return SOAP_OK;
}

int
controlService::get_method_description (std::string quiddity_name,
					std::string method_name,
					std::string *result)
{
  using namespace switcher;
  QuiddityManager *manager = (QuiddityManager *) this->user;

  *result = manager->get_method_description (quiddity_name, method_name);
  return SOAP_OK;
}

