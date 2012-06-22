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

namespace switcher
{
  
  CtrlServer::CtrlServer() : 
    port_ (8080)
  { 
    soap_init(&soap_);
  }

  CtrlServer::~CtrlServer ()
  {
    stop ();
  }

   void
   CtrlServer::set_user_data (void *user_data)
   {
     soap_.user = user_data;
   }
  
  void 
  CtrlServer::set_port (int port)
  {
    port_ = port;
  }

  void 
  CtrlServer::start ()
  {
    service_ = new controlService (soap_);
    thread_ = g_thread_new ("CtrlServer", GThreadFunc(server_thread), this);
  }

  void
  CtrlServer::stop ()
  {
    if (service_)
      delete service_;
  }
  
  gpointer
  CtrlServer::server_thread (gpointer user_data)
  {
    CtrlServer *context = static_cast<CtrlServer*>(user_data);
    
    /* run iterative server on port until fatal error */
    if (context->service_->run(context->port_))
      { context->service_->soap_stream_fault(std::cerr);
      }
    return NULL;
  }
}


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
controlService::list_factory_capabilities(std::vector<std::string> *result){
  using namespace switcher;

    Factory<BaseEntity, std::string> *factory = 
    (Factory<BaseEntity, std::string> *) this->user;
 
  // //list available object in factory
  // std::vector<std::string> available_object_list = factory->getList ();
  // for (uint i=0; i < available_object_list.size (); i++)
  //   {
  //     std::cout<< "** available object: " << available_object_list[i] << std::endl; 
  //   }    
  
  *result = factory->getList ();
  return SOAP_OK;
}

int
controlService::list_base_entities(std::vector<std::string> *result){
  //  using namespace switcher;

  //TODO use base-entity-factory
  return SOAP_OK;
}
