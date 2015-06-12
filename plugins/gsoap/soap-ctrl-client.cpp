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

#include "soap-ctrl-client.hpp"
#include "webservices/control.nsmap"
#include "switcher/gst-utils.hpp"

namespace switcher
{
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    SoapCtrlClient,
    "SOAPcontrolClient",
    "Switcher Web Client (SOAP)",
    "control",
    "",
    "controling a switcher instance through SOAP webservices",
    "GPL",
    "Nicolas Bouillot");

SoapCtrlClient::SoapCtrlClient(const std::string &) :
    custom_props_(new CustomPropertyHelper()){
}

bool
SoapCtrlClient::init()
{
  switcher_control_ = new controlProxy(SOAP_IO_KEEPALIVE | SOAP_XML_INDENT);
  switcher_control_->send_timeout = 1;  // 1 seconds
  switcher_control_->recv_timeout = 1;  // 1 seconds
  url_ = nullptr;
  switcher_control_->soap_endpoint = url_;

  url_prop_ =
      custom_props_->make_string_property("url",
                                          "remote server url",
                                          "",
                                          (GParamFlags)G_PARAM_READABLE,
                                          nullptr,
                                          SoapCtrlClient::get_url,
                                          this);

  install_property_by_pspec(custom_props_->get_gobject(),
                            url_prop_,
                            "url",
                            "Remote URL");
  
  install_method("Set Remote Switcher",
                 "set_remote_url",
                 "set remote url to control (for instance http://localhost:8080)",
                 "success or fail",
                 Method::make_arg_description("URL",
                                              "url",
                                              "SOAP url",
                                              nullptr),
                 (Method::method_ptr) &set_remote_url,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, nullptr),
                 this);

  install_method("Set Remote Switcher URL And Retry Until Success",
                 "set_remote_url_retry",
                 "connect remote url to control until success, each try being notified with the on_connection_tried",
                 "periodic try has been instanciated",
                 Method::make_arg_description("URL",
                                              "url",
                                              "SOAP url",
                                              nullptr),
                 (Method::method_ptr) &set_remote_url_retry,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, nullptr),
                 this);

  GType connection_tried_type[] = {G_TYPE_STRING, G_TYPE_BOOLEAN};
  install_signal("On Connection Tried",
                 "on-connection-tried",
                 "a connection has been tried",
                 Signal::make_arg_description("URL",
                                              "url",
                                              "the remote switcher URL (SOAP control server)",
                                              "Connected",
                                              "connected",
                                              "connection succeed",
                                              nullptr),
                 2,
                 connection_tried_type);


  install_method("Create",
                 "create",
                 "create a quiddity",
                 "success or fail",
                 Method::make_arg_description("Quiddity Class Name",
                                              "class_name",
                                              "the class of the quiddity to create",
                                              "Quiddity Name",
                                              "quiddity_name",
                                              "the name to give",
                                              nullptr),
                 (Method::method_ptr) &create,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_STRING, nullptr),
                 this);


  install_method("Remove",
                 "remove",
                 "remove a quiddity",
                 "success or fail",
                 Method::make_arg_description("Quiddity Name",
                                              "quiddity_name",
                                              "name of quiddity to remove",
                                              nullptr),
                 (Method::method_ptr) &remove,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, nullptr),
                 this);


  install_method("Set Property",
                 "set_property",
                 "set a quiddity property",
                 "success or fail",
                 Method::make_arg_description("Quiddity Name",
                                              "quiddity_name",
                                              "name of quiddity to set",
                                              "Property Name",
                                              "property_name",
                                              "name of the property",
                                              "Property Value",
                                              "property_value",
                                              "value to set",
                                              nullptr),
                 (Method::method_ptr) &set_property,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, nullptr),
                 this);


  install_method("Invoke 1",
                 "invoke1",
                 "invoke a method with one argument",
                 "success or fail",
                 Method::make_arg_description("Quiddity Name ",
                                              "quiddity_name",
                                              "name of quiddity to invoke",
                                              "Method Name",
                                              "method_name",
                                              "name of the method",
                                              "First Argument",
                                              "arg1",
                                              "first argument",
                                              nullptr),
                 (Method::method_ptr) &invoke1,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, nullptr),
                 this);


  install_method("Invoke 2",
                 "invoke2",
                 "invoke a method with two arguments",
                 "success or fail",
                 Method::make_arg_description("quiddity Name",
                                              "quiddity_name",
                                              "name of quiddity to invoke",
                                              "Method Name",
                                              "method_name",
                                              "name of the method",
                                              "First Argument",
                                              "arg1",
                                              "first argument",
                                              "Second Argument",
                                              "arg2",
                                              "second argument",
                                              nullptr),
                 (Method::method_ptr) &invoke2,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_STRING,
                                                   G_TYPE_STRING, G_TYPE_STRING, nullptr),
                 this);


  install_method("Invoke 3",
                 "invoke3",
                 "invoke a method with three arguments",
                 "success or fail",
                 Method::make_arg_description("quiddity Name",
                                              "quiddity_name",
                                              "name of quiddity to invoke",
                                              "Method Name",
                                              "method_name",
                                              "name of the method",
                                              "First Argument",
                                              "arg1",
                                              "first argument",
                                              "Second Argument",
                                              "arg2",
                                              "second argument",
                                              "Third Argument",
                                              "arg3",
                                              "third argument",
                                              nullptr),
                 (Method::method_ptr) &invoke3,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_STRING,
                                                   G_TYPE_STRING, G_TYPE_STRING,
                                                   G_TYPE_STRING, nullptr),
                 this);

  install_method("Invoke 4",
                 "invoke4",
                 "invoke a method with four arguments",
                 "success or fail",
                 Method::make_arg_description("quiddity Name",
                                              "quiddity_name",
                                              "name of quiddity to invoke",
                                              "Method Name",
                                              "method_name",
                                              "name of the method",
                                              "First Argument",
                                              "arg1",
                                              "first argument",
                                              "Second Argument",
                                              "arg2",
                                              "second argument",
                                              "Third Argument",
                                              "arg3",
                                              "third argument",
                                              "Fourth Argument",
                                              "arg4",
                                              "fourth argument",
                                              nullptr),
                 (Method::method_ptr) &invoke4,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, G_TYPE_STRING,
                                                   G_TYPE_STRING, G_TYPE_STRING,
                                                   G_TYPE_STRING, G_TYPE_STRING, nullptr),
                 this);
  return true;
}

SoapCtrlClient::~SoapCtrlClient()
{
  reset_endpoint();
  if (switcher_control_ != nullptr)
    delete switcher_control_;
}

void
SoapCtrlClient::reset_endpoint()
{
  std::unique_lock<std::mutex> lock(try_connect_mutex_);
  if (url_ != nullptr)
    g_free(url_);
  if (nullptr != try_connect_g_source_ && !g_source_is_destroyed(try_connect_g_source_))
  {
    g_source_destroy(try_connect_g_source_);
    try_connect_g_source_ = nullptr;
  }
}

gboolean
SoapCtrlClient::set_remote_url(gpointer url,
                               gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  context->reset_endpoint();
  context->url_ = g_strdup((char *)url);
  context->switcher_control_->soap_endpoint = context->url_;

  std::vector<std::string> resultlist;
  context->switcher_control_->get_quiddity_names(&resultlist);

  if (context->switcher_control_->error)
  {
    g_warning("SoapCtrlClient::set_remote_url, url not valid or not responding");
    g_free(context->url_);
    context->url_ = nullptr;
    return FALSE;
  }
  return TRUE;
}

gboolean
SoapCtrlClient::set_remote_url_retry(gpointer url,
                                     gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  context->reset_endpoint();
  context->url_ = g_strdup((char *)url);
  context->switcher_control_->soap_endpoint = context->url_;
  if (TRUE == context->try_connect(context))
  {
    context->try_connect_g_source_ =
        GstUtils::g_timeout_add_to_context(2000,  //must be higher than gsoap timeouts
                                           try_connect,
                                           context,
                                           context->get_g_main_context());
  }
  return TRUE;
}

gboolean
SoapCtrlClient::try_connect(gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  if (context->url_ == nullptr)
    return FALSE;

  std::unique_lock<std::mutex> lock(context->try_connect_mutex_);

  std::vector<std::string> resultlist;
  context->switcher_control_->get_quiddity_names(&resultlist);

  if (context->switcher_control_->error)
  {
    g_debug("SoapCtrlClient::try_connect (%s) failled, will retry",
            context->url_);
    context->signal_emit("on-connection-tried", context->url_, FALSE);
    return TRUE;
  }
  context->signal_emit("on-connection-tried", context->url_, TRUE);
  return FALSE;
}

gboolean
SoapCtrlClient::create(gpointer class_name,
                       gpointer quiddity_name,
                       gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  if (context->url_ == nullptr){
    return FALSE;
  }
  std::string name;
  context->switcher_control_->create_named_quiddity((const char *)class_name,
                                                    (const char *)quiddity_name,
                                                    &name);
  if (g_strcmp0((gchar *)quiddity_name, name.c_str()) != 0){
    if (!name.empty())
      context->switcher_control_->delete_quiddity(name.c_str());
    return FALSE;
  }
  return TRUE;
}

gboolean
SoapCtrlClient::remove(gpointer quiddity_name,
                       gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  if (context->url_ == nullptr)
    return FALSE;
  context->switcher_control_->delete_quiddity((gchar *)quiddity_name);
  if (context->switcher_control_->error)
    return FALSE;
  return TRUE;
}


gboolean
SoapCtrlClient::set_property(gpointer quiddity_name,
                             gpointer property_name,
                             gpointer value,
                             gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  if (context->url_ == nullptr)
    return FALSE;
  context->switcher_control_->send_set_property((gchar *)quiddity_name,
                                                (gchar *)property_name,
                                                (gchar *)value);
  if (context->switcher_control_->recv_set_property_empty_response())
    return FALSE;//context->switcher_control_->soap_print_fault(stderr);

  // connection should not be kept alive after the last call: be nice to the server and tell it that we close the connection after this call
  soap_clr_omode((context->switcher_control_), SOAP_IO_KEEPALIVE);
  context->switcher_control_->soap_close_socket();
  return TRUE;
}

gboolean
SoapCtrlClient::invoke1(gpointer quiddity_name,
                        gpointer method_name,
                        gpointer arg1,
                        gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  if (context->url_ == nullptr)
    return FALSE;

  std::vector<std::string> args;
  args.push_back((char *) arg1);
  std::string result;
  context->switcher_control_->invoke_method((gchar *)quiddity_name,
                                            (gchar *)method_name,
                                            args,
                                            &result);
  return TRUE;
}

gboolean
SoapCtrlClient::invoke2(gpointer quiddity_name,
                        gpointer method_name,
                        gpointer arg1,
                        gpointer arg2,
                        gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  if (context->url_ == nullptr)
    return FALSE;

  std::vector<std::string> args;
  args.push_back((char *) arg1);
  args.push_back((char *) arg2);
  std::string result;
  context->switcher_control_->invoke_method((gchar *)quiddity_name,
                                            (gchar *)method_name,
                                            args,
                                            &result);
  return TRUE;
}

gboolean
SoapCtrlClient::invoke3(gpointer quiddity_name,
                        gpointer method_name,
                        gpointer arg1,
                        gpointer arg2,
                        gpointer arg3,
                        gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  if (context->url_ == nullptr)
    return FALSE;

  std::vector<std::string> args;
  args.push_back((char *) arg1);
  args.push_back((char *) arg2);
  args.push_back((char *) arg3);
  std::string result;
  context->switcher_control_->invoke_method((gchar *)quiddity_name,
                                            (gchar *)method_name,
                                            args,
                                            &result);
  return TRUE;
}

gboolean
SoapCtrlClient::invoke4(gpointer quiddity_name,
                        gpointer method_name,
                        gpointer arg1,
                        gpointer arg2,
                        gpointer arg3,
                        gpointer arg4,
                        gpointer user_data)
{
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  if (context->url_ == nullptr)
    return FALSE;

  std::vector<std::string> args;
  args.push_back((char *) arg1);
  args.push_back((char *) arg2);
  args.push_back((char *) arg3);
  args.push_back((char *) arg4);
  std::string result;
  context->switcher_control_->invoke_method((gchar *)quiddity_name,
                                            (gchar *)method_name,
                                            args,
                                            &result);
  return TRUE;
}

const gchar *SoapCtrlClient::get_url(void *user_data) {
  SoapCtrlClient *context = static_cast<SoapCtrlClient *>(user_data);
  return context->url_;
}

}
