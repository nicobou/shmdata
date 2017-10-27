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

#include "soap-ctrl-server.hpp"
#include "switcher/file-utils.hpp"
#include "switcher/information-tree-json.hpp"
#include "switcher/scope-exit.hpp"
#include "webservices/control.nsmap"

// hacking gsoap bug for ubuntu 13.10
#ifdef WITH_IPV6
#define SOAPBINDTO "::"
#else
#define SOAPBINDTO nullptr
#endif

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(SoapCtrlServer,
                                     "SOAPcontrolServer",
                                     "Switcher Web Controler (SOAP)",
                                     "control",
                                     "",
                                     "getting switcher controled through SOAP webservices",
                                     "GPL",
                                     "Nicolas Bouillot");

SoapCtrlServer::SoapCtrlServer(QuiddityConfiguration&& conf)
    : SwitcherWrapper(std::forward<QuiddityConfiguration>(conf)) {
  soap_init(&soap_);
  // release port
  soap_.connect_flags = SO_LINGER;
  soap_.accept_flags = SO_LINGER;
  soap_.accept_timeout = 100 * -1000;  // 100ms
  soap_.fget = SoapCtrlServer::http_get;

  install_method("Set Port",
                 "set_port",
                 "set the port used by the soap server",
                 "success or fail",
                 Method::make_arg_description("Port", "port", "the port to bind", nullptr),
                 (Method::method_ptr)&set_port_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_INT, nullptr),
                 this);
}

SoapCtrlServer::~SoapCtrlServer() {
  quit_server_thread_ = true;
  if (thread_.joinable()) thread_.join();
  if (-1 != socket_) soap_closesocket(socket_);
  if (nullptr != service_) {
    soap_destroy(service_);
    soap_end(service_);
    soap_done(service_);
    delete service_;
  }
  soap_destroy(&soap_);
  soap_end(&soap_);
  soap_done(&soap_);
}

std::shared_ptr<Switcher> SoapCtrlServer::get_quiddity_manager() { return manager_.lock(); }

int SoapCtrlServer::http_get(struct soap* soap) {
  std::string rtpsession_name;
  std::string destination_name;

  if (g_str_has_prefix(soap->path, "/sdp")) {
    gchar** query = g_strsplit_set(soap->path, "?", -1);
    if (query[1] == nullptr) return 404;
    gchar** query_vars = g_strsplit_set(query[1], "&", -1);
    int i = 0;
    while (query_vars[i] != nullptr) {
      gchar** var = g_strsplit_set(query_vars[i], "=", -1);
      if (g_strcmp0(var[0], "rtpsession") == 0 && var[1] != nullptr) {
        rtpsession_name.clear();
        rtpsession_name.append(var[1]);
      } else if (g_strcmp0(var[0], "destination") == 0 && var[1] != nullptr) {
        destination_name.clear();
        destination_name.append(var[1]);
      }
      g_strfreev(var);
      i++;
    }
    g_strfreev(query_vars);
    g_strfreev(query);
    SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(soap->user);
    Switcher::ptr manager;
    if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();
    if (!(bool)manager) return 404;
    std::vector<std::string> arg;
    arg.push_back(destination_name);
    if (!manager->invoke(rtpsession_name, "write_sdp_file", nullptr, arg)) return 404;
    // sending file to client
    std::string sdp_file = get_socket_dir();
    sdp_file.append("/");
    sdp_file.append(get_socket_name_prefix());
    sdp_file.append(manager->get_name() + "_" + rtpsession_name + "_" + destination_name + ".sdp");
    gchar* sdp_contents = nullptr;
    gsize file_length;
    if (g_file_get_contents(sdp_file.c_str(),
                            &sdp_contents,
                            &file_length,
                            nullptr) &&
        0 != file_length) {  // not getting errors
      On_scope_exit { g_free(sdp_contents); };
      soap_response(soap, SOAP_FILE);
      soap->http_content = "application/x-sdp";
      soap_send_raw(soap, sdp_contents, file_length);
      return soap_end_send(soap);
    } else
      ctrl_server->warning("cannot get sdp description for rtpsession %, destination %",
                           rtpsession_name,
                           destination_name);
  }
  return 404;
}

gboolean SoapCtrlServer::set_port_wrapped(gint port, gpointer user_data) {
  SoapCtrlServer* context = static_cast<SoapCtrlServer*>(user_data);
  if (context->set_port(port))
    return TRUE;
  else
    return FALSE;
}

bool SoapCtrlServer::set_port(int port) {
  if (0 != port_) {
    warning("soap port can be set only once");
    return false;
  }
  port_ = port;
  if (!start()) {
    return false;
  }
  disable_method("set_port");
  return true;
}

bool SoapCtrlServer::start() {
  soap_.user = (void*)this;
  quit_server_thread_ = false;
  service_ = new controlService(soap_);
  socket_ = service_->bind(SOAPBINDTO, port_, 100 /* BACKLOG */);
  if (!soap_valid_socket(socket_)) {
    service_->soap_print_fault(stderr);
    delete service_;
    service_ = nullptr;
    return false;
  }

  thread_ = std::thread(&SoapCtrlServer::server_thread, this);
  return true;
}

// gboolean
// SoapCtrlServer::stop_wrapped (gpointer user_data)
// {
//   SoapCtrlServer *context = static_cast<SoapCtrlServer*>(user_data);
//   context->stop ();
//   return TRUE;
// }

// bool
// SoapCtrlServer::stop()
// {
//   quit_server_thread_ = true;
//   if (thread_.joinable())
//     thread_.join();
//   else
//     g_print("++++++++++++++++++++++++++++++ !!!!!! thread not joinable\n");
//   //soap_closesocket(socket_);
//   soap_destroy(&soap_);
//   soap_end(&soap_);
//   soap_done(&soap_);
//   if (nullptr != service_)
//     delete service_;
//   return true;
// }

void SoapCtrlServer::server_thread() {
  // /* run iterative server on port until fatal error */
  // if (context->service_->run(context->port_))
  //   { context->service_->soap_stream_fault(std::cerr);
  //   }
  // return nullptr;

  // for (int i = 1; ; i++)
  while (!quit_server_thread_) {
    SOAP_SOCKET s = service_->accept();
    if (!soap_valid_socket(s)) {
      if (service_->errnum)
        service_->soap_print_fault(stderr);
    } else {
      std::unique_lock<std::mutex> lock(mutex_);
      controlService* tcontrol = service_->copy();
      if (service_->errnum) service_->soap_print_fault(stderr);
      tcontrol->serve();
      delete tcontrol;
    }
  }
}

}  // end of SoapCtrlServer class

/**********************************************
 * below is the implementation of the service *
 **********************************************/

int controlService::get_classes(std::vector<std::string>* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  if (ctrl_server == nullptr || !(bool)manager) {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s, "controlService::get_classes: cannot get manager (nullptr)");
    return soap_senderfault("error in get_classes", s);
  }

  *result = manager->get_classes();

  return SOAP_OK;
}

int controlService::get_classes_doc(std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  if (ctrl_server == nullptr || !(bool)manager) {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s, "controlService::get_classes: cannot get manager (nullptr)");
    return soap_senderfault("error in get_classes_doc", s);
  }

  *result = manager->get_classes_doc();

  return SOAP_OK;
}

int controlService::get_quiddity_description(std::string quiddity_name, std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  if (ctrl_server == nullptr || !(bool)manager) {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s, "controlService::get_classes: cannot get manager (nullptr)");
    return soap_senderfault("error in get_class_doc", s);
  }

  *result = manager->get_quiddity_description(quiddity_name);

  return SOAP_OK;
}

int controlService::get_quiddities_description(std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  if (ctrl_server == nullptr || !(bool)manager) {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s,
            "controlService::get_quiddities_description: cannot get manager "
            "(nullptr)");
    return soap_senderfault("error in get_classes_doc", s);
  }

  *result = manager->get_quiddities_description();

  return SOAP_OK;
}

int controlService::get_class_doc(std::string class_name, std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  if (ctrl_server == nullptr || !(bool)manager) {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s, "controlService::get_classes: cannot get manager (nullptr)");
    return soap_senderfault("error in get_class_doc", s);
  }

  *result = manager->get_class_doc(class_name);

  return SOAP_OK;
}

int controlService::get_quiddity_names(std::vector<std::string>* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  *result = manager->get_quiddities();

  return SOAP_OK;
}

int controlService::set_property(std::string quiddity_name,
                                 std::string property_name,
                                 std::string property_value) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  auto id = manager->use_prop<MPtr(&PContainer::get_id)>(quiddity_name, property_name);
  if (0 == id) {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(
        s, "property %s not found for quiddity %s", property_name.c_str(), quiddity_name.c_str());
    return soap_senderfault("set property error", s);
  }
  manager->use_prop<MPtr(&PContainer::set_str)>(quiddity_name, id, property_value);
  return send_set_property_empty_response(SOAP_OK);
}

int controlService::get_property(std::string quiddity_name,
                                 std::string property_name,
                                 std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  auto id = manager->use_prop<MPtr(&PContainer::get_id)>(quiddity_name, property_name);
  if (0 == id)
    *result = std::string();
  else
    *result = manager->use_prop<MPtr(&PContainer::get_str)>(quiddity_name, id);
  return SOAP_OK;
}

int controlService::create_quiddity(std::string quiddity_class, std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  std::string name = manager->create(quiddity_class);
  if (!name.empty()) {
    *result = name;
  } else {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s, "%s cannot be created, see switcher logs", quiddity_class.c_str());
    return soap_senderfault("Quiddity creation error", s);
  }
  return SOAP_OK;
}

int controlService::create_named_quiddity(std::string quiddity_class,
                                          std::string nick_name,
                                          std::string* result) {
  using namespace switcher;

  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  std::string name = manager->create(quiddity_class, nick_name);
  if (!name.empty()) {
    *result = name;
  } else {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s, "%s cannot be created, see switcher logs", quiddity_class.c_str());
    return soap_senderfault("Quiddity creation error", s);
  }
  return SOAP_OK;
}

int controlService::delete_quiddity(std::string quiddity_name) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  if (manager->remove(quiddity_name))
    return send_set_property_empty_response(SOAP_OK);
  else {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s, "%s is not found, not deleting", quiddity_name.c_str());
    return send_set_property_empty_response(soap_senderfault("Quiddity creation error", s));
  }
}

int controlService::invoke_method(std::string quiddity_name,
                                  std::string method_name,
                                  std::vector<std::string> args,
                                  std::string* result) {
  using namespace switcher;

  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();
  std::string* invocation_result;
  if (manager->invoke(quiddity_name, method_name, &invocation_result, args)) {
    *result = *invocation_result;
    return SOAP_OK;
  } else {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s, "invoking %s/%s returned false", quiddity_name.c_str(), method_name.c_str());
    return soap_senderfault("Method invocation error", s);
  }
}

int controlService::get_methods_description(std::string quiddity_name, std::string* result) {
  using namespace switcher;

  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  *result = manager->get_methods_description(quiddity_name);
  return SOAP_OK;
}

int controlService::get_method_description(std::string quiddity_name,
                                           std::string method_name,
                                           std::string* result) {
  using namespace switcher;

  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  *result = manager->get_method_description(quiddity_name, method_name);
  return SOAP_OK;
}

int controlService::get_methods_description_by_class(std::string class_name, std::string* result) {
  using namespace switcher;

  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  *result = manager->get_methods_description_by_class(class_name);
  return SOAP_OK;
}

int controlService::get_method_description_by_class(std::string class_name,
                                                    std::string method_name,
                                                    std::string* result) {
  using namespace switcher;

  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  *result = manager->get_method_description_by_class(class_name, method_name);
  return SOAP_OK;
}

int controlService::save(std::string file_name, std::string* result) {
  using namespace switcher;

  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  if (FileUtils::save(JSONSerializer::serialize(manager->get_state().get()), file_name))
    *result = "true";
  else
    *result = "false";
  return SOAP_OK;
}

int controlService::load(std::string file_name, std::string* result) {
  using namespace switcher;

  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  manager->reset_state(true);

  if (!manager->load_state(JSONSerializer::deserialize(FileUtils::get_content(file_name)))) {
    *result = "false";
    return SOAP_OK;
  }
  *result = "true";
  return SOAP_OK;
}

int controlService::run(std::string file_name, std::string* result) {
  using namespace switcher;

  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  if (!manager->load_state(JSONSerializer::deserialize(FileUtils::get_content(file_name)))) {
    *result = "false";
    return SOAP_OK;
  }
  *result = "true";
  return SOAP_OK;
}

int controlService::get_information_tree(std::string quiddity_name,
                                         std::string path,
                                         std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  *result = manager->use_tree<MPtr(&InfoTree::serialize_json)>(quiddity_name, path);

  return SOAP_OK;
}

int controlService::get_user_data(std::string quiddity_name,
                                  std::string path,
                                  std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  *result = manager->user_data<MPtr(&InfoTree::serialize_json)>(quiddity_name, path);

  return SOAP_OK;
}

int controlService::prune_user_data(std::string quiddity_name,
                                    std::string path,
                                    std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  *result = static_cast<bool>(manager->user_data<MPtr(&InfoTree::prune)>(quiddity_name, path))
                ? "true"
                : "false";

  return SOAP_OK;
}

int controlService::graft_user_data(std::string quiddity_name,
                                    std::string path,
                                    std::string type,
                                    std::string value,
                                    std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  auto res = false;
  if (type == "int") {
    auto deserialized = deserialize::apply<int>(value);
    if (deserialized.first &&
        manager->user_data<MPtr(&InfoTree::graft)>(
            quiddity_name, path, InfoTree::make(deserialized.second)))
      res = true;
  } else if (type == "float") {
    auto deserialized = deserialize::apply<float>(value);
    if (deserialized.first &&
        manager->user_data<MPtr(&InfoTree::graft)>(
            quiddity_name, path, InfoTree::make(deserialized.second)))
      res = true;
  } else if (type == "bool") {
    auto deserialized = deserialize::apply<bool>(value);
    if (deserialized.first &&
        manager->user_data<MPtr(&InfoTree::graft)>(
            quiddity_name, path, InfoTree::make(deserialized.second)))
      res = true;
  } else if (type == "string") {
    if (manager->user_data<MPtr(&InfoTree::graft)>(quiddity_name, path, InfoTree::make(value)))
      res = true;
  } else {
    char* s = (char*)soap_malloc(this, 1024);
    sprintf(s, "type not handled with soap");
    return soap_senderfault("error in get_classes", s);
  }

  if (res)
    *result = "true";
  else
    *result = "false";

  return SOAP_OK;
}

int controlService::tag_as_array_user_data(std::string quiddity_name,
                                           std::string path,
                                           bool is_array,
                                           std::string* result) {
  using namespace switcher;
  SoapCtrlServer* ctrl_server = static_cast<SoapCtrlServer*>(this->user);
  Switcher::ptr manager;
  if (ctrl_server != nullptr) manager = ctrl_server->get_quiddity_manager();

  *result = static_cast<bool>(
                manager->user_data<MPtr(&InfoTree::tag_as_array)>(quiddity_name, path, is_array))
                ? "true"
                : "false";

  return SOAP_OK;
}
