/*
 * This file is part of switcher-osc.
 *
 * switcher-osc is free software; you can redistribute it and/or
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

#include "./osc-ctrl-server.hpp"
#include <lo/lo.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <utility>  // std::make_pair (,)

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(OscCtrlServer,
                                     "OSCctl",
                                     "Switcher OSC Controler",
                                     "control",
                                     "",
                                     "OSCcontrolServer allows for managing switcher through OSC",
                                     "LGPL",
                                     "Nicolas Bouillot");

OscCtrlServer::OscCtrlServer(QuiddityConfiguration&& conf)
    : SwitcherWrapper(std::forward<QuiddityConfiguration>(conf)),
      port_(),
      osc_subscribers_(),
      osc_thread_(nullptr) {
  osc_thread_ = nullptr;
  install_method("Set Port",
                 "set_port",
                 "set the port used by the osc server and start listening messages",
                 "success or fail",
                 Method::make_arg_description("Port", "port", "the port to bind", nullptr),
                 (Method::method_ptr)&set_port_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING, nullptr),
                 this);
}

OscCtrlServer::~OscCtrlServer() { stop(); }

void OscCtrlServer::prop_cb(const std::string& internal_subscriber_name,
                            const std::string& quiddity_name,
                            const std::string& property_name,
                            const std::string& value,
                            void* user_data) {
  OscCtrlServer* context = static_cast<OscCtrlServer*>(user_data);

  auto it = context->osc_subscribers_.find(internal_subscriber_name);
  if (context->osc_subscribers_.end() == it) return;

  std::pair<std::string, std::string> address = context->osc_subscribers_[internal_subscriber_name];

  lo_address t = lo_address_new(address.first.c_str(), address.second.c_str());

  gchar* subscriber_name = context->retrieve_subscriber_name(internal_subscriber_name.c_str());
  gchar* message = g_strdup_printf(
      "/property/%s/%s/%s", subscriber_name, quiddity_name.c_str(), property_name.c_str());
  lo_send(t, message, "s", value.c_str());
  g_free(subscriber_name);
  g_free(message);
  lo_address_free(t);
}

std::shared_ptr<Switcher> OscCtrlServer::get_quiddity_manager() { return manager_.lock(); }

gchar* OscCtrlServer::make_internal_subscriber_name(const gchar* name) {
  return g_strdup_printf("%s%s", get_name().c_str(), name);
}

gchar* OscCtrlServer::retrieve_subscriber_name(const gchar* internal_name) {
  gchar* res;
  gchar** split = g_strsplit(internal_name, get_name().c_str(), 2);
  res = g_strdup(split[1]);
  g_strfreev(split);
  return res;
}

// floor
gchar* OscCtrlServer::string_float_to_string_int(const gchar* string_float) {
  gchar* res;
  gchar** split = g_strsplit(string_float, ".", 2);
  res = g_strdup(split[0]);
  g_strfreev(split);
  return res;
}

gboolean OscCtrlServer::set_port_wrapped(gpointer port, gpointer user_data) {
  OscCtrlServer* context = static_cast<OscCtrlServer*>(user_data);
  context->set_port((char*)port);
  return TRUE;
}

void OscCtrlServer::set_port(const std::string& port) {
  stop();
  port_ = port;
  start();
}

void OscCtrlServer::start() {
  osc_thread_ = lo_server_thread_new(port_.c_str(), osc_error);
  /* add method that will match any path and args */
  lo_server_thread_add_method(osc_thread_, nullptr, nullptr, osc_handler, this);
  lo_server_thread_start(osc_thread_);
}

void OscCtrlServer::stop() {
  if (osc_thread_ != nullptr) lo_server_thread_free(osc_thread_);
  osc_thread_ = nullptr;
}

/* catch any osc incoming messages. */
int OscCtrlServer::osc_handler(const char* path,
                               const char* types,
                               lo_arg** argv,
                               int argc,
                               void* /*data */,
                               void* user_data) {
  OscCtrlServer* context = static_cast<OscCtrlServer*>(user_data);
  std::shared_ptr<Switcher> manager = context->get_quiddity_manager();
  if (!(bool)manager) {
    context->warning("OscCtrlServer: cannot get quiddity manager");
    return 0;
  }

  // create
  if (g_str_has_prefix(path, "/c") || g_str_has_prefix(path, "/C")) {
    if (argc == 1) {
      gchar* class_name = string_from_osc_arg(types[0], argv[0]);
      manager->create(class_name);
      g_free(class_name);
    } else if (argc == 2) {
      gchar* class_name = string_from_osc_arg(types[0], argv[0]);
      gchar* quid_name = string_from_osc_arg(types[1], argv[1]);
      manager->create(class_name, quid_name);
      g_free(class_name);
      g_free(quid_name);
    } else
      context->warning("OSCctl: wrong arg number for create");
    return 0;
  }

  // remove
  if (g_str_has_prefix(path, "/r") || g_str_has_prefix(path, "/R")) {
    if (argc == 1) {
      gchar* quid_name = string_from_osc_arg(types[0], argv[0]);
      manager->remove(quid_name);
      g_free(quid_name);
    } else
      context->warning("OSCctl: wrong arg number for remove");
    return 0;
  }

  // set_property
  if (g_str_has_prefix(path, "/s") || g_str_has_prefix(path, "/S")) {
    if (argc == 3) {
      gchar* quid_name = string_from_osc_arg(types[0], argv[0]);
      gchar* prop_name = string_from_osc_arg(types[1], argv[1]);
      gchar* value = string_from_osc_arg(types[2], argv[2]);
      auto id = manager->use_prop<MPtr(&PContainer::get_id)>(quid_name, prop_name);
      if (0 != id) manager->use_prop<MPtr(&PContainer::set_str)>(quid_name, id, value);
      g_free(quid_name);
      g_free(prop_name);
      g_free(value);
    } else
      context->warning("OSCctl: wrong arg number for set_property");
    return 0;
  }

  // invoke
  if (g_str_has_prefix(path, "/i") || g_str_has_prefix(path, "/I")) {
    if (argc >= 2) {
      gchar* quid_name = string_from_osc_arg(types[0], argv[0]);
      gchar* method_name = string_from_osc_arg(types[1], argv[1]);
      int i;
      std::vector<std::string> args;
      for (i = 2; i < argc; i++) {
        gchar* val = string_from_osc_arg(types[i], argv[i]);
        args.push_back(val);
        g_free(val);
      }
      manager->invoke(quid_name, method_name, nullptr, args);
      g_free(quid_name);
      g_free(method_name);
    } else
      context->warning("OSCctl: wrong arg number for invoke");
    return 0;
  }

  // add an osc subscriber
  if (g_str_has_prefix(path, "/a") || g_str_has_prefix(path, "/A")) {
    context->warning("osc subscriber has been disabled in code");
    return 0;
  }

  // delete an osc subscriber
  if (g_str_has_prefix(path, "/d") || g_str_has_prefix(path, "/D")) {
    context->warning("osc subscriber has been disabled in code");
    return 0;
  }

  // subscribe to a property
  if (g_strcmp0(path, "/get") == 0) {
    if (argc == 3) {
      gchar* quiddity_name = string_from_osc_arg(types[0], argv[0]);
      gchar* property_name = string_from_osc_arg(types[1], argv[1]);
      gchar* response_url = string_from_osc_arg(types[2], argv[2]);

      if (quiddity_name == nullptr || property_name == nullptr || response_url == nullptr) {
        context->warning(
            "OscCtrlServer: issue with quiddity name or property name or response url");
        return 0;
      }

      std::string value = manager->use_prop<MPtr(&PContainer::get_str)>(
          quiddity_name,
          manager->use_prop<MPtr(&PContainer::get_id)>(quiddity_name, property_name));
      lo_address response_lo_address = lo_address_new_from_url(response_url);

      if (response_lo_address != nullptr && !lo_address_errno(response_lo_address)) {
        gchar* message = g_strdup_printf("/%s/%s", quiddity_name, property_name);
        lo_send(response_lo_address, message, "s", value.c_str());
        lo_address_free(response_lo_address);
        g_free(message);
      } else
        context->debug("url osc error in get");

      g_free(quiddity_name);
      g_free(property_name);
      g_free(response_url);
    } else
      context->warning("OSCctl: subscribe property needs 3 args (name, quiddity, property)");
    return 0;
  }

  // subscribe to a property
  if (g_str_has_prefix(path, "/get_property_") || g_str_has_prefix(path, "/G")) {
    context->warning("osc subscribe property is disabled in code");
    return 0;
  }

  // unsubscribe to a property
  if (g_str_has_prefix(path, "/u") || g_str_has_prefix(path, "/U")) {
    context->warning("OSC unsubscribe to prop is disabled in code");
    return 0;
  }

  context->debug("unknown osc path %", std::string(path));
  return 0;
}

gchar* OscCtrlServer::string_from_osc_arg(char type, lo_arg* data) {
  // lo_arg_host_endian ((lo_type) type, data);
  gchar* res = nullptr;  // = g_strdup_printf ("videotestsrc");

  gchar* tmp;
  switch (type) {
    case LO_INT32:
      res = g_strdup_printf("%d", data->i);
      break;

    case LO_FLOAT:
      tmp = g_strdup_printf("%f", data->f);
      if (g_str_has_suffix(tmp, ".000000"))  // for pd
      {
        res = string_float_to_string_int(tmp);
        g_free(tmp);
      } else
        res = tmp;
      break;

    case LO_STRING:
      res = g_strdup_printf("%s", (char*)data);
      break;

    case LO_BLOB:
      break;

    case LO_INT64:
      res = g_strdup_printf("%lld", (long long int)data->i);
      break;

    case LO_TIMETAG:
      break;

    case LO_DOUBLE:
      tmp = g_strdup_printf("%f", data->f);
      if (g_str_has_suffix(tmp, ".000000"))  // for pd
      {
        res = string_float_to_string_int(tmp);
        g_free(tmp);
      } else
        res = tmp;
      break;

    case LO_SYMBOL:
      res = g_strdup_printf("'%s", (char*)data);
      break;

    case LO_CHAR:
      res = g_strdup_printf("'%c'", (char)data->c);
      break;

    case LO_MIDI:
      break;
    case LO_TRUE:
      res = g_strdup_printf("true");
      break;

    case LO_FALSE:
      res = g_strdup_printf("false");
      break;

    case LO_NIL:
      res = g_strdup_printf("Nil");
      break;

    case LO_INFINITUM:
      res = g_strdup_printf("Infinitum");
      break;

    default:
      break;
  }
  return res;
}

void OscCtrlServer::osc_error(int /*num*/, const char* /*msg*/, const char* /*path*/) {
  // FIXME debug("liblo server error % in path %: %",
  //       std::to_string(num),
  //       std::string(path),
  //       std::string(msg));
}
}  // end of OscCtrlServer class
