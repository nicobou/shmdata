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

#include "./logger.hpp"

namespace switcher {
bool Logger::installed_ = false;

SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    Logger,
    "logger",
    "Switcher Logger",
    "utils",
    "",
    "manage switcher logs and other glib log domains.",
    "LGPL",
    "Nicolas Bouillot");

Logger::Logger(const std::string &) :
    last_line_id_(pmanage<MPtr(&PContainer::make_string)>(
        "last-line",
        nullptr,
        [this](){
          std::unique_lock<std::mutex> lock(last_line_mutex_);
          return last_line_;
        },
        "Last Log Line",
        "Provide last log line",
        last_line_)){
  pmanage<MPtr(&PContainer::make_bool)>(
      "mute",
      [this](bool val){mute_ = val; return true;},
      [this](){return mute_;},
      "Mute",
      "Mute log messages",
      mute_);
  
  pmanage<MPtr(&PContainer::make_bool)>(
      "debug",
      [this](bool val){debug_ = val; return true;},
      [this](){return debug_;},
      "Debug",
      "Enable debug log level",
      debug_);

  pmanage<MPtr(&PContainer::make_bool)>(
      "verbose",
      [this](bool val){verbose_ = val; return true;},
      [this](){return verbose_;},
      "Verbose",
      "Enable verbose log level",
      verbose_);
}

bool Logger::init() {
  if (installed_) {
    g_warning("Only one logger instance is possible, cannot create");
    return false;
  } else {
    installed_ = true;
    i_am_the_one_ = true;
  }

  guint quiet_handler_id = g_log_set_handler("GLib-GObject",
                                             G_LOG_LEVEL_MASK,
                                             quiet_log_handler,
                                             nullptr);

  // handler must be installed after custom property creation
  handler_ids_["switcher"] = g_log_set_handler("switcher",
                                               G_LOG_LEVEL_MASK,
                                               log_handler,
                                               this);

  g_log_remove_handler("GLib-GObject", quiet_handler_id);

  install_method("Install Log Handler",
                 "install_log_handler",
                 "make the logger managing the log domain",
                 "success or fail",
                 Method::make_arg_description("LogDomain",
                                              "log domain",
                                              "the glib log domain (e.g. shmdata, Glib or GStreamer)",
                                              nullptr),
                 (Method::method_ptr) &install_log_handler_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr), this);

  install_method("Remove Log Handler",
                 "remove_log_handler",
                 "make the logger stop managing the log domain",
                 "success or fail",
                 Method::make_arg_description("Log Domain",
                                              "log domain",
                                              "the glib log domain (e.g. shmdata, Glib or GStreamer)",
                                              nullptr),
                 (Method::method_ptr) &remove_log_handler_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr), this);

  return true;
}

void Logger::quiet_log_handler(const gchar * /*log_domain */ ,
                               GLogLevelFlags /*log_level */ ,
                               const gchar * /*message */ ,
                               gpointer /*user_data */ ) {
}

Logger::~Logger() {
  if (i_am_the_one_) {
    for (auto &it : handler_ids_)
      g_log_remove_handler(it.first.c_str(), it.second);
    installed_ = false;
  }
}

gboolean
Logger::install_log_handler_wrapped(gpointer log_domain,
                                    gpointer user_data) {
  Logger *context = static_cast<Logger *>(user_data);
  return context->install_log_handler((gchar *) log_domain);
}

gboolean
Logger::remove_log_handler_wrapped(gpointer log_domain,
                                   gpointer user_data) {
  Logger *context = static_cast<Logger *>(user_data);
  return context->remove_log_handler((gchar *) log_domain);
}

gboolean Logger::install_log_handler(const gchar *log_domain) {
  auto it = handler_ids_.find(log_domain);
  if (handler_ids_.end() != it)
    return FALSE;

  handler_ids_[log_domain] = g_log_set_handler(log_domain,
                                               G_LOG_LEVEL_MASK,
                                               log_handler, this);
  return TRUE;
}

gboolean Logger::remove_log_handler(const gchar *log_domain) {
  auto it = handler_ids_.find(log_domain);
  if (handler_ids_.end() == it)
    return FALSE;

  g_log_remove_handler(log_domain, handler_ids_[log_domain]);
  return TRUE;
}

void Logger::replace_last_line(std::string next_line) {
  std::unique_lock<std::mutex> lock(last_line_mutex_);
  last_line_ = next_line;
}

void
Logger::log_handler(const gchar *log_domain,
                    GLogLevelFlags log_level,
                    const gchar *message, gpointer user_data) {
  Logger *context = static_cast<Logger *>(user_data);
  if (context->mute_)
    return;
  gboolean update_last_line = TRUE;
  std::string tmp_message =
      std::string((nullptr == message) ? "null-message" : message);
  std::string tmp_log_domain =
      std::string((nullptr == log_domain) ? "null-log-domain" : log_domain);
  std::string tmp_level = std::string("unknown");

  switch (log_level) {
    case G_LOG_LEVEL_ERROR:
      tmp_level = std::string("error");
      break;
    case G_LOG_LEVEL_CRITICAL:
      tmp_level = std::string("critical");
      break;
    case G_LOG_LEVEL_WARNING:
      tmp_level = std::string("warning");
      break;
    case G_LOG_LEVEL_MESSAGE:
      if (context->debug_ || context->verbose_)
        tmp_level = std::string("message");
      else
        update_last_line = FALSE;
      break;
    case G_LOG_LEVEL_INFO:
      if (context->debug_ || context->verbose_)
        tmp_level = std::string("info");
      else
        update_last_line = FALSE;
      break;
    case G_LOG_LEVEL_DEBUG:
      if (context->debug_)
        tmp_level = std::string("debug");
      else
        update_last_line = FALSE;
      break;
    default:
      break;
  }

  if (update_last_line){
    {
      auto lock = context->pmanage<MPtr(&PContainer::get_lock)>(context->last_line_id_);
      context->last_line_ = tmp_log_domain + "-" + tmp_level + ": " + tmp_message;
    }
    context->pmanage<MPtr(&PContainer::notify)>(context->last_line_id_);
  }
}

}  // namespace switcher
