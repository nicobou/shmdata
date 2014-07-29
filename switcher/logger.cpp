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

#include "logger.h"

namespace switcher
{

  bool Logger::installed_ = false;

  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(Logger,
				       "Switcher Logger",
				       "log", 
				       "manage switcher logs and other glib log domains.",
				       "LGPL",
				       "logger",
				       "Nicolas Bouillot");

  Logger::Logger () :
    i_am_the_one_ (false),
    last_line_ (),
    mute_ (false),
    debug_ (true),
    verbose_ (true),
    handler_ids_ (),
    custom_props_ (new CustomPropertyHelper ()),
    last_line_prop_ (nullptr),
    mute_prop_ (nullptr),
    debug_prop_ (nullptr),
    verbose_prop_ (nullptr),
    last_line_mutex_ ()
  {}
  
  bool
  Logger::init()
  {
    if (installed_)
      {
	g_warning ("Only one logger instance is possible, cannot create");
	return false;
      }
    else
      {
	installed_ = true;
	i_am_the_one_ = true;
      }
    
    //FIXME: make the following not necessary, 
    // avoid the following warnings:
    // Attempt to add property MyObject::customprop1 after class was initialised
    guint quiet_handler_id = g_log_set_handler ("GLib-GObject", 
						G_LOG_LEVEL_MASK, 
						quiet_log_handler, 
						nullptr);  

    last_line_prop_ = 
      custom_props_->make_string_property ("last-line", 
					   "last log line",
					   "",
					   (GParamFlags) G_PARAM_READABLE,
					   nullptr,
					   Logger::get_last_line,
					   this);
    
    install_property_by_pspec (custom_props_->get_gobject (), 
				last_line_prop_, 
				"last-line",
				"Last Line");
    
    mute_prop_ = 
      custom_props_->make_boolean_property ("mute", 
					    "mute log messages",
					    (gboolean)FALSE,
					    (GParamFlags) G_PARAM_READWRITE,
					    Logger::set_mute,
					    Logger::get_mute,
					    this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				mute_prop_, 
				"mute",
				"Mute");

    debug_prop_ = 
      custom_props_->make_boolean_property ("debug", 
					    "enable debug messages",
					    (gboolean)TRUE,
					    (GParamFlags) G_PARAM_READWRITE,
					    Logger::set_debug,
					    Logger::get_debug,
					    this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				debug_prop_, 
				"debug",
				"Debug");

    verbose_prop_ = 
      custom_props_->make_boolean_property ("verbose", 
					    "enable log messages",
					    TRUE,
					    (GParamFlags) G_PARAM_READWRITE,
					    Logger::set_verbose,
					    Logger::get_verbose,
					    this);
    install_property_by_pspec (custom_props_->get_gobject (), 
				verbose_prop_, 
				"verbose",
				"Verbose");

    
    
    //handler must be installed after custom property creation 
    handler_ids_["switcher"] = g_log_set_handler ("switcher", 
						  G_LOG_LEVEL_MASK, 
						  log_handler, 
						  this);
    
    g_log_remove_handler ("GLib-GObject", quiet_handler_id);

    install_method ("Install Log Handler",
		    "install_log_handler", 
		    "make the logger managing the log domain", 
		    "success or fail",
		    Method::make_arg_description ("LogDomain",
						  "log domain", 
						  "the glib log domain (e.g. shmdata, Glib or GStreamer)",
						  nullptr),
		    (Method::method_ptr) &install_log_handler_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr),
		    this);


    install_method ("Remove Log Handler",
		    "remove_log_handler", 
		    "make the logger stop managing the log domain", 
		    "success or fail",
		    Method::make_arg_description ("Log Domain",
						  "log domain", 
						  "the glib log domain (e.g. shmdata, Glib or GStreamer)",
						  nullptr),
		    (Method::method_ptr) &remove_log_handler_wrapped, 
		    G_TYPE_BOOLEAN,
		    Method::make_arg_type_description (G_TYPE_STRING, nullptr),
		    this);

    return true;
  }

  void
  Logger::quiet_log_handler (const gchar */*log_domain*/, 
			     GLogLevelFlags /*log_level*/,
			     const gchar */*message*/,
			     gpointer /*user_data*/)
  {}
  
  Logger::~Logger()
  {
    if (i_am_the_one_)
      {
	for (auto &it : handler_ids_)
	  g_log_remove_handler (it.first.c_str (), it.second);
	installed_ = false;
      }
  }
    
  
  gboolean
  Logger::install_log_handler_wrapped (gpointer log_domain, gpointer user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    return context->install_log_handler ((gchar *)log_domain);
  }

  gboolean
  Logger::remove_log_handler_wrapped (gpointer log_domain, gpointer user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    return context->remove_log_handler ((gchar *)log_domain);
  }


  gboolean
  Logger::install_log_handler (const gchar *log_domain)
  {
    auto it = handler_ids_.find (log_domain);
    if (handler_ids_.end () != it)
      return FALSE;

    handler_ids_[log_domain] = g_log_set_handler (log_domain, 
						  G_LOG_LEVEL_MASK, 
						  log_handler, 
						  this);
    return TRUE;
  }

  gboolean
  Logger::remove_log_handler (const gchar *log_domain)
  {
    auto it = handler_ids_.find (log_domain);
    if (handler_ids_.end () == it)
      return FALSE;

    g_log_remove_handler (log_domain, handler_ids_[log_domain]);
    return TRUE;
  }
  
  void 
  Logger::replace_last_line(std::string next_line)
  {
    std::unique_lock<std::mutex> lock (last_line_mutex_); 
    last_line_ = next_line;
  }

  void
  Logger::log_handler (const gchar *log_domain, 
		       GLogLevelFlags log_level,
		       const gchar *message,
		       gpointer user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    if (context->mute_)
      return;
    gboolean update_last_line = TRUE;
    std::string tmp_message = std::string ((nullptr == message) ? "null-message" : message);
    std::string tmp_log_domain = std::string ((nullptr == log_domain) ? "null-log-domain" : log_domain);
    std::string tmp_level = std::string ("unknown");

    //FIXME: 
    if (0 == tmp_log_domain.compare ("GLib-GObject") 
	&& 0 == tmp_message.compare  (0, 23, "Attempt to add property"))
      return;

    switch (log_level) {
    case G_LOG_LEVEL_ERROR:
      tmp_level = std::string ("error");
      break;
    case G_LOG_LEVEL_CRITICAL:
      tmp_level = std::string ("critical");
      break;
    case G_LOG_LEVEL_WARNING:
      tmp_level = std::string ("warning");
      break;
    case G_LOG_LEVEL_MESSAGE:
      if (context->debug_ || context->verbose_)
	tmp_level = std::string ("message");
      else
	update_last_line = FALSE;
      break;
    case G_LOG_LEVEL_INFO:
      if (context->debug_ || context->verbose_)
	tmp_level = std::string ("info");
      else
	update_last_line = FALSE;
      break;
    case G_LOG_LEVEL_DEBUG:
      if (context->debug_)
	tmp_level = std::string ("debug");
      else
	update_last_line = FALSE;
      break;
    default:
      break;
    }
    if (update_last_line)
      {
	context->replace_last_line (tmp_log_domain + "-" + tmp_level + ": " + tmp_message);
	context->custom_props_->notify_property_changed (context->last_line_prop_);
      }
  }

  const gchar *
  Logger::get_last_line (void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    std::unique_lock<std::mutex> lock (context->last_line_mutex_); 
    return context->last_line_.c_str ();
  }

  
  gboolean 
  Logger::get_mute (void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    return context->mute_;
  }

  void 
  Logger::set_mute (gboolean mute, void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    context->mute_ = mute;
  }
  
  gboolean 
  Logger::get_debug (void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    return context->debug_;
  }

  void 
  Logger::set_debug (gboolean debug, void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    context->debug_ = debug;
  }
  
  gboolean 
  Logger::get_verbose (void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    return context->verbose_;
  }

  void 
  Logger::set_verbose (gboolean verbose, void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    context->verbose_ = verbose;
  }
  
}
