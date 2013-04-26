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

#include "logger.h"

namespace switcher
{

  bool Logger::installed_ = false;
  
  QuiddityDocumentation Logger::doc_  ("log", "logger",
				       "manage switcher logs and other glib log domains.");
  bool
  Logger::init()
  {
    i_am_the_one_ = false;
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
    set_name ("logger");
    
    custom_props_.reset (new CustomPropertyHelper ());
    mute_ = false;
    debug_ = true;
    verbose_ = true;
    last_line_ = g_strdup ("");

    last_line_prop_ = 
      custom_props_->make_string_property ("last-line", 
					   "last log line",
					   "",
					   (GParamFlags) G_PARAM_READABLE,
					   NULL,
					   Logger::get_last_line,
					   this);
    
    register_property_by_pspec (custom_props_->get_gobject (), 
				last_line_prop_, 
				"last-line");
    
    mute_prop_ = 
      custom_props_->make_boolean_property ("mute", 
					    "mute log messages",
					    (gboolean)FALSE,
					    (GParamFlags) G_PARAM_READWRITE,
					    Logger::set_mute,
					    Logger::get_mute,
					    this);
    register_property_by_pspec (custom_props_->get_gobject (), 
				mute_prop_, 
				"mute");

    debug_prop_ = 
      custom_props_->make_boolean_property ("debug", 
					    "enable debug messages",
					    (gboolean)TRUE,
					    (GParamFlags) G_PARAM_READWRITE,
					    Logger::set_debug,
					    Logger::get_debug,
					    this);
    register_property_by_pspec (custom_props_->get_gobject (), 
				debug_prop_, 
				"debug");

    verbose_prop_ = 
      custom_props_->make_boolean_property ("verbose", 
					    "enable log messages",
					    TRUE,
					    (GParamFlags) G_PARAM_READWRITE,
					    Logger::set_verbose,
					    Logger::get_verbose,
					    this);
    register_property_by_pspec (custom_props_->get_gobject (), 
				verbose_prop_, 
				"verbose");
    
    //handler must be installed after custom property creation 
    handler_ids_.insert ("switcher",
			 g_log_set_handler ("switcher", 
					    G_LOG_LEVEL_MASK, 
					    log_handler, 
					    this));
    
    
    //registering install
    register_method("install_log_handler",
		    (void *)&install_log_handler_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("install_log_handler", 
			    "make the logger managing the log domain", 
			    Method::make_arg_description ("log domain", 
							  "the glib log domain (e.g. shmdata, Glib or GStreamer)",
							  NULL));

    //registering remove
    register_method("remove_log_handler",
		    (void *)&remove_log_handler_wrapped, 
		    Method::make_arg_type_description (G_TYPE_STRING, NULL),
		    (gpointer)this);
    set_method_description ("remove_log_handler", 
			    "make the logger stop managing the log domain", 
			    Method::make_arg_description ("log domain", 
							  "the glib log domain (e.g. shmdata, Glib or GStreamer)",
							  NULL));


    return true;
  }

  Logger::~Logger()
  {
    if (i_am_the_one_)
      {
	std::map<std::string, guint> handlers = handler_ids_.get_map ();
	std::map<std::string, guint>::iterator it;
	for (it = handlers.begin (); it != handlers.end (); it++)
	  g_log_remove_handler (it->first.c_str (), it->second);
	
	g_free (last_line_);
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
    if (handler_ids_.contains (log_domain))
      return FALSE;

    handler_ids_.insert (log_domain,
			 g_log_set_handler (log_domain, 
					    G_LOG_LEVEL_MASK, 
					    log_handler, 
					    this));
    return TRUE;
  }

  gboolean
  Logger::remove_log_handler (const gchar *log_domain)
  {
    if (!handler_ids_.contains (log_domain))
      return FALSE;

    g_log_remove_handler (log_domain, handler_ids_.lookup(log_domain));
    return TRUE;
  }
  
  void 
  Logger::replace_last_line(gchar *next_line)
  {
    gchar *old_line = last_line_;
    last_line_ = next_line;
    g_free (old_line);
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

     switch (log_level) {
     case G_LOG_LEVEL_ERROR:
       context->replace_last_line(g_strdup_printf ("%s-error: %s",log_domain, message));
       break;
     case G_LOG_LEVEL_CRITICAL:
       context->replace_last_line(g_strdup_printf ("%s-critical: %s",log_domain, message));
       break;
     case G_LOG_LEVEL_WARNING:
       context->replace_last_line(g_strdup_printf ("%s-warning: %s",log_domain, message));
       break;
     case G_LOG_LEVEL_MESSAGE:
       if (context->debug_ || context->verbose_)
	   context->replace_last_line(g_strdup_printf ("%s-message: %s",log_domain, message));
       else
	 update_last_line = FALSE;
       break;
     case G_LOG_LEVEL_INFO:
       if (context->debug_ || context->verbose_)
	   context->replace_last_line(g_strdup_printf ("%s-info: %s",log_domain, message));
       else
	 update_last_line = FALSE;
       break;
     case G_LOG_LEVEL_DEBUG:
       if (context->debug_)
	 context->replace_last_line(g_strdup_printf ("%s-debug: %s",log_domain, message));
       else
	 update_last_line = FALSE;
       break;
     default:
       context->replace_last_line(g_strdup_printf ("%s-unknown-level: %s",log_domain,message));
       break;
     }

     if (update_last_line)
       context->custom_props_->notify_property_changed (context->last_line_prop_);
}


  QuiddityDocumentation 
  Logger::get_documentation ()
  {
    return doc_;
  }

  gchar *
  Logger::get_last_line (void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    return context->last_line_;
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
