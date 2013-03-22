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

#include "switcher/logger.h"

namespace switcher
{

  bool Logger::installed_ = false;
  
  QuiddityDocumentation Logger::doc_  ("log", "logger",
				       "manage switcher logs and other glib log domains.");
  bool
  Logger::init()
  {
    if (installed_)
      {
	g_warning ("Only one logger instance is possible, cannot create");
	return false;
      }
    else
      installed_ = true;
    
    set_name ("logger");
    
    custom_props_.reset (new CustomPropertyHelper ());
    GParamSpec *test_string = custom_props_->make_string_property ("pouet", 
								  "ah ba pouet",
								  "coucou pouet",
								   (GParamFlags)G_PARAM_READWRITE,
								  test_set_string_method,
								  test_get_string_method,
								  this);

    register_property_by_pspec (custom_props_->get_gobject (), 
				test_string, 
				"pouet");

    mute_ = false;
    debug_ = true;
    verbose_ = true;
    last_line_ = g_strdup ("");
    gobject_.reset (new GObjectWrapper ());
    gobject_->set_default_user_data (this);
    last_line_prop_ = 
      GObjectWrapper::make_string_property ("last-line", 
					    "last log line",
					    "",
					    (GParamFlags) G_PARAM_READABLE,
					    NULL,
					    Logger::get_last_line_by_gvalue);
    register_property_by_pspec (gobject_->get_gobject (), 
				last_line_prop_, 
				"last-line");

    mute_prop_ = 
      GObjectWrapper::make_boolean_property ("mute", 
					     "mute log messages",
					     FALSE,
					     (GParamFlags) G_PARAM_READWRITE,
					     Logger::set_mute_by_gvalue,
					     Logger::get_mute_by_gvalue);
    register_property_by_pspec (gobject_->get_gobject (), 
				mute_prop_, 
				"mute");

    debug_prop_ = 
      GObjectWrapper::make_boolean_property ("debug", 
					     "enable debug messages",
					     TRUE,
					     (GParamFlags) G_PARAM_READWRITE,
					     Logger::set_debug_by_gvalue,
					     Logger::get_debug_by_gvalue);
    register_property_by_pspec (gobject_->get_gobject (), 
				debug_prop_, 
				"debug");

    verbose_prop_ = 
      GObjectWrapper::make_boolean_property ("verbose", 
					     "enable log messages",
					     TRUE,
					     (GParamFlags) G_PARAM_READWRITE,
					     Logger::set_verbose_by_gvalue,
					     Logger::get_verbose_by_gvalue);
    register_property_by_pspec (gobject_->get_gobject (), 
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
    std::map<std::string, guint> handlers = handler_ids_.get_map ();
    std::map<std::string, guint>::iterator it;
    for (it = handlers.begin (); it != handlers.end (); it++)
	g_log_remove_handler (it->first.c_str (), it->second);

    g_free (last_line_);
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
       break;
     case G_LOG_LEVEL_INFO:
       if (context->debug_ || context->verbose_)
     	  context->replace_last_line(g_strdup_printf ("%s-info: %s",log_domain, message));
       break;
     case G_LOG_LEVEL_DEBUG:
       if (context->debug_)
	 context->replace_last_line(g_strdup_printf ("%s-debug: %s",log_domain, message));
       break;
     default:
       context->replace_last_line(g_strdup_printf ("%s-unknown-level: %s",log_domain,message));
       break;
     }

     GObjectWrapper::notify_property_changed (context->gobject_->get_gobject (),
					      context->last_line_prop_);
}

  gchar *
  Logger::get_last_line ()
  {
    return last_line_;
  }

  bool
  Logger::get_last_line_by_gvalue (GValue *value,
				   void *user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    g_value_set_string (value, context->get_last_line ());
    return TRUE;
  }


  QuiddityDocumentation 
  Logger::get_documentation ()
  {
    return doc_;
  }
  
  gboolean 
  Logger::get_mute ()
  {
    return mute_;
  }

  void 
  Logger::set_mute (gboolean mute)
  {
    mute_ = mute;
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (),
					     mute_prop_);
  }
  
  bool
  Logger::set_mute_by_gvalue (const GValue *val, void *user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    context->set_mute (g_value_get_boolean (val));
    return true;
  }
    
  bool Logger::get_mute_by_gvalue (GValue *val, void *user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    g_value_set_boolean (val, context->get_mute ());
    return true;
  }
  
  gboolean 
  Logger::get_debug ()
  {
    return debug_;
  }

  void 
  Logger::set_debug (gboolean debug)
  {
    debug_ = debug;
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (),
					     debug_prop_);
  }
  
  bool 
  Logger::set_debug_by_gvalue (const GValue *val, void *user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    context->set_debug (g_value_get_boolean (val));
    return true;
  }
  
  bool 
  Logger::get_debug_by_gvalue (GValue *val, void *user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    g_value_set_boolean (val, context->get_debug ());
    return true;
  }
  
  
  gboolean 
  Logger::get_verbose ()
  {
    return verbose_;
  }

  void 
  Logger::set_verbose (gboolean verbose)
  {
    verbose_ = verbose;
    GObjectWrapper::notify_property_changed (gobject_->get_gobject (),
					     verbose_prop_);
  }

  
  bool 
  Logger::set_verbose_by_gvalue (const GValue *val, void *user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    context->set_verbose (g_value_get_boolean (val));
    return true;
  }
  
  bool 
  Logger::get_verbose_by_gvalue (GValue *val, void *user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);
    g_value_set_boolean (val, context->get_verbose ());
    return true;
  }

  void 
  Logger::test_set_string_method(const gchar *value, void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    if (context->pouet_ != NULL)
      g_free (context->pouet_);
    context->pouet_ = g_strdup (value);
  }
  
  gchar *
  Logger::test_get_string_method(void *user_data)
  {
    Logger *context = static_cast<Logger *> (user_data);
    return context->pouet_;
  }


}
