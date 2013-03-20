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
  
  QuiddityDocumentation Logger::doc_  ("Logger", "logger",
				       "provide switcher internal logs");
  bool
  Logger::init()
  {
    if (installed_)
      return false;
    
    set_name ("logger");

    g_log_set_default_handler (log_handler, this);

    installed_ = true;
    quiet_ = false;
    debug_ = true;
    verbose_ = true;
    
    return true;
  }

  Logger::~Logger()
  {
    
  }
  
  void
  Logger::log_handler (const gchar *log_domain, 
		       GLogLevelFlags log_level,
		       const gchar *message,
		       gpointer user_data)
  {
    Logger *context = static_cast<Logger *>(user_data);

    if (context->quiet_)
      return;
    
    switch (log_level) {
    case G_LOG_LEVEL_ERROR:
      g_print ("COUCOU %s-error: %s\n",log_domain, message);
      break;
    case G_LOG_LEVEL_CRITICAL:
      g_print ("COUCOU %s-critical: %s\n",log_domain, message);
      break;
    case G_LOG_LEVEL_WARNING:
      g_print ("COUCOU %s-warning: %s\n",log_domain, message);
      break;
    case G_LOG_LEVEL_MESSAGE:
      if (context->debug_ || context->verbose_)
	g_print ("COUCOU %s-message: %s\n",log_domain, message);
      break;
    case G_LOG_LEVEL_INFO:
      if (context->debug_ || context->verbose_)
	g_print ("COUCOU %s-info: %s\n",log_domain, message);
      break;
    case G_LOG_LEVEL_DEBUG:
      if (context->debug_)
	g_print ("COUCOU %s-debug: %s\n",log_domain, message);
      break;
    default:
      g_print ("COUCOU %s-unknown-level: %s\n",log_domain,message);
      break;
    }
}

  QuiddityDocumentation 
  Logger::get_documentation ()
  {
    return doc_;
  }
  

}
