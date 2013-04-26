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


#ifndef __SWITCHER_LOGGER_H__
#define __SWITCHER_LOGGER_H__

#include "quiddity.h"
#include "custom-property-helper.h"
#include <glib.h>

namespace switcher
{

  class Logger : public Quiddity
  {
  public:
    typedef std::shared_ptr<Logger> ptr;
    bool init ();
    ~Logger ();

    gboolean install_log_handler (const gchar *log_domain);
    gboolean remove_log_handler (const gchar *log_domain);
    static gchar *get_last_line (void *user_data);
    static gboolean get_mute (void *user_data);
    static void set_mute (gboolean mute, void *user_data);
    static gboolean get_debug (void *user_data);
    static void set_debug (gboolean debug, void *user_data);
    static gboolean get_verbose (void *user_data);
    static void set_verbose (gboolean verbose, void *user_data);

    static gboolean install_log_handler_wrapped (gpointer log_domain, gpointer user_data);
    static gboolean remove_log_handler_wrapped (gpointer log_domain, gpointer user_data);
    static void log_handler (const gchar *log_domain, 
			     GLogLevelFlags log_level,
			     const gchar *message,
			     gpointer user_data);

    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

  private:
    void replace_last_line(gchar *next_line);
    static bool installed_;
    bool i_am_the_one_;
    gchar *last_line_;
    bool mute_;
    bool debug_;
    bool verbose_;
    StringMap <guint> handler_ids_;
    //custom properties 
    CustomPropertyHelper::ptr custom_props_;
    GParamSpec *last_line_prop_;
    GParamSpec *mute_prop_;
    GParamSpec *debug_prop_;
    GParamSpec *verbose_prop_;
  };
  
}  // end of namespace

#endif // ifndef
