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

#include "switcher/quiddity.h"
#include "switcher/gobject-wrapper.h"
#include "switcher/custom-property-helper.h"
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
    gchar *get_last_line ();
    gboolean get_mute ();
    void set_mute (gboolean mute);
    gboolean get_debug ();
    void set_debug (gboolean debug);
    gboolean get_verbose ();
    void set_verbose (gboolean verbose);


    static void test_set_string_method(const gchar *value, void *user_data);
    static gchar *test_get_string_method(void *user_data);

    static gboolean install_log_handler_wrapped (gpointer log_domain, gpointer user_data);
    static gboolean remove_log_handler_wrapped (gpointer log_domain, gpointer user_data);
    static void log_handler (const gchar *log_domain, 
			     GLogLevelFlags log_level,
			     const gchar *message,
			     gpointer user_data);
    static bool get_last_line_by_gvalue (GValue *value, void *user_data);
    
    static bool set_mute_by_gvalue (const GValue *val, void *user_data);
    static bool get_mute_by_gvalue (GValue *val, void *user_data);
    static bool set_debug_by_gvalue (const GValue *val, void *user_data);
    static bool get_debug_by_gvalue (GValue *val, void *user_data);
    static bool set_verbose_by_gvalue (const GValue *val, void *user_data);
    static bool get_verbose_by_gvalue (GValue *val, void *user_data);

    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

    gchar *pouet_;
    
  private:
    void replace_last_line(gchar *next_line);
    static bool installed_;
    gchar *last_line_;
    bool mute_;
    bool debug_;
    bool verbose_;
    StringMap <guint> handler_ids_;
    //custom property for "last_line" 
    //no need to make it static since logger is a singleton
    CustomPropertyHelper::ptr custom_props_;
    GObjectWrapper::ptr gobject_;
    GParamSpec *last_line_prop_;
    GParamSpec *mute_prop_;
    GParamSpec *debug_prop_;
    GParamSpec *verbose_prop_;
  };
  
}  // end of namespace

#endif // ifndef
