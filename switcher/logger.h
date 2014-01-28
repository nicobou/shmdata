/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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


#ifndef __SWITCHER_LOGGER_H__
#define __SWITCHER_LOGGER_H__

#include <glib.h>
#include <unordered_map>
#include "quiddity.h"
#include "custom-property-helper.h"

namespace switcher
{

  class Logger : public Quiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(Logger);
    Logger ();
    ~Logger ();
    Logger (const Logger &) = delete;
    Logger &operator= (const Logger &) = delete;
    bool init ();

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

  private:
    void replace_last_line(gchar *next_line);
    static bool installed_;
    bool i_am_the_one_;
    gchar *last_line_;
    bool mute_;
    bool debug_;
    bool verbose_;
    std::unordered_map <std::string, guint> handler_ids_;
    //custom properties 
    CustomPropertyHelper::ptr custom_props_;
    GParamSpec *last_line_prop_;
    GParamSpec *mute_prop_;
    GParamSpec *debug_prop_;
    GParamSpec *verbose_prop_;
  };
  
}  // end of namespace

#endif // ifndef
