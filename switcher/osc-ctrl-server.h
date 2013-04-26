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


#ifndef __SWITCHER_OSC_CTRL_SERVER_H__
#define __SWITCHER_OSC_CTRL_SERVER_H__

#include "quiddity-manager-wrapper.h"
#include "lo/lo.h"
#include "string-map.h"
#include <memory>

namespace switcher
{

  class OscCtrlServer : public QuiddityManagerWrapper
  {
  public:
    typedef std::shared_ptr<OscCtrlServer> ptr;

    ~OscCtrlServer ();
    void set_port (std::string port);
    void start (); 
    void stop ();
    //for invocation into osc handlers:
    std::shared_ptr<QuiddityManager> get_quiddity_manager ();
    //wrappers
    static gboolean set_port_wrapped (gpointer port, gpointer user_data);
    
    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

  private:
    std::string port_;
    StringMap< std::pair <std::string, std::string> > osc_subscribers_; //(host + port)
    lo_server_thread osc_thread_;
    static void prop_cb (std::string subscriber_name, 
			 std::string quiddity_name, 
			 std::string property_name, 
			 std::string value, 
			 void *user_data);
    static int osc_handler(const char *path, const char *types, lo_arg **argv,
			   int argc, void *data, void *user_data);
    static void osc_error(int num, const char *msg, const char *path);
    static gchar *string_from_osc_arg (char types, lo_arg *data);
    static gchar *string_float_to_string_int (const gchar *string_float);
    gchar *make_internal_subscriber_name (const gchar *name);
    gchar *retrieve_subscriber_name (const gchar *internal_name);
  };

}  // end of namespace

#endif // ifndef
