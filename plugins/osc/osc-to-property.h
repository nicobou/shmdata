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

#ifndef __SWITCHER_OSC_CTRL_SERVER_H__
#define __SWITCHER_OSC_CTRL_SERVER_H__

#include "switcher/quiddity.h"
#include "switcher/custom-property-helper.h"
#include "lo/lo.h"

namespace switcher
{

  class OscToProperty : public Quiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(OscToProperty);
    OscToProperty ();
    ~OscToProperty ();
    OscToProperty (const OscToProperty &) = delete;
    OscToProperty &operator=  (const OscToProperty &) = delete;
    bool init ();

    void set_port (std::string port);

  private:
    CustomPropertyHelper::ptr custom_props_; 
    std::string port_;
    lo_server_thread osc_thread_;

    void start ();
    void stop ();
    static gboolean set_port_wrapped (gpointer port, gpointer user_data);
    static int osc_handler(const char *path, const char *types, lo_arg **argv,
			   int argc, void *data, void *user_data);
    static void osc_error(int num, const char *msg, const char *path);
    static gchar *string_from_osc_arg (char types, lo_arg *data);
    static gchar *string_float_to_string_int (const gchar *string_float);
  };

  SWITCHER_DECLARE_PLUGIN(OscToProperty);

}  // end of namespace

#endif // ifndef
