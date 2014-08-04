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
#include "switcher/segment.h"
#include "switcher/custom-property-helper.h"
#include "switcher/startable-quiddity.h"
#include "lo/lo.h"

#include <chrono>

namespace switcher
{
  class OscToShmdata : public Quiddity, public Segment, public StartableQuiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(OscToShmdata);
    OscToShmdata ();
    ~OscToShmdata ();
    OscToShmdata (const OscToShmdata &) = delete;
    OscToShmdata &operator=  (const OscToShmdata &) = delete;

  private:
    CustomPropertyHelper::ptr custom_props_; 
    gint port_;
    lo_server_thread osc_thread_;
    GParamSpec *port_spec_;
    std::chrono::time_point<std::chrono::system_clock> start_;
    ShmdataAnyWriter::ptr shm_any_;

    bool init () final;
    bool start () final;
    bool stop () final;

    static int osc_handler(const char *path, const char *types, lo_arg **argv,
			   int argc, void *data, void *user_data);
    static void osc_error(int num, const char *msg, const char *path);
    static void set_port (const gint value, void *user_data);
    static gint get_port (void *user_data);
  };

  SWITCHER_DECLARE_PLUGIN(OscToShmdata);

}  // end of namespace

#endif // ifndef
