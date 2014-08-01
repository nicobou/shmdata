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

#ifndef __SWITCHER_SHMDATA_TO_OSC_H__
#define __SWITCHER_SHMDATA_TO_OSC_H__

#include "switcher/quiddity.h"
#include "switcher/segment.h"
#include "switcher/custom-property-helper.h"
#include "switcher/startable-quiddity.h"
#include <lo/lo.h>
#include <shmdata/any-data-reader.h>
#include <mutex>

#include <chrono>

namespace switcher
{
  class ShmdataToOsc : public Quiddity, public Segment, public StartableQuiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(ShmdataToOsc);
    ShmdataToOsc ();
    ~ShmdataToOsc ();
    ShmdataToOsc (const ShmdataToOsc &) = delete;
    ShmdataToOsc &operator=  (const ShmdataToOsc &) = delete;

  private:
    CustomPropertyHelper::ptr custom_props_; 
    gint port_;
    std::string host_;
    GParamSpec *port_spec_;
    GParamSpec *host_spec_;
    lo_address address_;
    std::mutex address_mutex_;

    bool init () final;
    bool start () final;
    bool stop () final;

    //segment handlers
    bool connect (std::string shmdata_path);
    bool can_sink_caps (std::string caps);

    void on_shmreader_data (void *data,
			    int data_size,
			    unsigned long long timestamp,
			    const char *type_description, 
			    void *user_data);
    
    static void set_port (const gint value, void *user_data);
    static gint get_port (void *user_data);
    static void set_host (const gchar *value, void *user_data);
    static const gchar *get_host (void *user_data);
  };

  SWITCHER_DECLARE_PLUGIN(ShmdataToOsc);

}  // end of namespace

#endif // ifndef
