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

#include "switcher/gpipe.h" //only for shmdata management
#include "switcher/custom-property-helper.h"
#include "switcher/startable-quiddity.h"
#include <lo/lo.h>
#include <shmdata/any-data-reader.h>
#include <mutex>

#include <chrono>

namespace switcher
{
  class ShmdataToOsc : public GPipe, public StartableQuiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(ShmdataToOsc);
    ShmdataToOsc ();
    ~ShmdataToOsc ();
    ShmdataToOsc (const ShmdataToOsc &) = delete;
    ShmdataToOsc &operator=  (const ShmdataToOsc &) = delete;
    bool init_gpipe () final;

  private:
    CustomPropertyHelper::ptr custom_props_; 
    gint port_;
    std::string host_;
    std::string shmdata_path_;
    GParamSpec *port_spec_;
    GParamSpec *host_spec_;
    GParamSpec *shmdata_path_spec_;
    lo_address address_;
    shmdata_any_reader_t *reader_;
    std::mutex address_mutex_;

    bool start ();
    bool stop ();
    static void set_port (const gint value, void *user_data);
    static gint get_port (void *user_data);
    static void set_host (const gchar *value, void *user_data);
    static const gchar *get_host (void *user_data);
    static void set_shmdata_path (const gchar * value, void *user_data);
    static const gchar *get_shmdata_path (void *user_data);
    static void on_shmreader_data (shmdata_any_reader_t */*reader*/,
				   void *shmbuf,
				   void *data,
				   int /*data_size*/,
				   unsigned long long /*timestamp*/,
				   const char */*type_description*/, 
				   void *user_data);
  };

  SWITCHER_DECLARE_PLUGIN(ShmdataToOsc);

}  // end of namespace

#endif // ifndef
