/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-pulse.
 *
 * switcher-pulse is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_PULSE_SRC_H__
#define __SWITCHER_PULSE_SRC_H__

#include <pulse/pulseaudio.h>
#include <pulse/glib-mainloop.h>
#include "switcher/audio-source.h"
#include "switcher/custom-property-helper.h"
#include "switcher/startable-quiddity.h"

namespace switcher
{

  class PulseSrc : public AudioSource, public StartableQuiddity 
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PulseSrc);
    ~PulseSrc ();
    
    bool start ();
    bool stop ();

  private:
    GstElement *pulsesrc_;
    GstElement *capsfilter_;
    GstElement *pulsesrc_bin_;
    bool make_elements ();
    static gchar *get_capture_devices_json (void *user_data);
    static gboolean async_get_pulse_devices (void *user_data);
    GCond devices_cond_;
    GMutex devices_mutex_;

    bool capture_device ();
    void update_capture_device ();

    void make_device_description (pa_context *pulse_context);
    void make_json_description ();

    //custom property:
    CustomPropertyHelper::ptr custom_props_; 
    GParamSpec *capture_devices_description_spec_;//json formated
    gchar *capture_devices_description_;//json formated

    //device enum and select
    GParamSpec *devices_enum_spec_;
    GEnumValue devices_enum_ [128];
    gint device_;
    static void set_device (const gint value, void *user_data);
    static gint get_device (void *user_data);

    //pulse_audio
    pa_glib_mainloop *pa_glib_mainloop_;
    pa_mainloop_api *pa_mainloop_api_;
    pa_context *pa_context_;
    char *server_;

    static void pa_context_state_callback(pa_context *c, void *userdata);
    static void get_source_info_callback(pa_context *c, const pa_source_info *i, int is_last, void *userdata);
    static void on_pa_event_callback(pa_context *c, 
				     pa_subscription_event_type_t t,
				     uint32_t idx, 
				     void *userdata);

    typedef struct {
      std::string name_;
      std::string description_;
      std::string state_;
      std::string sample_format_;
      std::string sample_rate_;
      std::string channels_;
      std::vector <std::pair <std::string/*port*/,std::string /*description*/> > ports_;
      std::string active_port_;
    } DeviceDescription;

    std::vector <DeviceDescription> capture_devices_; 


  };

  SWITCHER_DECLARE_PLUGIN(PulseSrc);

}  // end of namespace

#endif // ifndef
