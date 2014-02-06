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

#ifndef __SWITCHER_PULSE_SINK_H__
#define __SWITCHER_PULSE_SINK_H__

#include "switcher/audio-sink.h"
#include <pulse/pulseaudio.h>
#include <pulse/glib-mainloop.h>
#include "switcher/custom-property-helper.h"

namespace switcher
{

  class PulseSink : public AudioSink
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PulseSink);
    PulseSink ();
    ~PulseSink ();

  private:
    GstElement *pulsesink_;
    GstElement *audioconvert_;
    GstElement *pulsesink_bin_;
    bool connected_to_pulse_;
    bool make_elements ();
    static gchar *get_devices_json (void *user_data);
    void make_device_description (pa_context *pulse_context);
    void make_json_description ();

    //custom property:
    CustomPropertyHelper::ptr custom_props_; 
    GParamSpec *devices_description_spec_;//json formated
    gchar *devices_description_;//json formated
    GParamSpec *device_name_spec_;//json formated
    gchar *device_name_;//json formated

    //pulse_audio
    pa_glib_mainloop *pa_glib_mainloop_;
    pa_mainloop_api *pa_mainloop_api_;
    pa_context *pa_context_;
    char *server_;

    bool init_segment ();
    static void pa_context_state_callback(pa_context *c, void *userdata);
    static void get_sink_info_callback(pa_context *c, const pa_sink_info *i, int is_last, void *userdata);
    static void on_pa_event_callback(pa_context *c, 
				     pa_subscription_event_type_t t,
				     uint32_t idx, 
				     void *userdata);
    static gchar *get_device_name (void *user_data);
    static void set_device_name (const gchar *value, 
				 void *user_data);
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
    std::map <std::string, DeviceDescription> devices_; //indexed by pulse_device_name
    void on_shmdata_disconnect ();
    void on_shmdata_connect (std::string /* shmdata_sochet_path */);
  };

  SWITCHER_DECLARE_PLUGIN(PulseSink);

}  // end of namespace

#endif // ifndef
