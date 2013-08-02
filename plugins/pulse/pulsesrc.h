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

#include "switcher/audio-source.h"
#include <pulse/pulseaudio.h>
#include <pulse/glib-mainloop.h>
#include "switcher/custom-property-helper.h"

namespace switcher
{

  class PulseSrc : public AudioSource
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PulseSrc);
    ~PulseSrc ();

  private:
    GstElement *pulsesrc_;
    GstElement *capsfilter_;
    GstElement *pulsesrc_bin_;
    bool make_elements ();
    void clean_elements ();
    static gchar *get_capture_devices_json (void *user_data);
    static gboolean capture_wrapped (gpointer device_file_path, 
			      gpointer user_data);
    static gboolean capture_device_wrapped (gpointer pulse_device_name,
					    gpointer user_data);
    bool capture_device (const char *pulse_device_name);

    //custom property:
    CustomPropertyHelper::ptr custom_props_; 
    GParamSpec *capture_devices_description_spec_;//json formated
    gchar *capture_devices_description_;//json formated

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

    std::map <std::string, std::string> capture_devices_; //indexed by pulse_device_name


  };

  SWITCHER_DECLARE_PLUGIN(PulseSrc);

}  // end of namespace

#endif // ifndef
