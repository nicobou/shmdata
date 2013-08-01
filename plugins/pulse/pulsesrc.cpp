/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher-pulse. 
 *
 * Partially from pactl.c Copyright 2004-2006 Lennart Poettering 
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

#include "pulsesrc.h"

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PulseSrc,
				       "Audio Device Source (Pulse)",
				       "audio source", 
				       "Inspecting Devices And Getting Audio From Inputs",
				       "LGPL",
				       "pulsesrc",
				       "Nicolas Bouillot");
    
  bool
  PulseSrc::init ()
  {
    set_name ("coucou");
    pa_context *context = NULL;
    pa_mainloop_api *mainloop_api = NULL;
    pa_proplist *proplist = NULL;
    char *server = NULL;

    proplist = pa_proplist_new();
    
    pa_mainloop *pulse_main_loop = NULL;
    if (!(m = pa_mainloop_new())) {
      g_debug ("pa_mainloop_new() failed.");
      return false;
    }
    
    mainloop_api = pa_mainloop_get_api(pulse_main_loop);
    if (!(context = pa_context_new_with_proplist(mainloop_api, NULL, proplist))) {
      g_debug ("pa_context_new() failed.");
      return false;
    }

    pa_context_set_state_callback(context, context_state_callback, pulse_main_loop);
    if (pa_context_connect(context, server, (pa_context_flags_t)0, NULL) < 0) {
      g_debug ("pa_context_connect() failed: %s", pa_strerror(pa_context_errno(context)));
    return false;
    }
    return true;
  }


  void 
  PulseSrc::context_state_callback(pa_context *c, void *user_data) {

    pulse_audio_main_loop = ()user_data;

    g_debug ("heheheheh 1");
    pa_context_state_t state;
    state = pa_context_get_state(c);
    if (state == PA_CONTEXT_TERMINATED)
      pa_operation_unref(pa_context_get_source_info_list(c, get_source_info_callback, NULL));
    else
      pa_mainloop_iterate(pa_ml, 1, NULL);


    g_debug ("heheheheh 2");
    
  }

  void 
  PulseSrc::get_source_info_callback(pa_context *c, const pa_source_info *i, int is_last, void *userdata) {

    // static const char *state_table[] = {
    //   [1+PA_SOURCE_INVALID_STATE] = "n/a",
    //   [1+PA_SOURCE_RUNNING] = "RUNNING",
    //   [1+PA_SOURCE_IDLE] = "IDLE",
    //   [1+PA_SOURCE_SUSPENDED] = "SUSPENDED"
    // };
    
    char
      s[PA_SAMPLE_SPEC_SNPRINT_MAX],
      cv[PA_CVOLUME_SNPRINT_MAX],
      cvdb[PA_SW_CVOLUME_SNPRINT_DB_MAX],
      v[PA_VOLUME_SNPRINT_MAX],
      vdb[PA_SW_VOLUME_SNPRINT_DB_MAX],
      cm[PA_CHANNEL_MAP_SNPRINT_MAX],
      f[PA_FORMAT_INFO_SNPRINT_MAX];
    char *pl;
    
    if (is_last < 0) {
      g_debug ("Failed to get source information: %s", pa_strerror(pa_context_errno(c)));
      return;
    }
    
    if (is_last) {
      g_debug ("nico should complete action :(");
      //complete_action();
      return;
    }
    
    
    printf("\n");
    
    // printf("%u\t%s\t%s\t%s\t%s\n",
    // 	     i->index,
    // 	     i->name,
    // 	     pa_strnull(i->driver),
    // 	     pa_sample_spec_snprint(s, sizeof(s), &i->sample_spec),
    // 	     state_table[1+i->state]);
    printf("SHORT %u\t%s\t%s\t%s\t ?nico? \n",
	   i->index,
	   i->name,
	   i->driver,//pa_strnull(i->driver),
	   pa_sample_spec_snprint(s, sizeof(s), &i->sample_spec)//,
	   //state_table[1+i->state]
	   );
    
    printf("Source #%u\n"
	   //"\tState: %s\n"
	   "\tName: %s\n"
	   "\tDescription: %s\n"
	   "\tDriver: %s\n"
	   "\tSample Specification: %s\n"
	   "\tChannel Map: %s\n"
	   "\tOwner Module: %u\n"
	   "\tMute: %s\n"
	   "\tVolume: %s%s%s\n"
	   "\t        balance %0.2f\n"
	   "\tBase Volume: %s%s%s\n"
	   "\tMonitor of Sink: %s\n"
	   "\tLatency: %0.0f usec, configured %0.0f usec\n"
	   "\tFlags: %s%s%s%s%s%s\n"
	   "\tProperties:\n\t\t%s\n",
           i->index,
           //state_table[1+i->state],
           i->name,
           i->description,//pa_strnull(i->description),
           i->driver,//pa_strnull(i->driver),
           pa_sample_spec_snprint(s, sizeof(s), &i->sample_spec),
           pa_channel_map_snprint(cm, sizeof(cm), &i->channel_map),
           i->owner_module,
           i->mute,//pa_yes_no(i->mute),
           pa_cvolume_snprint(cv, sizeof(cv), &i->volume),
           i->flags & PA_SOURCE_DECIBEL_VOLUME ? "\n\t        " : "",
           i->flags & PA_SOURCE_DECIBEL_VOLUME ? pa_sw_cvolume_snprint_dB(cvdb, sizeof(cvdb), &i->volume) : "",
           pa_cvolume_get_balance(&i->volume, &i->channel_map),
           pa_volume_snprint(v, sizeof(v), i->base_volume),
           i->flags & PA_SOURCE_DECIBEL_VOLUME ? "\n\t             " : "",
           i->flags & PA_SOURCE_DECIBEL_VOLUME ? pa_sw_volume_snprint_dB(vdb, sizeof(vdb), i->base_volume) : "",
           i->monitor_of_sink_name ? i->monitor_of_sink_name : "n/a",
           (double) i->latency, (double) i->configured_latency,
           i->flags & PA_SOURCE_HARDWARE ? "HARDWARE " : "",
           i->flags & PA_SOURCE_NETWORK ? "NETWORK " : "",
           i->flags & PA_SOURCE_HW_MUTE_CTRL ? "HW_MUTE_CTRL " : "",
           i->flags & PA_SOURCE_HW_VOLUME_CTRL ? "HW_VOLUME_CTRL " : "",
           i->flags & PA_SOURCE_DECIBEL_VOLUME ? "DECIBEL_VOLUME " : "",
           i->flags & PA_SOURCE_LATENCY ? "LATENCY " : "",
           pl = pa_proplist_to_string_sep(i->proplist, "\n\t\t"));

    pa_xfree(pl);

    if (i->ports) {
        pa_source_port_info **p;

        printf("\tPorts:\n");
        for (p = i->ports; *p; p++)
            printf("\t\t%s: %s (priority. %u)\n", (*p)->name, (*p)->description, (*p)->priority);
    }

    if (i->active_port)
        printf("\tActive Port: %s\n",
               i->active_port->name);

    if (i->formats) {
        uint8_t j;

        printf("\tFormats:\n");
        for (j = 0; j < i->n_formats; j++)
            printf("\t\t%s\n", pa_format_info_snprint(f, sizeof(f), i->formats[j]));
    }

  }
}//end of PulseSrc class


