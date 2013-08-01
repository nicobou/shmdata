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
#include <pulse/pulseaudio.h>

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
    pa_operation_unref(pa_context_get_source_info_list(c, get_source_info_callback, NULL));
    return true;
  }
  
  
  static void context_state_callback(pa_context *c, void *userdata) {
    
  }

  static void 
  PulseSrc::get_source_info_callback(pa_context *c, const pa_source_info *i, int is_last, void *userdata) {

    static const char *state_table[] = {
      [1+PA_SOURCE_INVALID_STATE] = "n/a",
      [1+PA_SOURCE_RUNNING] = "RUNNING",
      [1+PA_SOURCE_IDLE] = "IDLE",
      [1+PA_SOURCE_SUSPENDED] = "SUSPENDED"
    };
    
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
      pa_log(_("Failed to get source information: %s"), pa_strerror(pa_context_errno(c)));
      quit(1);
      return;
    }
    
    if (is_last) {
      complete_action();
      return;
    }
    
    pa_assert(i);
    
    if (nl && !short_list_format)
      printf("\n");
    nl = TRUE;
    
    if (short_list_format) {
      printf("%u\t%s\t%s\t%s\t%s\n",
	     i->index,
	     i->name,
	     pa_strnull(i->driver),
	     pa_sample_spec_snprint(s, sizeof(s), &i->sample_spec),
	     state_table[1+i->state]);
      return;
    }
    
    printf(_("Source #%u\n"
             "\tState: %s\n"
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
             "\tProperties:\n\t\t%s\n"),
           i->index,
           state_table[1+i->state],
           i->name,
           pa_strnull(i->description),
           pa_strnull(i->driver),
           pa_sample_spec_snprint(s, sizeof(s), &i->sample_spec),
           pa_channel_map_snprint(cm, sizeof(cm), &i->channel_map),
           i->owner_module,
           pa_yes_no(i->mute),
           pa_cvolume_snprint(cv, sizeof(cv), &i->volume),
           i->flags & PA_SOURCE_DECIBEL_VOLUME ? "\n\t        " : "",
           i->flags & PA_SOURCE_DECIBEL_VOLUME ? pa_sw_cvolume_snprint_dB(cvdb, sizeof(cvdb), &i->volume) : "",
           pa_cvolume_get_balance(&i->volume, &i->channel_map),
           pa_volume_snprint(v, sizeof(v), i->base_volume),
           i->flags & PA_SOURCE_DECIBEL_VOLUME ? "\n\t             " : "",
           i->flags & PA_SOURCE_DECIBEL_VOLUME ? pa_sw_volume_snprint_dB(vdb, sizeof(vdb), i->base_volume) : "",
           i->monitor_of_sink_name ? i->monitor_of_sink_name : _("n/a"),
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

        printf(_("\tPorts:\n"));
        for (p = i->ports; *p; p++)
            printf("\t\t%s: %s (priority. %u)\n", (*p)->name, (*p)->description, (*p)->priority);
    }

    if (i->active_port)
        printf(_("\tActive Port: %s\n"),
               i->active_port->name);

    if (i->formats) {
        uint8_t j;

        printf(_("\tFormats:\n"));
        for (j = 0; j < i->n_formats; j++)
            printf("\t\t%s\n", pa_format_info_snprint(f, sizeof(f), i->formats[j]));
    }

  }
}//end of PulseSrc class


