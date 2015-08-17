/* GStreamer
 * Copyright (C) <2015> Nicolas Bouillot
 * Copyright (C) <2009> Collabora Ltd
 *  @author: Olivier Crete <olivier.crete@collabora.co.uk
 * Copyright (C) <2009> Nokia Inc
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#include "gstshmdatasrc.h"
#include "gstshmdatasink.h"

#ifndef PACKAGE
#define PACKAGE "shmdata"
#endif

static gboolean
plugin_init (GstPlugin * plugin)
{
  return
      gst_element_register (plugin, "shmdatasrc",
                            GST_RANK_NONE, GST_TYPE_SHMDATA_SRC) &&
      gst_element_register (plugin, "shmdatasink",
                            GST_RANK_NONE, GST_TYPE_SHMDATA_SINK);
}

GST_PLUGIN_DEFINE (GST_VERSION_MAJOR,
                   GST_VERSION_MINOR,
                   shm,
                   "shmdata sink source",
                   plugin_init,
                   "1.0",
                   "LGPL",
                   PACKAGE,
                   "https://github.com/nicobou/shmdata")
