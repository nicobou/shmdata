/* GStreamer
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

#ifndef __GST_SHMDATA_LOGGER_H__
#define __GST_SHMDATA_LOGGER_H__


static void gst_shmdata_on_error(void *self, const char *msg) {
  GST_ERROR_OBJECT(G_OBJECT(self), "%s", msg);
}

static void gst_shmdata_on_critical(void *self, const char *msg) {
  GST_ERROR_OBJECT(G_OBJECT(self), "%s", msg);
}

static void gst_shmdata_on_warning(void *self, const char *msg) {
  GST_WARNING_OBJECT(G_OBJECT(self), "%s", msg);
}

static void gst_shmdata_on_message(void *self, const char *msg) {
  GST_LOG_OBJECT(G_OBJECT(self), "%s", msg);
}

static void gst_shmdata_on_info(void *self, const char *msg) {
  GST_INFO_OBJECT(G_OBJECT(self), "%s", msg);
}

static void gst_shmdata_on_debug(void *self, const char *msg) {
  GST_DEBUG_OBJECT(G_OBJECT(self), "%s", msg);
}

#endif
