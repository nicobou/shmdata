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

#ifndef __GST_SHMDATA_SRC_H__
#define __GST_SHMDATA_SRC_H__

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>
#include <gst/base/gstbasesrc.h>
#include "shmdata/clogger.h"
#include "shmdata/cfollower.h"

G_BEGIN_DECLS
#define GST_TYPE_SHMDATA_SRC \
  (gst_shmdata_src_get_type())
#define GST_SHMDATA_SRC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_SHMDATA_SRC,GstShmdataSrc))
#define GST_SHMDATA_SRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_SHMDATA_SRC,GstShmdataSrcClass))
#define GST_IS_SHMDATA_SRC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_SHMDATA_SRC))
#define GST_IS_SHMDATA_SRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_SHMDATA_SRC))
typedef struct _GstShmdataSrc GstShmdataSrc;
typedef struct _GstShmdataSrcClass GstShmdataSrcClass;

struct _GstShmdataSrc
{
  GstPushSrc element;
  gchar *socket_path;
  ShmdataFollower shmfollower;
  ShmdataLogger shmlogger;
  GMutex on_data_mutex;
  GCond on_data_cond;
  gboolean on_data;  // managing spurious wake with GCond
  GMutex data_rendered_mutex;
  GCond data_rendered_cond;
  gboolean data_rendered;  // managing spurious wake with GCond
  void *current_data;
  size_t current_size;
  gboolean is_first_read;
  gboolean has_new_caps;
  GstCaps *caps;
  guint64 bytes_since_last_request;
  gboolean unlocked;
  gboolean copy_buffers;
  gboolean stop_read;
};

struct _GstShmdataSrcClass
{
  GstPushSrcClass parent_class;
};

GType gst_shmdata_src_get_type (void);

G_END_DECLS
#endif /* __GST_SHMDATA_SRC_H__ */
