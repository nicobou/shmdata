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

#ifndef __GST_SHMDATA_SINK_H__
#define __GST_SHMDATA_SINK_H__

#include <gst/gst.h>
#include <gst/base/gstbasesink.h>
#include "shmdata/clogger.h"
#include "shmdata/cwriter.h"

G_BEGIN_DECLS
#define GST_TYPE_SHMDATA_SINK \
  (gst_shmdata_sink_get_type())
#define GST_SHMDATA_SINK(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_SHMDATA_SINK,GstShmdataSink))
#define GST_SHMDATA_SINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_SHMDATA_SINK,GstShmdataSinkClass))
#define GST_IS_SHMDATA_SINK(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_SHMDATA_SINK))
#define GST_IS_SHMDATA_SINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_SHMDATA_SINK))
typedef struct _GstShmdataSink GstShmdataSink;
typedef struct _GstShmdataSinkClass GstShmdataSinkClass;
typedef struct _GstShmdataSinkAllocator GstShmdataSinkAllocator;

struct _GstShmdataSink
{
  GstBaseSink element;
  gchar *socket_path;
  gchar *caps;
  guint64 bytes_since_last_request;
  ShmdataWriter shmwriter;
  ShmdataLogger shmlogger;
  ShmdataWriterAccess access;
  guint perms;
  size_t size;
  gboolean stop;
  gboolean unlock;
  GCond cond;
  GstShmdataSinkAllocator *allocator;
  GstAllocationParams params;
};

struct _GstShmdataSinkClass
{
  GstBaseSinkClass parent_class;
};

GType gst_shmdata_sink_get_type (void);

G_END_DECLS
#endif /* __GST_SHMDATA_SINK_H__ */
