/* 
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
/**
 * SECTION:element-shmdatasink
 *
 * Send data over shared memory to the matching source.
 *
 * <refsect2>
 * <title>Example launch lines</title>
 * |[
 * gst-launch -v videotestsrc !  shmdatasink socket-path=/tmp/blah shm-size=1000000
 * ]| Send video to shm buffers.
 * </refsect2>
 */

#include <string.h>
#include "./gstshmdatasink.h"
#include "./gstshmdatalogger.h"

// TODO perms, size

/* signals */
enum
{
  SIGNAL_CLIENT_CONNECTED,
  SIGNAL_CLIENT_DISCONNECTED,
  LAST_SIGNAL
};

/* properties */
enum
{
  PROP_0,
  PROP_SOCKET_PATH,
  PROP_CAPS,
  PROP_BYTES_SINCE_LAST_REQUEST
  // PROP_PERMS,
  // PROP_SHM_SIZE,
};

#define DEFAULT_SIZE ( 33554432 )
#define DEFAULT_WAIT_FOR_CONNECTION (TRUE)
/* Default is user read/write, group read */
//#define DEFAULT_PERMS ( S_IRUSR | S_IWUSR | S_IRGRP )


GST_DEBUG_CATEGORY_STATIC (shmdatasink_debug);
#define GST_CAT_DEFAULT shmdatasink_debug

static GstStaticPadTemplate sinktemplate = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS_ANY);

#define gst_shmdata_sink_parent_class parent_class
G_DEFINE_TYPE (GstShmdataSink, gst_shmdata_sink, GST_TYPE_BASE_SINK);

static void gst_shmdata_sink_finalize (GObject * object);
static void gst_shmdata_sink_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_shmdata_sink_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);

static gboolean gst_shmdata_sink_start (GstBaseSink * bsink);
static gboolean gst_shmdata_sink_stop (GstBaseSink * bsink);
static GstFlowReturn gst_shmdata_sink_render (GstBaseSink * bsink, GstBuffer * buf);

static gboolean gst_shmdata_sink_event (GstBaseSink * bsink, GstEvent * event);
static gboolean gst_shmdata_sink_unlock (GstBaseSink * bsink);
static gboolean gst_shmdata_sink_unlock_stop (GstBaseSink * bsink);
static gboolean gst_shmdata_sink_propose_allocation (GstBaseSink * sink,
                                                     GstQuery * query);
static gboolean gst_shmdata_sink_on_caps (GstBaseSink *sink, GstCaps *caps);
static void gst_shmdata_sink_on_client_connected(void *user_data, int id);
static void gst_shmdata_sink_on_client_disconnected(void *user_data, int id);

static guint signals[LAST_SIGNAL] = { 0 };



/********************
 * CUSTOM ALLOCATOR *
 ********************/

#define GST_TYPE_SHMDATA_SINK_ALLOCATOR \
  (gst_shmdata_sink_allocator_get_type())
#define GST_SHMDATA_SINK_ALLOCATOR(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_SHMDATA_SINK_ALLOCATOR, \
      GstShmdataSinkAllocator))
#define GST_SHMDATA_SINK_ALLOCATOR_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_SHMDATA_SINK_ALLOCATOR, \
      GstShmdataSinkAllocatorClass))
#define GST_IS_SHMDATA_SINK_ALLOCATOR(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_SHMDATA_SINK_ALLOCATOR))
#define GST_IS_SHMDATA_SINK_ALLOCATOR_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_SHMDATA_SINK_ALLOCATOR))

struct _GstShmdataSinkAllocator
{
  GstAllocator parent;

  GstShmdataSink *sink;
};

typedef struct _GstShmdataSinkAllocatorClass
{
  GstAllocatorClass parent;
} GstShmdataSinkAllocatorClass;

typedef struct _GstShmdataSinkMemory
{
  GstMemory mem;

  gchar *data;
  GstShmdataSink *sink;
} GstShmdataSinkMemory;

GType gst_shmdata_sink_allocator_get_type (void);

G_DEFINE_TYPE (GstShmdataSinkAllocator, gst_shmdata_sink_allocator, GST_TYPE_ALLOCATOR);

static void
gst_shmdata_sink_allocator_dispose (GObject * object)
{
  GstShmdataSinkAllocator *self = GST_SHMDATA_SINK_ALLOCATOR (object);

  if (self->sink)
    gst_object_unref (self->sink);
  self->sink = NULL;

  G_OBJECT_CLASS (gst_shmdata_sink_allocator_parent_class)->dispose (object);
}

static void
gst_shmdata_sink_allocator_free (GstAllocator * allocator, GstMemory * mem)
{
  GstShmdataSinkMemory *mymem = (GstShmdataSinkMemory *) mem;

  if (mymem->data) {
    gst_object_unref (mymem->sink);
  }
  gst_object_unref (mem->allocator);

  g_slice_free (GstShmdataSinkMemory, mymem);
}

static gpointer
gst_shmdata_sink_allocator_mem_map (GstMemory * mem, gsize maxsize,
    GstMapFlags flags)
{
  GstShmdataSinkMemory *mymem = (GstShmdataSinkMemory *) mem;

  return mymem->data;
}

static void
gst_shmdata_sink_allocator_mem_unmap (GstMemory * mem)
{
}

static GstMemory *
gst_shmdata_sink_allocator_mem_share (GstMemory * mem, gssize offset, gssize size)
{
  GstShmdataSinkMemory *mymem = (GstShmdataSinkMemory *) mem;
  GstShmdataSinkMemory *mysub;
  GstMemory *parent;

  /* find the real parent */
  if ((parent = mem->parent) == NULL)
    parent = mem;

  if (size == -1)
    size = mem->size - offset;

  mysub = g_slice_new0 (GstShmdataSinkMemory);
  /* the shared memory is always readonly */
  gst_memory_init (GST_MEMORY_CAST (mysub), GST_MINI_OBJECT_FLAGS (parent) |
      GST_MINI_OBJECT_FLAG_LOCK_READONLY, gst_object_ref (mem->allocator),
      parent, mem->maxsize, mem->align, mem->offset + offset, size);
  mysub->data = mymem->data;

  return (GstMemory *) mysub;
}

static gboolean
gst_shmdata_sink_allocator_mem_is_span (GstMemory * mem1, GstMemory * mem2,
    gsize * offset)
{
  GstShmdataSinkMemory *mymem1 = (GstShmdataSinkMemory *) mem1;
  GstShmdataSinkMemory *mymem2 = (GstShmdataSinkMemory *) mem2;

  if (offset) {
    GstMemory *parent;

    parent = mem1->parent;

    *offset = mem1->offset - parent->offset;
  }

  /* and memory is contiguous */
  return mymem1->data + mem1->offset + mem1->size ==
      mymem2->data + mem2->offset;
}

static void
gst_shmdata_sink_allocator_init (GstShmdataSinkAllocator * self)
{
  GstAllocator *allocator = GST_ALLOCATOR (self);

  allocator->mem_map = gst_shmdata_sink_allocator_mem_map;
  allocator->mem_unmap = gst_shmdata_sink_allocator_mem_unmap;
  allocator->mem_share = gst_shmdata_sink_allocator_mem_share;
  allocator->mem_is_span = gst_shmdata_sink_allocator_mem_is_span;
}


static GstMemory *
gst_shmdata_sink_allocator_alloc_locked (GstShmdataSinkAllocator * self, gsize size,
    GstAllocationParams * params)
{
  GstMemory *memory = NULL;
  gsize maxsize = size + params->prefix + params->padding;
  gsize align = params->align;

  /* ensure configured alignment */
  align |= gst_memory_alignment;
  /* allocate more to compensate for alignment */
  maxsize += align;

  void *data = shmdata_get_mem(self->sink->access);
  if (data) {
    GstShmdataSinkMemory *mymem;
    gsize aoffset, padding;

    GST_LOG_OBJECT (self,
        "Allocated block with %" G_GSIZE_FORMAT " bytes at %p", size,
        data);

    mymem = g_slice_new0 (GstShmdataSinkMemory);
    memory = GST_MEMORY_CAST (mymem);
    mymem->data = data;
    mymem->sink = gst_object_ref (self->sink);

    /* do alignment */
    if ((aoffset = ((guintptr) mymem->data & align))) {
      aoffset = (align + 1) - aoffset;
      mymem->data += aoffset;
      maxsize -= aoffset;
    }

    if (params->prefix && (params->flags & GST_MEMORY_FLAG_ZERO_PREFIXED))
      memset (mymem->data, 0, params->prefix);

    padding = maxsize - (params->prefix + size);
    if (padding && (params->flags & GST_MEMORY_FLAG_ZERO_PADDED))
      memset (mymem->data + params->prefix + size, 0, padding);

    gst_memory_init (memory, params->flags, g_object_ref (self), NULL,
        maxsize, align, params->prefix, size);
  }

  return memory;
}

static GstMemory *
gst_shmdata_sink_allocator_alloc (GstAllocator * allocator, gsize size,
    GstAllocationParams * params)
{
  GstShmdataSinkAllocator *self = GST_SHMDATA_SINK_ALLOCATOR (allocator);
  GstMemory *memory = NULL;

  GST_OBJECT_LOCK (self->sink);
  memory = gst_shmdata_sink_allocator_alloc_locked (self, size, params);
  GST_OBJECT_UNLOCK (self->sink);

  if (!memory) {
    memory = gst_allocator_alloc (NULL, size, params);
    GST_LOG_OBJECT (self,
        "Not enough shared memory for GstMemory of %" G_GSIZE_FORMAT
        "bytes, allocating using standard allocator", size);
  }

  return memory;
}


static void
gst_shmdata_sink_allocator_class_init (GstShmdataSinkAllocatorClass * klass)
{
  GstAllocatorClass *allocator_class = GST_ALLOCATOR_CLASS (klass);
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  allocator_class->alloc = gst_shmdata_sink_allocator_alloc;
  allocator_class->free = gst_shmdata_sink_allocator_free;
  object_class->dispose = gst_shmdata_sink_allocator_dispose;
}

static GstShmdataSinkAllocator *
gst_shmdata_sink_allocator_new (GstShmdataSink * sink)
{
  GstShmdataSinkAllocator *self = g_object_new (GST_TYPE_SHMDATA_SINK_ALLOCATOR, NULL);

  self->sink = gst_object_ref (sink);

  return self;
}


/***************
 * MAIN OBJECT *
 ***************/

static void
gst_shmdata_sink_init (GstShmdataSink * self)
{
  g_cond_init (&self->cond);
  self->size = DEFAULT_SIZE;
  //  self->perms = DEFAULT_PERMS;
  gst_allocation_params_init (&self->params);
}

static void
gst_shmdata_sink_class_init (GstShmdataSinkClass * klass)
{
  GObjectClass *gobject_class;
  GstElementClass *gstelement_class;
  GstBaseSinkClass *gstbasesink_class;

  gobject_class = (GObjectClass *) klass;
  gstelement_class = (GstElementClass *) klass;
  gstbasesink_class = (GstBaseSinkClass *) klass;

  gobject_class->finalize = gst_shmdata_sink_finalize;
  gobject_class->set_property = gst_shmdata_sink_set_property;
  gobject_class->get_property = gst_shmdata_sink_get_property;

  gstbasesink_class->set_caps = GST_DEBUG_FUNCPTR (gst_shmdata_sink_on_caps);
  gstbasesink_class->start = GST_DEBUG_FUNCPTR (gst_shmdata_sink_start);
  gstbasesink_class->stop = GST_DEBUG_FUNCPTR (gst_shmdata_sink_stop);
  gstbasesink_class->render = GST_DEBUG_FUNCPTR (gst_shmdata_sink_render);
  gstbasesink_class->event = GST_DEBUG_FUNCPTR (gst_shmdata_sink_event);
  gstbasesink_class->unlock = GST_DEBUG_FUNCPTR (gst_shmdata_sink_unlock);
  gstbasesink_class->unlock_stop = GST_DEBUG_FUNCPTR (gst_shmdata_sink_unlock_stop);
  gstbasesink_class->propose_allocation =
      GST_DEBUG_FUNCPTR (gst_shmdata_sink_propose_allocation);

  g_object_class_install_property (
      gobject_class, PROP_SOCKET_PATH,
      g_param_spec_string (
          "socket-path",
          "Path to the control socket",
          "The path to the control socket used to control the shared memory transport",
          NULL,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  
  g_object_class_install_property (
      gobject_class,
      PROP_CAPS,
      g_param_spec_string (
          "caps",
          "Data type exposed in the shared memory",
          "The data type (caps) exposed in the shared memory, as negociated with other elements",
          NULL,
          G_PARAM_READABLE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (
      gobject_class,
      PROP_BYTES_SINCE_LAST_REQUEST,
      g_param_spec_uint64 (
          "bytes",
          "Bytes number since last request",
          "The number of bytes that passed the shmdata since last request",
          0,
          G_MAXUINT64,
          0,
          G_PARAM_READABLE | G_PARAM_STATIC_STRINGS));

  /* g_object_class_install_property (gobject_class, PROP_PERMS, */
  /*     g_param_spec_uint ("perms", */
  /*         "Permissions on the shm area", */
  /*         "Permissions to set on the shm area", */
  /*         0, 07777, DEFAULT_PERMS, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)); */

  /* g_object_class_install_property (gobject_class, PROP_SHM_SIZE, */
  /*     g_param_spec_uint ("shm-size", */
  /*         "Size of the shm area", */
  /*         "Size of the shared memory area", */
  /*         0, G_MAXUINT, DEFAULT_SIZE, */
  /*         G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)); */

  signals[SIGNAL_CLIENT_CONNECTED] = g_signal_new ("client-connected",
      GST_TYPE_SHMDATA_SINK, G_SIGNAL_RUN_LAST, 0, NULL, NULL,
      g_cclosure_marshal_VOID__INT, G_TYPE_NONE, 1, G_TYPE_INT);

  signals[SIGNAL_CLIENT_DISCONNECTED] = g_signal_new ("client-disconnected",
      GST_TYPE_SHMDATA_SINK, G_SIGNAL_RUN_LAST, 0, NULL, NULL,
      g_cclosure_marshal_VOID__INT, G_TYPE_NONE, 1, G_TYPE_INT);

  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get (&sinktemplate));

  gst_element_class_set_static_metadata (gstelement_class,
                                         "Shmdata Sink",
                                         "Sink",
                                         "Share data over a shmdata",
                                         "Nicolas Bouillot <nicolas.bouillot@gmail.com>");

  GST_DEBUG_CATEGORY_INIT (shmdatasink_debug, "shmdatasink", 0, "Shared Memory Sink");
}

static void
gst_shmdata_sink_finalize (GObject * object)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (object);

  g_cond_clear (&self->cond);
  g_free (self->socket_path);
  g_free (self->caps);
  G_OBJECT_CLASS (parent_class)->finalize (object);
}

/*
 * Set the value of a property for the server sink.
 */
static void
gst_shmdata_sink_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (object);
  int ret = 0;

  switch (prop_id) {
    case PROP_SOCKET_PATH:
      GST_OBJECT_LOCK (object);
      g_free (self->socket_path);
      self->socket_path = g_value_dup_string (value);
      GST_OBJECT_UNLOCK (object);
      break;
    /* case PROP_PERMS: */
    /*   GST_OBJECT_LOCK (object); */
    /*   self->perms = g_value_get_uint (value); */
    /*   if (self->pipe) */
    /*     ret = sp_writer_setperms_shm (self->pipe, self->perms); */
    /*   GST_OBJECT_UNLOCK (object); */
    /*   if (ret < 0) */
    /*     GST_WARNING_OBJECT (object, "Could not set permissions on pipe: %s", */
    /*         strerror (ret)); */
    /*   break; */
    /* case PROP_SHM_SIZE: */
    /*   GST_OBJECT_LOCK (object); */
    /*   if (self->shmwriter) { */
    /*     if (sp_writer_resize (self->pipe, g_value_get_uint (value)) < 0) { */
    /*       /\* Swap allocators, so we can know immediately if the memory is */
    /*        * ours *\/ */
    /*       gst_object_unref (self->allocator); */
    /*       self->allocator = gst_shmdata_sink_allocator_new (self); */

    /*       GST_DEBUG_OBJECT (self, "Resized shared memory area from %u to " */
    /*           "%u bytes", self->size, g_value_get_uint (value)); */
    /*     } else { */
    /*       GST_WARNING_OBJECT (self, "Could not resize shared memory area from" */
    /*           "%u to %u bytes", self->size, g_value_get_uint (value)); */
    /*     } */
    /*   } */
    /*   self->size = g_value_get_uint (value); */
    /*   GST_OBJECT_UNLOCK (object); */
    /*   break; */
    default:
      break;
  }
}

static void
gst_shmdata_sink_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (object);

  GST_OBJECT_LOCK (object);

  switch (prop_id) {
    case PROP_SOCKET_PATH:
      g_value_set_string (value, self->socket_path);
      break;
    case PROP_CAPS:
      g_value_set_string (value, self->caps);
      break;
    case PROP_BYTES_SINCE_LAST_REQUEST:
      g_value_set_uint64 (value, self->bytes_since_last_request);
      self->bytes_since_last_request = 0;
      break;
    /* case PROP_PERMS: */
    /*   g_value_set_uint (value, self->perms); */
    /*   break; */
    /* case PROP_SHM_SIZE: */
    /*   g_value_set_uint (value, self->size); */
    /*   break; */
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }

  GST_OBJECT_UNLOCK (object);
}



static gboolean
gst_shmdata_sink_start (GstBaseSink * bsink)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (bsink);
  GError *err = NULL;
  self->stop = FALSE;
  return TRUE;
}


static gboolean
gst_shmdata_sink_stop (GstBaseSink * bsink)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (bsink);
  self->stop = TRUE;
  if (self->allocator)
    gst_object_unref (self->allocator);
  self->allocator = NULL;
  GST_DEBUG_OBJECT (self, "Stopping");

  //     g_signal_emit (self, signals[SIGNAL_CLIENT_DISCONNECTED], 0,
  //        client->pollfd.fd);

  shmdata_delete_writer(self->shmwriter);
  self->shmwriter = NULL;

  return TRUE;
}

static GstFlowReturn
gst_shmdata_sink_render (GstBaseSink * bsink, GstBuffer * buf)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (bsink);
  int rv = 0;
  GstMapInfo map;
  gboolean need_new_memory = FALSE;
  GstFlowReturn ret = GST_FLOW_OK;
  GstMemory *memory = NULL;
  GstBuffer *sendbuf = NULL;

  GST_OBJECT_LOCK (self);
  if (gst_buffer_n_memory (buf) > 1) {
    GST_LOG_OBJECT (self, "Buffer %p has %d GstMemory, we only support a single"
        " one, need to do a memcpy", buf, gst_buffer_n_memory (buf));
    need_new_memory = TRUE;
  } else {
    memory = gst_buffer_peek_memory (buf, 0);

    if (memory->allocator != GST_ALLOCATOR (self->allocator)) {
      need_new_memory = TRUE;
      GST_LOG_OBJECT (self, "Memory in buffer %p was not allocated by "
          "%" GST_PTR_FORMAT ", will memcpy", buf, memory->allocator);
    }
  }

  if (need_new_memory) {
    if (gst_buffer_get_size (buf) > self->size) { 
      gsize area_size = self->size; 
      GST_OBJECT_UNLOCK (self); 
      GST_ELEMENT_ERROR (self, RESOURCE, NO_SPACE_LEFT, 
                         ("Shared memory area is too small"), 
                         ("Shared memory area of size %" G_GSIZE_FORMAT " is smaller than" 
                          "buffer of size %" G_GSIZE_FORMAT, area_size, 
                          gst_buffer_get_size (buf))); 
      return GST_FLOW_ERROR;
    }
    
    while ((memory =
            gst_shmdata_sink_allocator_alloc_locked (self->allocator,
                gst_buffer_get_size (buf), &self->params)) == NULL) {
      g_cond_wait (&self->cond, GST_OBJECT_GET_LOCK (self));
      if (self->unlock)
        goto flushing;
    }

    gst_memory_map (memory, &map, GST_MAP_WRITE);
    gst_buffer_extract (buf, 0, map.data, map.size);
    gst_memory_unmap (memory, &map);

    sendbuf = gst_buffer_new ();
    gst_buffer_copy_into (sendbuf, buf, GST_BUFFER_COPY_METADATA, 0, -1);
    gst_buffer_append_memory (sendbuf, memory);
  } else {
    sendbuf = gst_buffer_ref (buf);
  }

  gst_buffer_map (sendbuf, &map, GST_MAP_READ);
  /* Make the memory readonly as of now as we've sent it to the other side
   * We know it's not mapped for writing anywhere as we just mapped it for
   * reading
   */
  shmdata_notify_clients(self->access, map.size);
  self->bytes_since_last_request += map.size;
  shmdata_release_one_write_access(self->access);

  // wait for client to read and take the write lock
  self->access = shmdata_get_one_write_access(self->shmwriter);
   
  gst_buffer_unmap (sendbuf, &map);

  GST_OBJECT_UNLOCK (self);

  GST_DEBUG_OBJECT (self, "No clients connected, unreffing buffer");
  gst_buffer_unref (sendbuf);

  /* If we allocated our own memory, then unmap it */

  return ret;

flushing:
  GST_OBJECT_UNLOCK (self);
  return GST_FLOW_FLUSHING;
}

static void
free_buffer_locked (GstBuffer * buffer, void *data)
{
  GSList **list = data;

  g_assert (buffer != NULL);

  *list = g_slist_prepend (*list, buffer);
}

static gboolean
gst_shmdata_sink_event (GstBaseSink * bsink, GstEvent * event)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (bsink);

  switch (GST_EVENT_TYPE (event)) {
    case GST_EVENT_EOS:
      break;
    default:
      break;
  }

  return GST_BASE_SINK_CLASS (parent_class)->event (bsink, event);
}


static gboolean
gst_shmdata_sink_unlock (GstBaseSink * bsink)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (bsink);

  GST_OBJECT_LOCK (self);
  self->unlock = TRUE;
  GST_OBJECT_UNLOCK (self);

  g_cond_broadcast (&self->cond);
  return TRUE;
}

static gboolean
gst_shmdata_sink_unlock_stop (GstBaseSink * bsink)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (bsink);

  GST_OBJECT_LOCK (self);
  self->unlock = FALSE;
  GST_OBJECT_UNLOCK (self);

  return TRUE;
}

static gboolean
gst_shmdata_sink_propose_allocation (GstBaseSink * sink, GstQuery * query)
{
  GstShmdataSink *self = GST_SHMDATA_SINK (sink);
  if (self->allocator)
    gst_query_add_allocation_param (query, GST_ALLOCATOR (self->allocator),
                                    NULL);
  return TRUE;
}

static void gst_shmdata_sink_on_client_connected(void *user_data, int id) {
  GstShmdataSink *self = GST_SHMDATA_SINK (user_data); 
  g_signal_emit (self, signals[SIGNAL_CLIENT_CONNECTED], 0,
                 id);
}

static void gst_shmdata_sink_on_client_disconnected(void *user_data, int id) {
  GstShmdataSink *self = GST_SHMDATA_SINK (user_data); 
  g_signal_emit (self, signals[SIGNAL_CLIENT_DISCONNECTED], 0, id);
}

static gboolean gst_shmdata_sink_on_caps (GstBaseSink *sink, GstCaps *caps){
  GstShmdataSink *self = GST_SHMDATA_SINK (sink);
  if (!self->socket_path) { 
    GST_ELEMENT_ERROR (self, RESOURCE, OPEN_READ_WRITE, 
        ("Could not open socket."), (NULL)); 
    return FALSE; 
  } 

  GST_DEBUG_OBJECT (self, "Creating new socket at %s" 
      " with shared memory of %lu bytes", self->socket_path, self->size); 
  g_free(self->caps);
  self->caps = gst_caps_to_string (caps);
  g_object_notify(G_OBJECT(sink), "caps");

  GST_DEBUG_OBJECT(G_OBJECT(sink), "on_caps %s", self->caps);

  self->shmlogger = shmdata_make_logger(&gst_shmdata_on_error, 
                                        &gst_shmdata_on_critical, 
                                        &gst_shmdata_on_warning, 
                                        &gst_shmdata_on_message, 
                                        &gst_shmdata_on_info, 
                                        &gst_shmdata_on_debug, 
                                        self); 
  self->shmwriter = shmdata_make_writer(self->socket_path, 
                                        self->size, 
                                        NULL == self->caps ? "unknown" : self->caps, 
                                        &gst_shmdata_sink_on_client_connected,
                                        &gst_shmdata_sink_on_client_disconnected,
                                        self,
                                        self->shmlogger); 
  if (!self->shmwriter) { 
    GST_ELEMENT_ERROR (self, RESOURCE, OPEN_READ_WRITE, 
        ("Could not open socket."), (NULL)); 
    return FALSE; 
  } 

  self->access = shmdata_get_one_write_access(self->shmwriter);

  GST_DEBUG ("Created socket at %s", self->socket_path); 
  self->allocator = gst_shmdata_sink_allocator_new (self); 

  return TRUE;
}
