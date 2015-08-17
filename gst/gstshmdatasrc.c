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
 * SECTION:element-shmdatasrc
 *
 * Receive data from a shmdata.
 *
 * <refsect2>
 * <title>Example launch lines</title>
 * |[
 * gst-launch shmdatasrc socket-path=/tmp/blah ! autovideosink
 * ]| Render video from shm buffers.
 * </refsect2>
 */

#include <string.h>
#include <gst/gst.h>
#include "./gstshmdatasrc.h"
#include "./gstshmdatalogger.h"

/* signals */
enum
{
  LAST_SIGNAL
};

/* properties */
enum
{
  PROP_0,
  PROP_SOCKET_PATH,
  PROP_CAPS,
  PROP_BYTES_SINCE_LAST_REQUEST,
  PROP_COPY_BUFFERS
};

/* struct GstShmDataBuffer */
/* { */
/*   char *buf; */
/*   GstShmPipe *pipe; */
/* }; */


GST_DEBUG_CATEGORY_STATIC (shmdatasrc_debug);
#define GST_CAT_DEFAULT shmdatasrc_debug

static GstStaticPadTemplate srctemplate = GST_STATIC_PAD_TEMPLATE ("src",
                                                                   GST_PAD_SRC,
                                                                   GST_PAD_ALWAYS,
                                                                   GST_STATIC_CAPS_ANY);

#define gst_shmdata_src_parent_class parent_class
G_DEFINE_TYPE (GstShmdataSrc, gst_shmdata_src, GST_TYPE_PUSH_SRC);

static void gst_shmdata_src_set_property (GObject *object, guint prop_id,
                                          const GValue *value, GParamSpec *pspec);
static void gst_shmdata_src_get_property (GObject *object, guint prop_id,
                                          GValue *value, GParamSpec *pspec);
static void gst_shmdata_src_finalize (GObject *object);
static gboolean gst_shmdata_src_start (GstBaseSrc *bsrc);
static gboolean gst_shmdata_src_stop (GstBaseSrc *bsrc);
static void gst_shmdata_src_on_data(void *user_data, void *data, size_t size);
static GstFlowReturn gst_shmdata_src_create (GstPushSrc *psrc,
                                             GstBuffer **outbuf);
static void gst_shmdata_src_on_data_rendered(gpointer data);
static gboolean gst_shmdata_src_unlock (GstBaseSrc *bsrc);
static gboolean gst_shmdata_src_unlock_stop (GstBaseSrc *bsrc);
static GstStateChangeReturn gst_shmdata_src_change_state (GstElement *element,
                                                          GstStateChange transition);

// static guint gst_shmdata_src_signals[LAST_SIGNAL] = { 0 };

static void
gst_shmdata_src_class_init (GstShmdataSrcClass * klass)
{
  GObjectClass *gobject_class;
  GstElementClass *gstelement_class;
  GstBaseSrcClass *gstbasesrc_class;
  GstPushSrcClass *gstpush_src_class;

  gobject_class = (GObjectClass *) klass;
  gstelement_class = (GstElementClass *) klass;
  gstbasesrc_class = (GstBaseSrcClass *) klass;
  gstpush_src_class = (GstPushSrcClass *) klass;

  gobject_class->set_property = gst_shmdata_src_set_property;
  gobject_class->get_property = gst_shmdata_src_get_property;
  gobject_class->finalize = gst_shmdata_src_finalize;

  gstelement_class->change_state = gst_shmdata_src_change_state;

  gstbasesrc_class->start = GST_DEBUG_FUNCPTR (gst_shmdata_src_start);
  gstbasesrc_class->stop = GST_DEBUG_FUNCPTR (gst_shmdata_src_stop);
  gstbasesrc_class->unlock = GST_DEBUG_FUNCPTR (gst_shmdata_src_unlock);
  gstbasesrc_class->unlock_stop = GST_DEBUG_FUNCPTR (gst_shmdata_src_unlock_stop);

  gstpush_src_class->create = gst_shmdata_src_create;

  g_object_class_install_property (gobject_class, PROP_SOCKET_PATH,
                                   g_param_spec_string ("socket-path",
                                                        "Path to the control socket",
                                                        "The path to the control socket used to control the shared memory"
                                                        " transport", NULL, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (
      gobject_class,
      PROP_CAPS,
      g_param_spec_string (
          "caps",
          "Data type exposed in the shared memory",
          "The data type (caps) exposed in the shared memory, and proposed for negociation",
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
  
  g_object_class_install_property (
      gobject_class,
      PROP_COPY_BUFFERS,
      g_param_spec_boolean ("copy-buffers", "copy buffers into the pipeline",
                            "False if buffers from shared memory are used into the pipeline",
                            FALSE,
                            G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  
  gst_element_class_add_pad_template (gstelement_class,
                                      gst_static_pad_template_get (&srctemplate));

  gst_element_class_set_static_metadata (gstelement_class,
                                         "Shmdata Source",
                                         "Source",
                                         "Receive data from a shmdata",
                                         "Nicolas Bouillot <nicolas.bouillot@gmail.com>");

  GST_DEBUG_CATEGORY_INIT (shmdatasrc_debug, "shmdatasrc", 0, "Shmdata Source");
}

static void
gst_shmdata_src_init (GstShmdataSrc *self)
{
  self->is_first_read = TRUE;
  self->has_new_caps = FALSE;
  self->caps = NULL;
  self->on_data = FALSE;
  self->copy_buffers = FALSE;
  self->stop_read = FALSE;
  g_mutex_init(&self->on_data_mutex);
  g_cond_init (&self->on_data_cond);
  self->data_rendered = FALSE;
  g_mutex_init (&self->data_rendered_mutex);
  g_cond_init (&self->data_rendered_cond);
  gst_base_src_set_format (GST_BASE_SRC (self), GST_FORMAT_TIME);
  gst_base_src_set_live(GST_BASE_SRC (self), TRUE);
}

static void
gst_shmdata_src_finalize (GObject * object)
{
  GstShmdataSrc *self = GST_SHMDATA_SRC (object);
  g_mutex_clear (&self->on_data_mutex);
  g_cond_clear (&self->on_data_cond);
  g_mutex_clear (&self->data_rendered_mutex);
  g_cond_clear (&self->data_rendered_cond);
  if (NULL != self->caps)
    gst_caps_unref (self->caps);
  G_OBJECT_CLASS (parent_class)->finalize (object);
}


static void
gst_shmdata_src_set_property (GObject * object, guint prop_id,
                              const GValue * value, GParamSpec * pspec)
{
  GstShmdataSrc *self = GST_SHMDATA_SRC (object);
  switch (prop_id) {
    case PROP_SOCKET_PATH:
      GST_OBJECT_LOCK (object);
      if (self->shmfollower) {
        GST_WARNING_OBJECT (object, "Can not modify socket path while the "
                            "element is playing");
      } else {
        g_free (self->socket_path);
        self->socket_path = g_value_dup_string (value);
      }
      GST_OBJECT_UNLOCK (object);
      break;
    case PROP_COPY_BUFFERS:
      self->copy_buffers = g_value_get_boolean (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_shmdata_src_get_property (GObject * object, guint prop_id,
                              GValue * value, GParamSpec * pspec)
{
  GstShmdataSrc *self = GST_SHMDATA_SRC (object);
  gchar *caps_str = NULL;
  switch (prop_id) {
    case PROP_SOCKET_PATH:
      GST_OBJECT_LOCK (object);
      g_value_set_string (value, self->socket_path);
      GST_OBJECT_UNLOCK (object);
      break;
    case PROP_CAPS:
      if (NULL != self->caps) {
        caps_str = gst_caps_to_string(self->caps);
        g_value_set_string (value, caps_str);
        g_free(caps_str);
      } else
        g_value_set_string (value, NULL);
      break;
    case PROP_BYTES_SINCE_LAST_REQUEST:
      g_value_set_uint64 (value, self->bytes_since_last_request);
      self->bytes_since_last_request = 0;
      break;
    case PROP_COPY_BUFFERS:
      g_value_set_boolean (value, self->copy_buffers);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

void gst_shmdata_src_on_server_connect(void *user_data, const char *type_descr) {
  GstShmdataSrc *self = GST_SHMDATA_SRC (user_data);

  if (NULL != self->caps)
    gst_caps_unref(self->caps);
  self->caps = gst_caps_from_string(type_descr);
  if (NULL != self->caps)
    self->has_new_caps = TRUE;
}


static gboolean
gst_shmdata_src_start_reading (GstShmdataSrc * self)
{
  if (!self->socket_path) {
    GST_ELEMENT_ERROR (self, RESOURCE, NOT_FOUND,
                       ("No path specified for socket."), (NULL));
    return FALSE;
  }

  GST_DEBUG_OBJECT (self, "Opening socket %s", self->socket_path);

  GST_OBJECT_LOCK (self);
  self->is_first_read = TRUE;


  self->shmlogger = shmdata_make_logger(&gst_shmdata_on_error, 
                                        &gst_shmdata_on_critical, 
                                        &gst_shmdata_on_warning, 
                                        &gst_shmdata_on_message, 
                                        &gst_shmdata_on_info, 
                                        &gst_shmdata_on_debug, 
                                        self); 
  self->shmfollower = shmdata_make_follower(self->socket_path,
                                            &gst_shmdata_src_on_data,
                                            &gst_shmdata_src_on_server_connect,
                                            NULL, //&on_server_disconnect,
                                            self,
                                            self->shmlogger);
  GST_OBJECT_UNLOCK (self);

  if (!self->shmfollower) {
    GST_ELEMENT_ERROR (self, RESOURCE, OPEN_READ_WRITE,
                       ("Could not initialize shmdata follower %s", self->socket_path), (NULL));
    return FALSE;
  }

  return TRUE;
}

static void
gst_shmdata_src_stop_reading (GstShmdataSrc * self)
{
  GST_DEBUG_OBJECT (self, "Stopping %p", self);
  if(self->shmfollower) {
    shmdata_delete_follower(self->shmfollower);
    self->shmfollower = NULL;
  }
  if(self->shmlogger) {
    shmdata_delete_logger(self->shmlogger);
    self->shmlogger = NULL;
  }
}

static gboolean
gst_shmdata_src_start (GstBaseSrc * bsrc)
{
  return TRUE;
}

static gboolean
gst_shmdata_src_stop (GstBaseSrc * bsrc)
{
  return TRUE;
}

static void gst_shmdata_src_on_data(void *user_data, void *data, size_t size) {
  GstShmdataSrc *self = GST_SHMDATA_SRC (user_data);
  if (self->stop_read)
    return;
  self->current_data = data;
  self->current_size = size;
  self->bytes_since_last_request += size;
  // synchronizing with gst_shmdata_src_create
  g_mutex_lock (&self->on_data_mutex);
  self->on_data = TRUE;
  g_cond_broadcast (&self->on_data_cond);
  g_mutex_unlock (&self->on_data_mutex);
  g_mutex_lock (&self->data_rendered_mutex); 
  while(!self->data_rendered)  // spurious wake
    g_cond_wait (&self->data_rendered_cond, &self->data_rendered_mutex);
  self->data_rendered = FALSE;
  g_mutex_unlock (&self->data_rendered_mutex); 
}

static void gst_shmdata_src_on_data_rendered(gpointer user_data){
  GstShmdataSrc *self = GST_SHMDATA_SRC (user_data); 
  g_mutex_lock (&self->data_rendered_mutex); 
  self->data_rendered = TRUE;
  g_cond_broadcast(&self->data_rendered_cond); 
  g_mutex_unlock (&self->data_rendered_mutex); 
}

static void gst_shmdata_src_make_data_rendered(GstShmdataSrc *self){
  g_mutex_lock (&self->data_rendered_mutex);
  self->data_rendered = TRUE;
  g_cond_broadcast(&self->data_rendered_cond); 
  g_mutex_unlock (&self->data_rendered_mutex); 
}

static GstFlowReturn
gst_shmdata_src_create (GstPushSrc *psrc, GstBuffer **outbuf)
{
  GstShmdataSrc *self = GST_SHMDATA_SRC (psrc);

  if (self->unlocked) {
    return GST_FLOW_FLUSHING;
  }

  g_mutex_lock (&self->on_data_mutex);
  while (!self->on_data && !self->unlocked)
    g_cond_wait_until (&self->on_data_cond,
                       &self->on_data_mutex,
                       g_get_monotonic_time () + 10 * G_TIME_SPAN_MILLISECOND);
  if (self->unlocked) {
    self->on_data = FALSE;
    g_mutex_unlock (&self->on_data_mutex);
    gst_shmdata_src_make_data_rendered(self);
    return GST_FLOW_FLUSHING;
  }
  self->on_data = FALSE;
  g_mutex_unlock (&self->on_data_mutex);

  if (self->is_first_read) {
    gst_shmdata_src_make_data_rendered(self);
    self->is_first_read = FALSE;
  }

  if (self->has_new_caps &&
      (GST_STATE_PAUSED == GST_STATE(self) || GST_STATE_PLAYING == GST_STATE(self))) {
    self->has_new_caps = FALSE;
    g_object_notify(G_OBJECT(self), "caps");
    GstPad *pad = gst_element_get_static_pad (GST_ELEMENT(self),"src");
    if(!gst_pad_set_caps (pad, self->caps)) {
      GST_ELEMENT_ERROR (GST_ELEMENT(self), CORE, NEGOTIATION, (NULL),
                         ("caps fix caps from shmdata type description"));
      return GST_FLOW_ERROR;
    }
    gst_object_unref(pad);
  }
  if(!self->copy_buffers){
    *outbuf = gst_buffer_new_wrapped_full (GST_MEMORY_FLAG_READONLY, 
                                           self->current_data,
                                           self->current_size,
                                           0,
                                           self->current_size,
                                           self,
                                           gst_shmdata_src_on_data_rendered);
  } else {
    GstBuffer *tmp = gst_buffer_new_wrapped_full (GST_MEMORY_FLAG_READONLY, 
                                                  self->current_data,
                                                  self->current_size,
                                                  0,
                                                  self->current_size,
                                                  NULL,
                                                  NULL);
    *outbuf = gst_buffer_copy (tmp); 
    gst_shmdata_src_on_data_rendered(self);
    gst_buffer_unref(tmp);
  }
  
  return GST_FLOW_OK;
}

static GstStateChangeReturn
gst_shmdata_src_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  GstShmdataSrc *self = GST_SHMDATA_SRC (element);

  switch (transition) {
    case GST_STATE_CHANGE_NULL_TO_READY:
      if (!gst_shmdata_src_start_reading (self))
        return GST_STATE_CHANGE_FAILURE;
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (parent_class)->change_state (element, transition);
  if (ret == GST_STATE_CHANGE_FAILURE)
    return ret;

  switch (transition) {
    case GST_STATE_CHANGE_PAUSED_TO_READY:
      self->stop_read = TRUE;
      break;
    case GST_STATE_CHANGE_READY_TO_NULL:
      gst_shmdata_src_make_data_rendered(self);
      gst_shmdata_src_stop_reading (self);
    default:
      break;
  }

  return ret;
}

static gboolean
gst_shmdata_src_unlock (GstBaseSrc * bsrc)
{
  GstShmdataSrc *self = GST_SHMDATA_SRC (bsrc);
  self->unlocked = TRUE;
  return TRUE;
}

static gboolean
gst_shmdata_src_unlock_stop (GstBaseSrc * bsrc)
{
  GstShmdataSrc *self = GST_SHMDATA_SRC (bsrc);
  self->unlocked = FALSE;
  return TRUE;
}
