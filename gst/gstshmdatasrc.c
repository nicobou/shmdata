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
  PROP_IS_LIVE
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

static void gst_shmdata_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_shmdata_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);
static void gst_shmdata_src_finalize (GObject * object);
static gboolean gst_shmdata_src_start (GstBaseSrc * bsrc);
static gboolean gst_shmdata_src_stop (GstBaseSrc * bsrc);
static void gst_shmdata_src_on_data(void *user_data, void *data, size_t size);
static GstFlowReturn gst_shmdata_src_create (GstPushSrc * psrc,
    GstBuffer ** outbuf);
static void gst_shmdata_src_on_data_rendered(gpointer data);
static gboolean gst_shmdata_src_unlock (GstBaseSrc * bsrc);
static gboolean gst_shmdata_src_unlock_stop (GstBaseSrc * bsrc);
static GstStateChangeReturn gst_shmdata_src_change_state (GstElement * element,
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

  g_object_class_install_property (gobject_class, PROP_IS_LIVE,
      g_param_spec_boolean ("is-live", "Is this a live source",
          "True if the element cannot produce data in PAUSED", FALSE,
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
gst_shmdata_src_init (GstShmdataSrc * self)
{
  self->is_first_read = TRUE;
  g_mutex_init(&self->on_data_mutex);
  g_cond_init (&self->on_data_cond);
  g_mutex_init (&self->data_rendered_mutex);
  g_cond_init (&self->data_rendered_cond);
}

static void
gst_shmdata_src_finalize (GObject * object)
{
  GstShmdataSrc *self = GST_SHMDATA_SRC (object);
  g_mutex_clear (&self->on_data_mutex);
  g_cond_clear (&self->on_data_cond);
  g_mutex_clear (&self->data_rendered_mutex);
  g_cond_clear (&self->data_rendered_cond);
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
    case PROP_IS_LIVE:
      gst_base_src_set_live (GST_BASE_SRC (object),
          g_value_get_boolean (value));
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

  switch (prop_id) {
    case PROP_SOCKET_PATH:
      GST_OBJECT_LOCK (object);
      g_value_set_string (value, self->socket_path);
      GST_OBJECT_UNLOCK (object);
      break;
    case PROP_IS_LIVE:
      g_value_set_boolean (value, gst_base_src_is_live (GST_BASE_SRC (object)));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

void gst_shmdata_src_on_server_connect(void *user_data, const char *type_descr) {
  GstShmdataSrc *self = GST_SHMDATA_SRC (user_data);
  GstPad *pad = gst_element_get_static_pad (GST_ELEMENT(self),"src");
  gst_pad_use_fixed_caps (pad);
  GstCaps *caps = gst_caps_from_string(type_descr);
  if (!gst_pad_set_caps (pad, caps)) {
    GST_WARNING_OBJECT (self, "cannot set caps from shmdata type description: %s",
                        type_descr);
  }
  // FIXME unref
  
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
  if (gst_base_src_is_live (bsrc))
    return TRUE;
  else
    return gst_shmdata_src_start_reading (GST_SHMDATA_SRC (bsrc));
}

static gboolean
gst_shmdata_src_stop (GstBaseSrc * bsrc)
{
  if (!gst_base_src_is_live (bsrc))
    gst_shmdata_src_stop_reading (GST_SHMDATA_SRC (bsrc));

  return TRUE;
}


/* static void */
/* free_buffer (gpointer data) */
/* { */
/*   /\* struct GstShmDataBuffer *gsb = data; *\/ */
/*   /\* g_return_if_fail (gsb->shmfollower != NULL); *\/ */

/*   /\* GST_OBJECT_LOCK (gsb->pipe->src); *\/ */
/*   /\* // TODO release read lock (?) *\/ */
/*   /\* //sp_client_recv_finish (gsb->pipe->pipe, gsb->buf); *\/ */
/*   /\* GST_OBJECT_UNLOCK (gsb->pipe->src); *\/ */
/* } */

static void gst_shmdata_src_on_data(void *user_data, void *data, size_t size) {
  //g_printf("%s %d\n", __FUNCTION__, __LINE__);
  GstShmdataSrc *self = GST_SHMDATA_SRC (user_data);
  self->current_data = data;
  self->current_size = size;


  // synchronizing with gst_shmdata_src_create
  //g_printf("%s %d\n", __FUNCTION__, __LINE__);

  g_mutex_lock (&self->data_rendered_mutex); 

  
  g_mutex_lock (&self->on_data_mutex);
  g_cond_broadcast (&self->on_data_cond);
  g_mutex_unlock (&self->on_data_mutex);


  //g_printf("%s %d\n", __FUNCTION__, __LINE__); 
  //g_cond_wait (&self->data_rendered_cond, &self->data_rendered_mutex); 
  g_mutex_unlock (&self->data_rendered_mutex); 
  //g_printf("%s %d\n", __FUNCTION__, __LINE__);
}

static void gst_shmdata_src_on_data_rendered(gpointer user_data){
  GstShmdataSrc *self = GST_SHMDATA_SRC (user_data); 
  //g_printf("%s %d\n", __FUNCTION__, __LINE__); 
  g_mutex_lock (&self->data_rendered_mutex); 

  g_cond_broadcast(&self->data_rendered_cond); 
  g_mutex_unlock (&self->data_rendered_mutex); 
  /* //g_printf("%s %d\n", __FUNCTION__, __LINE__); */
}

static GstFlowReturn
gst_shmdata_src_create (GstPushSrc * psrc, GstBuffer ** outbuf)
{
  GstShmdataSrc *self = GST_SHMDATA_SRC (psrc);

  if (self->unlocked) {
    return GST_FLOW_FLUSHING;
  }
  //g_printf("%s %d\n", __FUNCTION__, __LINE__);
  g_mutex_lock (&self->on_data_mutex);
  g_cond_wait (&self->on_data_cond, &self->on_data_mutex);
  //g_printf("%s %d\n", __FUNCTION__, __LINE__);
  g_mutex_unlock (&self->on_data_mutex);

  *outbuf = gst_buffer_new_wrapped_full (GST_MEMORY_FLAG_READONLY, 
                                         self->current_data,
                                         self->current_size,
                                         0,
                                         self->current_size,
                                         self,
                                         gst_shmdata_src_on_data_rendered); 

  if (self->is_first_read) {
    g_mutex_lock (&self->data_rendered_mutex); 
    g_cond_broadcast(&self->data_rendered_cond); 
    g_mutex_unlock (&self->data_rendered_mutex); 
    self->is_first_read = FALSE;
  }
  
  //g_printf("%s %d\n", __FUNCTION__, __LINE__);

  //gchar *buf = NULL;
  //int rv = 0;
  //struct GstShmDataBuffer *gsb;

  /* do { */
  /*   if (gst_poll_wait (self->poll, GST_CLOCK_TIME_NONE) < 0) { */
  /*     if (errno == EBUSY) */
  /*       return GST_FLOW_FLUSHING; */
  /*     GST_ELEMENT_ERROR (self, RESOURCE, READ, ("Failed to read from shmdatasrc"), */
  /*         ("Poll failed on fd: %s", strerror (errno))); */
  /*     return GST_FLOW_ERROR; */
  /*   } */

  /*   if (self->unlocked) */
  /*     return GST_FLOW_FLUSHING; */

  /*   if (gst_poll_fd_has_closed (self->poll, &self->pollfd)) { */
  /*     GST_ELEMENT_ERROR (self, RESOURCE, READ, ("Failed to read from shmdatasrc"), */
  /*         ("Control socket has closed")); */
  /*     return GST_FLOW_ERROR; */
  /*   } */

  /*   if (gst_poll_fd_has_error (self->poll, &self->pollfd)) { */
  /*     GST_ELEMENT_ERROR (self, RESOURCE, READ, ("Failed to read from shmdatasrc"), */
  /*         ("Control socket has error")); */
  /*     return GST_FLOW_ERROR; */
  /*   } */

  /*   if (gst_poll_fd_can_read (self->poll, &self->pollfd)) { */
  /*     buf = NULL; */
  /*     GST_LOG_OBJECT (self, "Reading from pipe"); */
  /*     GST_OBJECT_LOCK (self); */
  /*     rv = sp_client_recv (self->pipe->pipe, &buf); */
  /*     GST_OBJECT_UNLOCK (self); */
  /*     if (rv < 0) { */
  /*       GST_ELEMENT_ERROR (self, RESOURCE, READ, ("Failed to read from shmdatasrc"), */
  /*           ("Error reading control data: %d", rv)); */
  /*       return GST_FLOW_ERROR; */
  /*     } */
  /*   } */
  /* } while (buf == NULL); */

  /* GST_LOG_OBJECT (self, "Got buffer %p of size %d", buf, rv); */

  /* gsb = g_slice_new0 (struct GstShmDataBuffer); */
  /* gsb->buf = buf; */
  /* gsb->pipe = self->pipe; */
  /* gst_shmdata_pipe_inc (self->pipe); */

  /* *outbuf = gst_buffer_new_wrapped_full (GST_MEMORY_FLAG_READONLY, */
  /*     buf, rv, 0, rv, gsb, free_buffer); */

  return GST_FLOW_OK;
}

static GstStateChangeReturn
gst_shmdata_src_change_state (GstElement * element, GstStateChange transition)
{
  GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
  GstShmdataSrc *self = GST_SHMDATA_SRC (element);

  switch (transition) {
    // case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
    case GST_STATE_CHANGE_NULL_TO_READY:
      if (gst_base_src_is_live (GST_BASE_SRC (element)))
        if (!gst_shmdata_src_start_reading (self))
          return GST_STATE_CHANGE_FAILURE;
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS (parent_class)->change_state (element, transition);
  if (ret == GST_STATE_CHANGE_FAILURE)
    return ret;

  switch (transition) {
    //case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
    case GST_STATE_CHANGE_READY_TO_NULL:
      if (gst_base_src_is_live (GST_BASE_SRC (element)))
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
