/*
 * This file is part of switcher-gtk.
 *
 * switcher-gtk is free software; you can redistribute it and/or
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

#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include <gdk/gdkkeysyms.h>
#include <gdk/gdkcursor.h>
#include "switcher/gst-utils.hpp"
#include "switcher/quiddity-command.hpp"
#include "switcher/quiddity-manager-impl.hpp"
#include "switcher/shmdata-utils.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/std2.hpp"
#ifdef HAVE_CONFIG_H
#include "../../config.h"
#endif
#include "./gtkvideo.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    GTKVideo,
    "gtkwin",
    "Video Display (configurable)",
    "video",
    "reader/device",
    "Video window with fullscreen",
    "LGPL",
    "Nicolas Bouillot");

guint GTKVideo::instances_counter_ = 0;
std::thread GTKVideo::gtk_main_thread_ {};

GTKVideo::GTKVideo(const std::string &):
    shmcntr_(static_cast<Quiddity *>(this)),
    gst_pipeline_(std2::make_unique<GstPipeliner>(
        nullptr,
        [this](GstMessage *msg){return this->bus_sync(msg);})),
    gtk_custom_props_(std::make_shared<CustomPropertyHelper>()) {
}

bool GTKVideo::init() {
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  if (!remake_elements())
    return false;
  if (instances_counter_ == 0) {
    if (0 == gtk_main_level()) {
      gtk_main_thread_ = std::thread(&GTKVideo::gtk_main_loop_thread);
      gtk_main_thread_.detach();
    }
    else
      g_debug("gtkvideosink: GTK main loop detected, using it");
  }
  instances_counter_++;
  shmcntr_.install_connect_method(
      [this](const std::string &shmpath){return this->on_shmdata_connect(shmpath);},
      [this](const std::string &){return this->on_shmdata_disconnect();},
      [this](){return this->on_shmdata_disconnect();},
      [this](const std::string &caps){return this->can_sink_caps(caps);},
      1);
  fullscreen_prop_spec_ =
      gtk_custom_props_->make_boolean_property("fullscreen",
                                               "Enable/Disable Fullscreen",
                                               (gboolean) FALSE, (GParamFlags)
                                               G_PARAM_READWRITE,
                                               GTKVideo::set_fullscreen,
                                               GTKVideo::get_fullscreen,
                                               this);
  install_property_by_pspec(gtk_custom_props_->get_gobject(),
                            fullscreen_prop_spec_, "fullscreen",
                            "Fullscreen");
  install_property(G_OBJECT(videoflip_.get_raw()),
                   "method", "method", "Flip Method");
  install_property(G_OBJECT(gamma_.get_raw()), "gamma", "gamma", "Gamma");
  install_property(G_OBJECT(videobalance_.get_raw()),
                   "contrast", "contrast", "Contrast");
  install_property(G_OBJECT(videobalance_.get_raw()),
                   "brightness", "brightness", "Brightness");
  install_property(G_OBJECT(videobalance_.get_raw()), "hue", "hue", "Hue");
  install_property(G_OBJECT(videobalance_.get_raw()),
                   "saturation", "saturation", "Saturation");
  title_ = g_strdup(get_name().c_str());
  title_prop_spec_ =
      gtk_custom_props_->make_string_property("title",
                                              "Window Title",
                                              title_,
                                              (GParamFlags)G_PARAM_READWRITE,
                                              GTKVideo::set_title,
                                              GTKVideo::get_title, this);
  install_property_by_pspec(gtk_custom_props_->get_gobject(),
                            title_prop_spec_, "title", "Window Title");
  std::unique_lock<std::mutex> lock(wait_window_mutex_);
  gtk_idle_add((GtkFunction) create_ui, this);
  wait_window_cond_.wait(lock);
  if (nullptr == display_)
    return false;

  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  return true;
}

void GTKVideo::gtk_main_loop_thread() {
  if (!gtk_init_check(nullptr, nullptr)) {
    g_debug("GTKVideo::init, cannot init gtk");
  }
  g_debug("GTKVideo::gtk_main_loop_thread starting");
  gtk_main();
}

gboolean GTKVideo::key_pressed_cb(GtkWidget */*widget */ ,
                                  GdkEventKey *event,
                                  gpointer data) {
  GTKVideo *context = static_cast<GTKVideo *>(data);
  // gchar outbuf[] = "coucou";
  // auto i = g_unichar_to_utf8(gdk_keyval_to_unicode(event->keyval),
  //                            outbuf);
  // outbuf[i] = '\0';
  // g_print("(%d)%s", i, outbuf);
  QuiddityManager_Impl::ptr manager;
  switch (event->keyval) {
    case GDK_f:
      context->toggle_fullscreen();
      break;
    case GDK_F:
      context->toggle_fullscreen();
      break;
    case GDK_Escape:
      context->toggle_fullscreen();
      break;
    case GDK_q:
      manager = context->manager_impl_.lock();
      if ((bool) manager)
        manager->remove(context->get_name());
      else
        g_debug("GTKVideo::key_pressed_cb q pressed, closing window");
      break;
    default:
      break;
  }
  return TRUE;
}

void GTKVideo::window_destroyed(gpointer user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  std::unique_lock<std::mutex> lock(context->window_destruction_mutex_);
  context->window_destruction_cond_.notify_all();
}

gboolean GTKVideo::destroy_window(gpointer user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  gtk_widget_destroy(GTK_WIDGET(context->main_window_));
  window_destroyed(context);
  return FALSE;
}

GTKVideo::~GTKVideo() {
  gst_pipeline_.reset();
  g_idle_remove_by_data(this);
  if (nullptr != title_)
    g_free(title_);
  // destroy child widgets too
  if (main_window_ != nullptr && GTK_IS_WIDGET(main_window_)) {
    std::unique_lock<std::mutex> lock(window_destruction_mutex_);
    // g_idle_add_full (G_PRIORITY_DEFAULT_IDLE,
    //     destroy_window,
    //     this,
    //     window_destroyed);
    gtk_idle_add(destroy_window, this);
    window_destruction_cond_.wait(lock);
  }
  if (blank_cursor_ != nullptr)
    gdk_cursor_destroy(blank_cursor_);
  // instances_counter_ --;
  // if (instances_counter_ == 0)
  //   {
  // g_debug ("GTKVideo::~GTKVideo invoking gtk_main_quit");
  // gtk_main_quit ();
  //   }
}

void GTKVideo::realize_cb(GtkWidget *widget, void *user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  GdkWindow *window = gtk_widget_get_window(widget);

  if (!gdk_window_ensure_native(window))
    g_debug("Couldn't create native window needed for GstXOverlay!");

  gdk_threads_enter();
  gdk_display_sync(context->display_);
  // gdk_error_trap_pop ();

  /* Retrieve window handler from GDK */
#if defined(GDK_WINDOWING_WIN32)
  context->window_handle_ = reinterpret_cast<guintptr>(GDK_WINDOW_HWND(window));
#elif defined(GDK_WINDOWING_QUARTZ)
  context->window_handle_ = gdk_quartz_window_get_nsview(window);
#elif defined(GDK_WINDOWING_X11)
  context->window_handle_ = GDK_WINDOW_XID(window);
#endif
  gdk_threads_leave();
  std::unique_lock<std::mutex> lock(context->wait_window_mutex_);
  context->wait_window_cond_.notify_all();
}

/* This function is called when the main window is closed */
void GTKVideo::delete_event_cb(GtkWidget * /*widget */ ,
                               GdkEvent * /*event */ ,
                               void *user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);

  context->gst_pipeline_.reset();
  gtk_widget_destroy(context->main_window_);
  context->main_window_ = nullptr;
  QuiddityManager_Impl::ptr manager = context->manager_impl_.lock();
  if ((bool) manager)
    manager->remove(context->get_name());
  else
    g_debug("GTKVideo::delete_event_cb cannot remove quiddity");
}

gboolean GTKVideo::create_ui(void *user_data) {
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  context->display_ = gdk_display_get_default();
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  if (nullptr == context->display_) {
    g_debug("gtkvideo: no default display, cannot create window");
    std::unique_lock<std::mutex> lock(context->wait_window_mutex_);
    context->wait_window_cond_.notify_all();
    return FALSE;
  }
  g_print("--%s %d\n", __FUNCTION__, __LINE__);

  context->main_window_ = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  g_signal_connect(G_OBJECT(context->main_window_),
                   "delete-event", G_CALLBACK(delete_event_cb), context);
  context->video_window_ = gtk_drawing_area_new();
  gtk_widget_set_double_buffered(context->video_window_, FALSE);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  GdkColor color;
  gdk_color_parse("black", &color);
  gtk_widget_modify_bg(context->video_window_, GTK_STATE_NORMAL, &color);
  g_signal_connect(context->video_window_, "realize",
                   G_CALLBACK(realize_cb), context);
  gtk_container_add(GTK_CONTAINER(context->main_window_),
                    context->video_window_);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  gtk_window_set_default_size(GTK_WINDOW(context->main_window_), 640, 480);
  gtk_window_set_title(GTK_WINDOW(context->main_window_), context->title_);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  context->blank_cursor_ = gdk_cursor_new(GDK_BLANK_CURSOR);
  gtk_widget_set_events(context->main_window_, GDK_KEY_PRESS_MASK);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  g_signal_connect(G_OBJECT(context->main_window_),
                   "key-press-event",
                   G_CALLBACK(GTKVideo::key_pressed_cb), context);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  gtk_widget_show_all((GtkWidget *) context->main_window_);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  return FALSE;
}

void GTKVideo::toggle_fullscreen() {
  if (is_fullscreen_)
    set_fullscreen(FALSE, this);
  else
    set_fullscreen(TRUE, this);
}

gboolean GTKVideo::get_fullscreen(void *user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  return context->is_fullscreen_;
}

void GTKVideo::set_fullscreen(gboolean fullscreen, void *user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);

  if (fullscreen) {
    if (context->main_window_ != nullptr) {
      gdk_window_set_cursor(GDK_WINDOW
                            (context->video_window_->window),
                            context->blank_cursor_);
      gtk_window_fullscreen(GTK_WINDOW(context->main_window_));
    }
    context->is_fullscreen_ = TRUE;
  }
  else {
    if (context->main_window_ != nullptr) {
      gdk_window_set_cursor(GDK_WINDOW
                            (context->video_window_->window), nullptr);
      gtk_window_unfullscreen(GTK_WINDOW(context->main_window_));
    }
    context->is_fullscreen_ = FALSE;
  }
  context->gtk_custom_props_->
      notify_property_changed(context->fullscreen_prop_spec_);
}

void GTKVideo::set_title(const gchar *value, void *user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  if (nullptr != context->title_)
    g_free(context->title_);
  context->title_ = g_strdup(value);
  gtk_window_set_title(GTK_WINDOW(context->main_window_), context->title_);
  context->gtk_custom_props_->
      notify_property_changed(context->title_prop_spec_);
}

const gchar *GTKVideo::get_title(void *user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  return context->title_;
}

bool GTKVideo::remake_elements(){
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  if (!UGstElem::renew(shmsrc_) || !UGstElem::renew(queue_)
      || !UGstElem::renew(videoconvert_) || !UGstElem::renew(videoflip_)
      || !UGstElem::renew(gamma_) || !UGstElem::renew(videobalance_)
      || !UGstElem::renew(xvimagesink_))
    return false;
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  g_object_set(G_OBJECT(xvimagesink_.get_raw()),
               "force-aspect-ratio", TRUE,
               "draw-borders", TRUE,
               "sync", FALSE,
               "qos", FALSE,
               nullptr);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  g_object_set_data(G_OBJECT(xvimagesink_.get_raw()),
                    "on-error-delete",
                    (gpointer) get_name().c_str());
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  return true;
}

bool GTKVideo::on_shmdata_disconnect() {
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  prune_tree(".shmdata.reader." + shmpath_);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  shm_sub_.reset();
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  On_scope_exit{gst_pipeline_ =
        std2::make_unique<GstPipeliner>(nullptr,
                                        [this](GstMessage *msg){
                                          return this->bus_sync(msg);
                                        });};
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  return remake_elements();
}

bool GTKVideo::on_shmdata_connect(const std::string &shmpath) {
  shmpath_ = shmpath;
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  g_object_set(G_OBJECT(shmsrc_.get_raw()),
               "socket-path", shmpath_.c_str(),
               nullptr);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  shm_sub_ = std2::make_unique<GstShmdataSubscriber>(
      shmsrc_.get_raw(),
      [this](std::string &&caps){
        this->graft_tree(".shmdata.reader." + shmpath_,
                         ShmdataUtils::make_tree(caps,
                                                 ShmdataUtils::get_category(caps),
                                                 0));
      },
      [this](GstShmdataSubscriber::num_bytes_t byte_rate){
        this->graft_tree(".shmdata.reader." + shmpath_ + ".byte_rate",
                         data::Tree::make(std::to_string(byte_rate)));
      });
  g_print("--%s %d\n", __FUNCTION__, __LINE__);

  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   shmsrc_.get_raw(),
                   queue_.get_raw(),
                   videoconvert_.get_raw(),
                   videoflip_.get_raw(),
                   gamma_.get_raw(),
                   videobalance_.get_raw(),
                   xvimagesink_.get_raw(),
                   nullptr);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  gst_element_link_many(shmsrc_.get_raw(),
                        queue_.get_raw(),
                        videoconvert_.get_raw(),
                        videoflip_.get_raw(),
                        gamma_.get_raw(),
                        videobalance_.get_raw(),
                        xvimagesink_.get_raw(),
                        nullptr);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  gst_pipeline_->play(true);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  return true;
}

bool GTKVideo::can_sink_caps(std::string caps) {
  return GstUtils::can_sink_caps("videoconvert", caps);
};

GstBusSyncReply GTKVideo::bus_sync(GstMessage *msg){
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  if (!gst_is_video_overlay_prepare_window_handle_message (msg))
    return GST_BUS_PASS;
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  GstVideoOverlay *overlay = GST_VIDEO_OVERLAY(GST_MESSAGE_SRC(msg));
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  gst_video_overlay_set_window_handle (overlay, window_handle_);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  gst_message_unref (msg);
  g_print("--%s %d\n", __FUNCTION__, __LINE__);
  return GST_BUS_DROP;
}

}  // namespace switcher
