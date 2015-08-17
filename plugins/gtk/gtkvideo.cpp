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
    "reader/device/writer",
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
                            fullscreen_prop_spec_,
                            "fullscreen",
                            "Fullscreen");
  keyb_interaction_spec_ =
      gtk_custom_props_->make_boolean_property("keyb_interaction",
                                               "Enable/Disable keybord interaction",
                                               keyb_interaction_,
                                               (GParamFlags) G_PARAM_READWRITE,
                                               GTKVideo::set_keyb_interaction,
                                               GTKVideo::get_keyb_interaction,
                                               this);
  install_property_by_pspec(gtk_custom_props_->get_gobject(),
                            keyb_interaction_spec_,
                            "keyb_interaction",
                            "Keyboard Interaction");

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
  keyb_shm_ = std2::make_unique<ShmdataWriter>(this,
                                               make_file_name("keyb"),
                                               sizeof(KeybEvent),
                                               "application/x-keyboard-events");
  if(!keyb_shm_.get()) {
    g_warning("GTK keyboard event shmdata writer failled");
    keyb_shm_.reset(nullptr);
  }
  mouse_shm_ = std2::make_unique<ShmdataWriter>(this,
                                               make_file_name("mouse"),
                                               sizeof(MouseEvent),
                                               "application/x-mouse-events");
  if(!mouse_shm_.get()) {
    g_warning("GTK mouse event shmdata writer failled");
    mouse_shm_.reset(nullptr);
  }
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
  guint32 val = event->keyval;
  auto keybevent = KeybEvent(val, 1);
  context->keyb_shm_->writer(&shmdata::Writer::copy_to_shm, &keybevent, sizeof(KeybEvent));
  context->keyb_shm_->bytes_written(sizeof(KeybEvent));
  if (context->keyb_interaction_) {
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
      default:
        break;
    }
  }  // if (context->keyb_interaction_) 
  return TRUE;
}

gboolean GTKVideo::key_release_cb(GtkWidget */*widget */ ,
                                  GdkEventKey *event,
                                  gpointer data) {
  GTKVideo *context = static_cast<GTKVideo *>(data);
  guint32 val = event->keyval;
  auto keybevent = KeybEvent(val, 0);
  context->keyb_shm_->writer(&shmdata::Writer::copy_to_shm, &keybevent, sizeof(KeybEvent));
  context->keyb_shm_->bytes_written(sizeof(KeybEvent));
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
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  context->display_ = gdk_display_get_default();
  if (nullptr == context->display_) {
    g_debug("gtkvideo: no default display, cannot create window");
    std::unique_lock<std::mutex> lock(context->wait_window_mutex_);
    context->wait_window_cond_.notify_all();
    return FALSE;
  }
  context->main_window_ = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  g_signal_connect(G_OBJECT(context->main_window_),
                   "delete-event", G_CALLBACK(delete_event_cb), context);
  context->video_window_ = gtk_drawing_area_new();
  gtk_widget_set_double_buffered(context->video_window_, FALSE);
  GdkColor color;
  gdk_color_parse("black", &color);
  gtk_widget_modify_bg(context->video_window_, GTK_STATE_NORMAL, &color);
  g_signal_connect(context->video_window_, "realize",
                   G_CALLBACK(realize_cb), context);
  g_signal_connect (context->video_window_,
                    "motion-notify-event",
                    G_CALLBACK (GTKVideo::motion_notify_event), context);
  g_signal_connect (context->video_window_,
                    "button-press-event",
                    G_CALLBACK (GTKVideo::button_event), context);
  g_signal_connect (context->video_window_,
                    "button-release-event",
                    G_CALLBACK (GTKVideo::button_event), context);
  g_signal_connect(context->video_window_,
                   "size-allocate",
                   G_CALLBACK(widget_getsize), context);
  gtk_widget_set_events (context->video_window_,
                         GDK_EXPOSURE_MASK
                         | GDK_LEAVE_NOTIFY_MASK
                         | GDK_BUTTON_PRESS_MASK
                         | GDK_POINTER_MOTION_MASK
                         | GDK_POINTER_MOTION_HINT_MASK);
  gtk_container_add(GTK_CONTAINER(context->main_window_),
                    context->video_window_);
  gtk_window_set_default_size(GTK_WINDOW(context->main_window_), 640, 480);
  gtk_window_set_title(GTK_WINDOW(context->main_window_), context->title_);
  context->blank_cursor_ = gdk_cursor_new(GDK_BLANK_CURSOR);
  gtk_widget_set_events(context->main_window_, GDK_KEY_PRESS_MASK);
  g_signal_connect(G_OBJECT(context->main_window_),
                   "key-press-event",
                   G_CALLBACK(GTKVideo::key_pressed_cb),
                   context);
  g_signal_connect(G_OBJECT(context->main_window_),
                   "key-release-event",
                   G_CALLBACK(GTKVideo::key_release_cb),
                   context);
  
  gtk_widget_show_all((GtkWidget *) context->main_window_);
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
  if (!UGstElem::renew(shmsrc_) || !UGstElem::renew(queue_)
      || !UGstElem::renew(videoconvert_) || !UGstElem::renew(videoflip_)
      || !UGstElem::renew(gamma_) || !UGstElem::renew(videobalance_)
      || !UGstElem::renew(xvimagesink_))
    return false;
  g_object_set(G_OBJECT(xvimagesink_.get_raw()),
               "force-aspect-ratio", TRUE,
               "draw-borders", FALSE,
               "sync", FALSE,
               "qos", FALSE,
               nullptr);
  return true;
}

bool GTKVideo::on_shmdata_disconnect() {
  prune_tree(".shmdata.reader." + shmpath_);
  shm_sub_.reset();
  On_scope_exit{gst_pipeline_ =
        std2::make_unique<GstPipeliner>(nullptr,
                                        [this](GstMessage *msg){
                                          return this->bus_sync(msg);
                                        });};
  return remake_elements();
}

bool GTKVideo::on_shmdata_connect(const std::string &shmpath) {
  shmpath_ = shmpath;
  g_object_set(G_OBJECT(shmsrc_.get_raw()),
               "socket-path", shmpath_.c_str(),
               nullptr);
  shm_sub_ = std2::make_unique<GstShmdataSubscriber>(
      shmsrc_.get_raw(),
      [this]( const std::string &caps){
        this->graft_tree(".shmdata.reader." + shmpath_,
                         ShmdataUtils::make_tree(caps,
                                                 ShmdataUtils::get_category(caps),
                                                 0));
        GstCaps *gstcaps = gst_caps_from_string(caps.c_str());
        On_scope_exit{if (gstcaps) gst_caps_unref(gstcaps);};
        GstStructure *caps_struct = gst_caps_get_structure (gstcaps, 0);
        const GValue *width_val = gst_structure_get_value (caps_struct, "width");
        const GValue *height_val = gst_structure_get_value (caps_struct, "height");
        this->vid_width_ = g_value_get_int(width_val);
        this->vid_height_ = g_value_get_int(height_val);
        this->update_padding(this->video_window_);
      },
      [this](GstShmdataSubscriber::num_bytes_t byte_rate){
        this->graft_tree(".shmdata.reader." + shmpath_ + ".byte_rate",
                         data::Tree::make(std::to_string(byte_rate)));
      });
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   shmsrc_.get_raw(),
                   queue_.get_raw(),
                   videoconvert_.get_raw(),
                   videoflip_.get_raw(),
                   gamma_.get_raw(),
                   videobalance_.get_raw(),
                   xvimagesink_.get_raw(),
                   nullptr);
  gst_element_link_many(shmsrc_.get_raw(),
                        queue_.get_raw(),
                        videoconvert_.get_raw(),
                        videoflip_.get_raw(),
                        gamma_.get_raw(),
                        videobalance_.get_raw(),
                        xvimagesink_.get_raw(),
                        nullptr);
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  gst_pipeline_->play(true);
  return true;
}

bool GTKVideo::can_sink_caps(std::string caps) {
  return GstUtils::can_sink_caps("videoconvert", caps);
};

GstBusSyncReply GTKVideo::bus_sync(GstMessage *msg){
  if (!gst_is_video_overlay_prepare_window_handle_message (msg))
    return GST_BUS_PASS;
  GstVideoOverlay *overlay = GST_VIDEO_OVERLAY(GST_MESSAGE_SRC(msg));
  gst_video_overlay_set_window_handle (overlay, window_handle_);
  gst_message_unref (msg);
  return GST_BUS_DROP;
}

gboolean GTKVideo::get_keyb_interaction(void *user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  return context->keyb_interaction_;
}

void GTKVideo::set_keyb_interaction(gboolean keyb_interaction, void *user_data) {
  GTKVideo *context = static_cast<GTKVideo *>(user_data);
  context->keyb_interaction_ = keyb_interaction;
  context->gtk_custom_props_->
      notify_property_changed(context->keyb_interaction_spec_);
}

gboolean GTKVideo::button_event (GtkWidget */*widget*/,
                                 GdkEventButton *event,
                                 gpointer data){
  GTKVideo *context = static_cast<GTKVideo *>(data);
  if (-1 == context->vid_width_)
    return TRUE;
  int x, y;
  GdkModifierType state;
  gdk_window_get_pointer (event->window, &x, &y, &state);
  context->write_mouse_info_to_shmdata(x, y, state);
  return TRUE;
}

gboolean GTKVideo::motion_notify_event (GtkWidget */*widget*/,
                                        GdkEventMotion *event,
                                        gpointer data){
  GTKVideo *context = static_cast<GTKVideo *>(data);
  if (-1 == context->vid_width_)
    return TRUE;
  int x, y;
  GdkModifierType state;
  if (event->is_hint)
    gdk_window_get_pointer (event->window, &x, &y, &state);
  else {
    x = event->x;
    y = event->y;
  }
  context->write_mouse_info_to_shmdata(x, y, state);
  return TRUE;
}

void GTKVideo::widget_getsize(GtkWidget *widget,
                              GtkAllocation */*allocation*/,
                              void *data) {
  GTKVideo *context = static_cast<GTKVideo *>(data);
  context->horizontal_padding_ = 0;
  context->vertical_padding_ = 0;
  context->drawed_video_width_ = widget->allocation.width;
  context->drawed_video_height_ = widget->allocation.height;
  context->update_padding(widget);
}

void GTKVideo::update_padding(GtkWidget *widget){
  if (-1 == drawed_video_width_ || -1 == vid_width_)
    return;
  if (widget->allocation.width / widget->allocation.height
      > vid_width_ / vid_height_) {
    // padding is horizontal
    drawed_video_width_ =
        widget->allocation.height * vid_width_ / vid_height_;
    horizontal_padding_ = (widget->allocation.width - drawed_video_width_) / 2;
  } else {
    // padding is vertical
    drawed_video_height_ =
        widget->allocation.width * vid_height_ / vid_width_;
    vertical_padding_ = (widget->allocation.height - drawed_video_height_) / 2;
  }
}

void GTKVideo::write_mouse_info_to_shmdata(
    int x, int y, const GdkModifierType &state){
  int vid_x = (x - horizontal_padding_)
      * 100000. / drawed_video_width_;
  int vid_y = (y - vertical_padding_)
      * 100000. / drawed_video_height_;
  if (0 > vid_x || 100000 < vid_x || 0 > vid_y || 100000 < vid_y)
    return;  // pointer is out of the video
  int button_mask = 0;
  if (state & GDK_BUTTON1_MASK)
    button_mask = 1; 
  if (state & GDK_BUTTON2_MASK)
    button_mask += 2; 
  if (state & GDK_BUTTON3_MASK)
    button_mask += 4;
  auto mouse_event = MouseEvent(vid_x, vid_y, button_mask);
  mouse_shm_->writer(&shmdata::Writer::copy_to_shm, &mouse_event, sizeof(MouseEvent));
  mouse_shm_->bytes_written(sizeof(MouseEvent));
}

}  // namespace switcher
