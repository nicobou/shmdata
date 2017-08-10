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

#include "./gtkvideo.hpp"
#include <gst/gst.h>
#include <gst/video/videooverlay.h>
#include "switcher/gprop-to-prop.hpp"
#include "switcher/gst-utils.hpp"
#include "switcher/invocation-spec.hpp"
#include "switcher/quiddity-container.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GTKVideo,
                                     "gtkwin",
                                     "GTK Video Display (configurable)",
                                     "video",
                                     "reader/device/occasional-writer",
                                     "Video window with fullscreen",
                                     "LGPL",
                                     "Nicolas Bouillot");

guint GTKVideo::instances_counter_ = 0;
std::thread GTKVideo::gtk_main_thread_{};

GTKVideo::GTKVideo(QuiddityConfiguration&& conf)
    : Quiddity(std::forward<QuiddityConfiguration>(conf)),
      shmcntr_(static_cast<Quiddity*>(this)),
      gst_pipeline_(std::make_unique<GstPipeliner>(
          nullptr, [this](GstMessage* msg) { return this->bus_sync(msg); })),
      title_(get_name()),
      fullscreen_id_(pmanage<MPtr(&PContainer::make_bool)>("fullscreen",
                                                           [this](const bool& val) {
                                                             is_fullscreen_ = val;
                                                             g_idle_add((GSourceFunc)set_fullscreen,
                                                                        this);
                                                             return true;
                                                           },
                                                           [this]() { return is_fullscreen_; },
                                                           "Fullscreen",
                                                           "Enable/Disable Fullscreen",
                                                           is_fullscreen_)),
      xevents_to_shmdata_id_(pmanage<MPtr(&PContainer::make_bool)>(
          "xevents",
          [this](bool val) {
            xevents_to_shmdata_ = val;
            if (xevents_to_shmdata_) {
              keyb_shm_ = std::make_unique<ShmdataWriter>(
                  this, make_file_name("keyb"), sizeof(KeybEvent), "application/x-keyboard-events");
              if (!keyb_shm_.get()) {
                warning("GTK keyboard event shmdata writer failed");
                keyb_shm_.reset(nullptr);
              }

              mouse_shm_ = std::make_unique<ShmdataWriter>(
                  this, make_file_name("mouse"), sizeof(MouseEvent), "application/x-mouse-events");
              if (!mouse_shm_.get()) {
                warning("GTK mouse event shmdata writer failed");
                mouse_shm_.reset(nullptr);
              }
            } else {
              mouse_shm_.reset(nullptr);
              keyb_shm_.reset(nullptr);
            }
            return true;
          },
          [this]() { return xevents_to_shmdata_; },
          "Keyboard/Mouse Events",
          "Capture Keyboard/Mouse Events",
          xevents_to_shmdata_)),
      decorated_id_(pmanage<MPtr(&PContainer::make_bool)>("decorated",
                                                          [this](bool val) {
                                                            decorated_ = val;
                                                            g_idle_add((GSourceFunc)set_geometry,
                                                                       this);
                                                            return true;
                                                          },
                                                          [this]() { return decorated_; },
                                                          "Window Decoration",
                                                          "Show/Hide Window Decoration",
                                                          true)),
      always_on_top_id_(
          pmanage<MPtr(&PContainer::make_bool)>("always_on_top",
                                                [this](bool val) {
                                                  always_on_top_ = val;
                                                  g_idle_add((GSourceFunc)set_geometry, this);
                                                  return true;
                                                },
                                                [this]() { return always_on_top_; },
                                                "Always On Top",
                                                "Toggle Window Always On Top",
                                                true)) {
  if (!remake_elements()) {
    is_valid_ = false;
    return;
  }

  if (instances_counter_ == 0) {
    if (0 == gtk_main_level()) {
      gtk_main_thread_ = std::thread(&GTKVideo::gtk_main_loop_thread);
      gtk_main_thread_.detach();
    } else
      debug("gtkvideosink: GTK main loop detected, using it");
  }

  instances_counter_++;

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return this->on_shmdata_connect(shmpath); },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);

  pmanage<MPtr(&PContainer::make_bool)>("keyb_interaction",
                                        [this](const bool& val) {
                                          keyb_interaction_ = val;
                                          return true;
                                        },
                                        [this]() { return keyb_interaction_; },
                                        "Keyboard Shortcuts",
                                        "Enable/Disable keybord shortcuts",
                                        keyb_interaction_);

  pmanage<MPtr(&PContainer::make_string)>("title",
                                          [this](const std::string& val) {
                                            title_ = val;
                                            g_idle_add((GSourceFunc)set_title, this);
                                            return true;
                                          },
                                          [this]() { return title_; },
                                          "Window Title",
                                          "Window Title",
                                          title_);

  std::unique_lock<std::mutex> lock(wait_window_mutex_);
  g_idle_add((GSourceFunc)create_ui, this);
  wait_window_cond_.wait(lock);

  if (nullptr == display_) {
    message("ERROR:GDK could not find a display (is the server running in ssh?)");
    is_valid_ = false;
    return;
  }

  int max_width = gdk_screen_get_width(gdk_screen_get_default());
  int max_height = gdk_screen_get_height(gdk_screen_get_default());

  width_id_ = pmanage<MPtr(&PContainer::make_int)>("width",
                                                   [this](const int& val) {
                                                     width_ = val;
                                                     g_idle_add((GSourceFunc)set_geometry, this);
                                                     return true;
                                                   },
                                                   [this]() { return width_; },
                                                   "Window Width",
                                                   "Set Window Width",
                                                   640,
                                                   1,
                                                   max_width);

  height_id_ = pmanage<MPtr(&PContainer::make_int)>("height",
                                                    [this](const int& val) {
                                                      height_ = val;
                                                      g_idle_add((GSourceFunc)set_geometry, this);
                                                      return true;
                                                    },
                                                    [this]() { return height_; },
                                                    "Window Height",
                                                    "Set Window Height",
                                                    480,
                                                    1,
                                                    max_height);

  position_x_id_ =
      pmanage<MPtr(&PContainer::make_int)>("position_x",
                                           [this](const int& val) {
                                             position_x_ = val;
                                             g_idle_add((GSourceFunc)set_geometry, this);
                                             return true;
                                           },
                                           [this]() { return position_x_; },
                                           "Window Position X",
                                           "Set Window Horizontal Position",
                                           0,
                                           0,
                                           max_width);

  position_y_id_ =
      pmanage<MPtr(&PContainer::make_int)>("position_y",
                                           [this](const int& val) {
                                             position_y_ = val;
                                             g_idle_add((GSourceFunc)set_geometry, this);
                                             return true;
                                           },
                                           [this]() { return position_y_; },
                                           "Window Position Y",
                                           "Set Window Vertical Position",
                                           0,
                                           0,
                                           max_height);

  position_task_ = std::make_unique<PeriodicTask<>>(
      [this]() { g_idle_add((GSourceFunc)window_update_position, this); },
      std::chrono::milliseconds(500));

  install_gst_properties();

}

GTKVideo::~GTKVideo() {
  keyb_shm_.reset();
  mouse_shm_.reset();
  gst_pipeline_.reset();
  g_idle_remove_by_data(this);
  // destroy child widgets too
  if (main_window_ != nullptr && GTK_IS_WIDGET(main_window_)) {
    std::unique_lock<std::mutex> lock(window_destruction_mutex_);
    g_idle_add(destroy_window, this);
    window_destruction_cond_.wait(lock);
  }
}

void GTKVideo::gtk_main_loop_thread() {
  gtk_init_check(nullptr, nullptr);
  gtk_main();
}

gboolean GTKVideo::key_pressed_cb(GtkWidget* /*widget */, GdkEventKey* event, gpointer data) {
  GTKVideo* context = static_cast<GTKVideo*>(data);

  if (context->keyb_shm_.get()) {
    guint32 val = event->keyval;
    auto keybevent = KeybEvent(val, 1);
    context->keyb_shm_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(&keybevent, sizeof(KeybEvent));
    context->keyb_shm_->bytes_written(sizeof(KeybEvent));
  }

  if (context->keyb_interaction_) {
    switch (event->keyval) {
      case GDK_KEY_f:
      case GDK_KEY_F:
      case GDK_KEY_Escape:
        context->pmanage<MPtr(&PContainer::set<bool>)>(
            context->fullscreen_id_,
            !context->is_fullscreen_);  // toggle fullscreen
        break;
      case GDK_KEY_d:
      case GDK_KEY_D:
        context->pmanage<MPtr(&PContainer::set<bool>)>(context->decorated_id_,
                                                       !context->decorated_);  // toggle decoration
        break;
      case GDK_KEY_t:
      case GDK_KEY_T:
        context->pmanage<MPtr(&PContainer::set<bool>)>(
            context->always_on_top_id_,
            !context->always_on_top_);  // toggle always on top status
        break;
      default:
        break;
    }
  }  // if (context->keyb_interaction_)

  return TRUE;
}

gboolean GTKVideo::key_release_cb(GtkWidget* /*widget */, GdkEventKey* event, gpointer data) {
  GTKVideo* context = static_cast<GTKVideo*>(data);

  if (!context->keyb_shm_.get()) return TRUE;
  guint32 val = event->keyval;
  auto keybevent = KeybEvent(val, 0);
  context->keyb_shm_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(&keybevent, sizeof(KeybEvent));
  context->keyb_shm_->bytes_written(sizeof(KeybEvent));
  return TRUE;
}

void GTKVideo::window_destroyed(gpointer user_data) {
  GTKVideo* context = static_cast<GTKVideo*>(user_data);
  std::unique_lock<std::mutex> lock(context->window_destruction_mutex_);
  context->window_destruction_cond_.notify_all();
}

gboolean GTKVideo::destroy_window(gpointer user_data) {
  GTKVideo* context = static_cast<GTKVideo*>(user_data);
  gtk_widget_destroy(GTK_WIDGET(context->main_window_));
  window_destroyed(context);
  return FALSE;
}

void GTKVideo::realize_cb(GtkWidget* widget, void* user_data) {
  GTKVideo* context = static_cast<GTKVideo*>(user_data);
  GdkWindow* window = gtk_widget_get_window(widget);
  if (!gdk_window_ensure_native(window))
    context->debug("Couldn't create native window needed for GstXOverlay!");
  gdk_display_sync(context->display_);
/* Retrieve window handler from GDK */
#if defined(GDK_WINDOWING_WIN32)
  context->window_handle_ = reinterpret_cast<guintptr>(GDK_WINDOW_HWND(window));
#elif defined(GDK_WINDOWING_QUARTZ)
  context->window_handle_ = gdk_quartz_window_get_nsview(window);
#elif defined(GDK_WINDOWING_X11)
  context->window_handle_ = GDK_WINDOW_XID(window);
#endif
  std::unique_lock<std::mutex> lock(context->wait_window_mutex_);
  context->wait_window_cond_.notify_all();
}

/* This function is called when the main window is closed */
void GTKVideo::delete_event_cb(GtkWidget* /*widget */, GdkEvent* /*event */, void* user_data) {
  GTKVideo* context = static_cast<GTKVideo*>(user_data);
  context->gst_pipeline_.reset();
  gtk_widget_destroy(context->main_window_);
  context->main_window_ = nullptr;
  context->qcontainer_->remove(context->get_name());
}

gboolean GTKVideo::create_ui(void* user_data) {
  GTKVideo* context = static_cast<GTKVideo*>(user_data);
  context->display_ = gdk_display_get_default();
  if (nullptr == context->display_) {
    context->debug("gtkvideo: no default display, cannot create window");
    std::unique_lock<std::mutex> lock(context->wait_window_mutex_);
    context->wait_window_cond_.notify_all();
    return FALSE;
  }

  GdkDeviceManager* device_manager = gdk_display_get_device_manager(context->display_);
  context->device_ = gdk_device_manager_get_client_pointer(device_manager);

  context->main_window_ = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  g_signal_connect(
      G_OBJECT(context->main_window_), "delete-event", G_CALLBACK(delete_event_cb), context);
  context->video_window_ = gtk_drawing_area_new();
  g_signal_connect(context->video_window_, "realize", G_CALLBACK(realize_cb), context);
  g_signal_connect(context->video_window_,
                   "motion-notify-event",
                   G_CALLBACK(GTKVideo::motion_notify_event),
                   context);
  if (context->device_) {
    g_signal_connect(
        context->video_window_, "button-press-event", G_CALLBACK(GTKVideo::button_event), context);
    g_signal_connect(context->video_window_,
                     "button-release-event",
                     G_CALLBACK(GTKVideo::button_event),
                     context);
  }

  g_signal_connect(G_OBJECT(gtk_widget_get_toplevel(context->main_window_)),
                   "state-flags-changed",
                   G_CALLBACK(widget_has_focus),
                   context);

  g_signal_connect(context->video_window_, "size-allocate", G_CALLBACK(widget_getsize), context);
  gtk_widget_set_events(context->video_window_,
                        GDK_EXPOSURE_MASK | GDK_LEAVE_NOTIFY_MASK | GDK_BUTTON_PRESS_MASK |
                            GDK_POINTER_MOTION_MASK | GDK_POINTER_MOTION_HINT_MASK |
                            GDK_FOCUS_CHANGE_MASK);
  gtk_container_add(GTK_CONTAINER(context->main_window_), context->video_window_);
  gtk_window_set_decorated(GTK_WINDOW(context->main_window_), context->decorated_);
  gtk_window_resize(GTK_WINDOW(context->main_window_), context->width_, context->height_);
  gtk_window_move(GTK_WINDOW(context->main_window_), context->position_x_, context->position_y_);
  gtk_window_set_title(GTK_WINDOW(context->main_window_), context->title_.c_str());
  context->blank_cursor_ = gdk_cursor_new_for_display(context->display_, GDK_BLANK_CURSOR);
  gtk_widget_set_events(context->main_window_, GDK_KEY_PRESS_MASK);
  g_signal_connect(G_OBJECT(context->main_window_),
                   "key-press-event",
                   G_CALLBACK(GTKVideo::key_pressed_cb),
                   context);
  g_signal_connect(G_OBJECT(context->main_window_),
                   "key-release-event",
                   G_CALLBACK(GTKVideo::key_release_cb),
                   context);

  gtk_widget_show_all((GtkWidget*)context->main_window_);
  return FALSE;
}

bool GTKVideo::remake_elements() {
  remove_gst_properties();
  if (!UGstElem::renew(shmsrc_) || !UGstElem::renew(queue_) || !UGstElem::renew(videoconvert_) ||
      !UGstElem::renew(videoflip_, {"method"}) || !UGstElem::renew(gamma_, {"gamma"}) ||
      !UGstElem::renew(videobalance_, {"contrast", "brightness", "hue", "saturation"}) ||
      !UGstElem::renew(xvimagesink_)) {
    error("gtkvideo could not renew GStreamer elements");
    return false;
  }
  g_object_set(G_OBJECT(xvimagesink_.get_raw()),
               "force-aspect-ratio",
               TRUE,
               "draw-borders",
               FALSE,
               "sync",
               FALSE,
               "qos",
               FALSE,
               nullptr);
  g_object_set(G_OBJECT(queue_.get_raw()), "max-size-buffers", 3, nullptr);
  g_object_set(G_OBJECT(shmsrc_.get_raw()), "copy-buffers", TRUE, nullptr);
  install_gst_properties();
  return true;
}

bool GTKVideo::on_shmdata_disconnect() {
  prune_tree(".shmdata.reader." + shmpath_);
  shm_sub_.reset();
  On_scope_exit {
    gst_pipeline_ = std::make_unique<GstPipeliner>(
        nullptr, [this](GstMessage* msg) { return this->bus_sync(msg); });
  };

  return remake_elements();
}

bool GTKVideo::on_shmdata_connect(const std::string& shmpath) {
  shmpath_ = shmpath;
  g_object_set(G_OBJECT(shmsrc_.get_raw()), "socket-path", shmpath_.c_str(), nullptr);
  shm_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmsrc_.get_raw(),
      [this](const std::string& caps) {
        this->graft_tree(
            ".shmdata.reader." + shmpath_,
            ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
        GstCaps* gstcaps = gst_caps_from_string(caps.c_str());
        On_scope_exit {
          if (gstcaps) gst_caps_unref(gstcaps);
        };
        GstStructure* caps_struct = gst_caps_get_structure(gstcaps, 0);
        const GValue* width_val = gst_structure_get_value(caps_struct, "width");
        const GValue* height_val = gst_structure_get_value(caps_struct, "height");
        this->vid_width_ = g_value_get_int(width_val);
        this->vid_height_ = g_value_get_int(height_val);
        this->update_padding(this->video_window_);
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.reader." + shmpath_),
      [this]() { redraw_window(); },
      [this](bool status) {
        if (!status) redraw_window();
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
  install_gst_properties();
  return true;
}

bool GTKVideo::can_sink_caps(std::string caps) {
  return GstUtils::can_sink_caps("videoconvert", caps);
};

GstBusSyncReply GTKVideo::bus_sync(GstMessage* msg) {
  if (!gst_is_video_overlay_prepare_window_handle_message(msg)) return GST_BUS_PASS;
  GstVideoOverlay* overlay = GST_VIDEO_OVERLAY(GST_MESSAGE_SRC(msg));
  gst_video_overlay_set_window_handle(overlay, (guintptr) window_handle_);
  gst_message_unref(msg);
  return GST_BUS_DROP;
}

gboolean GTKVideo::button_event(GtkWidget* /*widget*/, GdkEventButton* event, gpointer data) {
  GTKVideo* context = static_cast<GTKVideo*>(data);
  if (-1 == context->vid_width_) return TRUE;

  int x, y;
  GdkModifierType state;
  gdk_window_get_device_position(event->window, context->device_, &x, &y, &state);
  context->write_mouse_info_to_shmdata(x, y, state);
  return TRUE;
}

gboolean GTKVideo::motion_notify_event(GtkWidget* /*widget*/,
                                       GdkEventMotion* event,
                                       gpointer data) {
  GTKVideo* context = static_cast<GTKVideo*>(data);
  if (-1 == context->vid_width_) return TRUE;
  int x, y;
  GdkModifierType state;
  if (event->is_hint)
    gdk_window_get_device_position(event->window, context->device_, &x, &y, &state);
  else {
    x = event->x;
    y = event->y;
  }
  context->write_mouse_info_to_shmdata(x, y, state);
  return TRUE;
}

void GTKVideo::widget_getsize(GtkWidget* widget, GtkAllocation* /*allocation*/, void* data) {
  GTKVideo* context = static_cast<GTKVideo*>(data);
  context->horizontal_padding_ = 0;
  context->vertical_padding_ = 0;
  GtkAllocation allocation;
  gtk_widget_get_allocation(widget, &allocation);
  context->drawed_video_width_ = allocation.width;
  context->drawed_video_height_ = allocation.height;
  context->update_padding(widget);
  window_update_size(context);
}

void GTKVideo::widget_has_focus(GtkWidget* /*widget*/, GtkStateFlags flags, void* data) {
  GTKVideo* context = static_cast<GTKVideo*>(data);
  if (flags & GTK_STATE_FLAG_BACKDROP) {
    context->graft_tree(".focused.", InfoTree::make("true"));
  } else {
    context->graft_tree(".focused", InfoTree::make("false"));
  }
}

gboolean GTKVideo::window_update_position(void* data) {
  GTKVideo* context = static_cast<GTKVideo*>(data);
  auto lock_position_x = context->pmanage<MPtr(&PContainer::get_lock)>(context->position_x_id_);
  auto lock_position_y = context->pmanage<MPtr(&PContainer::get_lock)>(context->position_y_id_);

  int position_x = context->position_x_;
  int position_y = context->position_y_;

  if (gtk_window_get_decorated(GTK_WINDOW(context->main_window_))) {
    gtk_window_get_position(
        GTK_WINDOW(context->main_window_), &context->position_x_, &context->position_y_);
  } else {
    gdk_window_get_origin(
        gtk_widget_get_window(context->main_window_), &context->position_x_, &context->position_y_);
  }

  if (position_x != context->position_x_)
    context->pmanage<MPtr(&PContainer::notify)>(context->position_x_id_);
  if (position_y != context->position_y_)
    context->pmanage<MPtr(&PContainer::notify)>(context->position_y_id_);

  return FALSE;
}

gboolean GTKVideo::window_update_size(void* data) {
  GTKVideo* context = static_cast<GTKVideo*>(data);
  if (!context->is_valid_) return FALSE;
  auto lock_width = context->pmanage<MPtr(&PContainer::get_lock)>(context->width_id_);
  auto lock_height = context->pmanage<MPtr(&PContainer::get_lock)>(context->height_id_);

  gtk_window_get_size(GTK_WINDOW(context->main_window_), &context->width_, &context->height_);

  context->pmanage<MPtr(&PContainer::notify)>(context->width_id_);
  context->pmanage<MPtr(&PContainer::notify)>(context->height_id_);
  return FALSE;
}

void GTKVideo::update_padding(GtkWidget* widget) {
  if (-1 == drawed_video_width_ || -1 == vid_width_) return;
  GtkAllocation allocation;
  gtk_widget_get_allocation(widget, &allocation);
  if (allocation.width / allocation.height > vid_width_ / vid_height_) {
    // padding is horizontal
    drawed_video_width_ = allocation.height * vid_width_ / vid_height_;
    horizontal_padding_ = (allocation.width - drawed_video_width_) / 2;
  } else {
    // padding is vertical
    drawed_video_height_ = allocation.width * vid_height_ / vid_width_;
    vertical_padding_ = (allocation.height - drawed_video_height_) / 2;
  }
}

void GTKVideo::write_mouse_info_to_shmdata(int x, int y, const GdkModifierType& state) {
  if (!mouse_shm_.get()) return;
  int vid_x = (x - horizontal_padding_) * 100000. / drawed_video_width_;
  int vid_y = (y - vertical_padding_) * 100000. / drawed_video_height_;
  if (0 > vid_x || 100000 < vid_x || 0 > vid_y || 100000 < vid_y)
    return;  // pointer is out of the video
  int button_mask = 0;
  if (state & GDK_BUTTON1_MASK) button_mask = 1;
  if (state & GDK_BUTTON2_MASK) button_mask += 2;
  if (state & GDK_BUTTON3_MASK) button_mask += 4;
  auto mouse_event = MouseEvent(vid_x, vid_y, button_mask);
  mouse_shm_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(&mouse_event, sizeof(MouseEvent));
  mouse_shm_->bytes_written(sizeof(MouseEvent));
}

void GTKVideo::install_gst_properties() {
  pmanage<MPtr(&PContainer::push)>("method",
                                   GPropToProp::to_prop(G_OBJECT(videoflip_.get_raw()), "method"));
  videoflip_.register_notify_on_property_change("method", [this]() { this->redraw_window(); });
  pmanage<MPtr(&PContainer::push)>("gamma",
                                   GPropToProp::to_prop(G_OBJECT(gamma_.get_raw()), "gamma"));
  pmanage<MPtr(&PContainer::push)>(
      "contrast", GPropToProp::to_prop(G_OBJECT(videobalance_.get_raw()), "contrast"));
  pmanage<MPtr(&PContainer::push)>(
      "brightness", GPropToProp::to_prop(G_OBJECT(videobalance_.get_raw()), "brightness"));
  pmanage<MPtr(&PContainer::push)>("hue",
                                   GPropToProp::to_prop(G_OBJECT(videobalance_.get_raw()), "hue"));
  pmanage<MPtr(&PContainer::push)>(
      "saturation", GPropToProp::to_prop(G_OBJECT(videobalance_.get_raw()), "saturation"));
}

void GTKVideo::remove_gst_properties() {
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("method"));
  videoflip_.unregister_notify_on_property_change("method");
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("gamma"));
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("contrast"));
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("brightness"));
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("hue"));
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("saturation"));
}

gboolean GTKVideo::set_title(void* user_data) {
  GTKVideo* context = static_cast<GTKVideo*>(user_data);
  gtk_window_set_title(GTK_WINDOW(context->main_window_), context->title_.c_str());
  return FALSE;
}

gboolean GTKVideo::set_geometry(void* user_data) {
  GTKVideo* context = static_cast<GTKVideo*>(user_data);

  // Only update position if the decoration is changing, otherwise the position
  // won't be settable by the property. We need to update it in this case
  // because the position reference differs with or without decoration.
  if (gtk_window_get_decorated(GTK_WINDOW(context->main_window_)) != context->decorated_) {
    window_update_position(context);
    gtk_window_set_decorated(GTK_WINDOW(context->main_window_), context->decorated_);
  }

  gtk_window_set_keep_above(GTK_WINDOW(context->main_window_), context->always_on_top_);

  auto lock_width = context->pmanage<MPtr(&PContainer::get_lock)>(context->width_id_);
  auto lock_height = context->pmanage<MPtr(&PContainer::get_lock)>(context->height_id_);
  auto lock_position_x = context->pmanage<MPtr(&PContainer::get_lock)>(context->position_x_id_);
  auto lock_position_y = context->pmanage<MPtr(&PContainer::get_lock)>(context->position_y_id_);

  gtk_window_resize(GTK_WINDOW(context->main_window_), context->width_, context->height_);
  gtk_window_move(GTK_WINDOW(context->main_window_), context->position_x_, context->position_y_);
  return FALSE;
}

gboolean GTKVideo::set_fullscreen(void* user_data) {
  GTKVideo* context = static_cast<GTKVideo*>(user_data);
  if (context->is_fullscreen_ && context->main_window_ != nullptr) {
    gdk_window_set_cursor(gtk_widget_get_window(context->video_window_), context->blank_cursor_);
    gtk_window_fullscreen(GTK_WINDOW(context->main_window_));
  } else if (context->main_window_ != nullptr) {
    gdk_window_set_cursor(gtk_widget_get_window(context->video_window_), nullptr);
    gtk_window_unfullscreen(GTK_WINDOW(context->main_window_));
  }
  return FALSE;
}

void GTKVideo::redraw_window() { gtk_widget_queue_draw(main_window_); }

}  // namespace switcher
