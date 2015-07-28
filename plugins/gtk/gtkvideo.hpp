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

#ifndef __SWITCHER_GTK_VIDEO_H__
#define __SWITCHER_GTK_VIDEO_H__

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/custom-property-helper.hpp"

#ifdef HAVE_CONFIG_H
#include "../../config.h"
#endif

#include <gtk/gtk.h>
#include <gdk/gdk.h>
#if defined(GDK_WINDOWING_X11)
#include <gdk/gdkx.h>
#elif defined(GDK_WINDOWING_WIN32)
#include <gdk/gdkwin32.h>
#elif defined(GDK_WINDOWING_QUARTZ)
#include <gdk/gdkquartz.h>
#endif

namespace switcher {
class GTKVideo: public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(GTKVideo);
  GTKVideo(const std::string &);
  ~GTKVideo();
  GTKVideo(const GTKVideo &) = delete;
  GTKVideo &operator=(const GTKVideo &) = delete;
  void toggle_fullscreen();

 private:
  static guint instances_counter_;
  static std::thread gtk_main_thread_;
  // registering connect/disconnect/can_sink_caps:
  ShmdataConnector shmcntr_;
  // gst pipeline:
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  // gtk:
  GdkDisplay *display_{nullptr};
  GtkWidget *main_window_{nullptr};
  GtkWidget *video_window_{nullptr};
  // shmsubscriber (publishing to the information-tree):
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{nullptr};
  std::string shmpath_{};
  // gst elements:
  UGstElem shmsrc_{"shmdatasrc"};
  UGstElem queue_{"queue"};
  UGstElem videoconvert_{"videoconvert"};
  UGstElem videoflip_{"videoflip"};
  UGstElem gamma_{"gamma"};
  UGstElem videobalance_{"videobalance"};
#if HAVE_OSX
  UGstElem xvimagesink_{"osxvideosink"};
#else
  UGstElem xvimagesink_{"xvimagesink"};
#endif
#if HAVE_OSX
  NSView *window_handle_{nullptr};
#else
  guintptr window_handle_{0};
#endif
  GdkCursor *blank_cursor_{nullptr};
  CustomPropertyHelper::ptr gtk_custom_props_{};
  GParamSpec *fullscreen_prop_spec_{nullptr};
  gboolean is_fullscreen_{FALSE};
  GParamSpec *title_prop_spec_{nullptr};
  gchar *title_{nullptr};
  GParamSpec *keyb_interaction_spec_{nullptr};
  gboolean keyb_interaction_{TRUE};
  std::mutex wait_window_mutex_{};
  std::condition_variable wait_window_cond_{};
  std::mutex window_destruction_mutex_{};
  std::condition_variable window_destruction_cond_{};
  // keyboard to shmdata
  std::unique_ptr<ShmdataWriter> keyb_shm_{nullptr};
  // mouse to shmdata (relative to the video, not the window)
  std::unique_ptr<ShmdataWriter> mouse_shm_{nullptr};
  // video width & height (for mouse shmdata)
  int vid_width_{-1};
  int vid_height_{-1};
  int horizontal_padding_{0};
  int vertical_padding_{0};
  float drawed_video_width_{-1};
  float drawed_video_height_{-1};
  
  bool init() final;
  bool remake_elements();
  bool on_shmdata_connect(const std::string &shmpath);
  bool on_shmdata_disconnect();
  bool can_sink_caps(std::string caps);
  GstBusSyncReply bus_sync(GstMessage *msg);
  static gboolean create_ui(void *user_data);
  static void realize_cb(GtkWidget *widget, void *user_data);
  static void delete_event_cb(GtkWidget *widget,
                              GdkEvent *event, void *user_data);
  static void gtk_main_loop_thread();
  static gboolean key_pressed_cb(GtkWidget *widget,
                                 GdkEventKey *event, gpointer data);
  static gboolean key_release_cb(GtkWidget */*widget */,
                                 GdkEventKey *event,
                                 gpointer data);
  static gboolean get_fullscreen(void *user_data);
  static void set_fullscreen(gboolean fullscreen, void *user_data);
  static gboolean on_destroy_event(GtkWidget *widget,
                                   GdkEvent *event, gpointer user_data);
  static void window_destroyed(gpointer data);
  static gboolean destroy_window(gpointer data);
  static void set_title(const gchar *value, void *user_data);
  static const gchar *get_title(void *user_data);
  static gboolean get_keyb_interaction(void *user_data);
  static void set_keyb_interaction(gboolean keyb_interaction, void *user_data);
  static gboolean button_event (GtkWidget *widget, GdkEventButton *event, gpointer data);
  static gboolean motion_notify_event (GtkWidget *widget, GdkEventMotion *event, gpointer data);
  static void widget_getsize(GtkWidget *widget, GtkAllocation *allocation, void *data);
  void update_padding(GtkWidget *widget);
  void write_mouse_info_to_shmdata(int x, int y, const GdkModifierType &state);
  
  struct KeybEvent{
    KeybEvent (guint32 keyval, guint32 down) : keyval_(keyval), down_(down){}
    guint32 keyval_{0};
    guint32 down_{0};
  };

  struct MouseEvent{
    MouseEvent (guint32 x, guint32 y, guint32 button_mask):
        x_(x), y_(y), button_mask_(button_mask){}
    guint32 x_{0};
    guint32 y_{0};
    guint32 button_mask_{0};
  };
};

SWITCHER_DECLARE_PLUGIN(GTKVideo);

}  // namespace switcher
#endif
