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
#include "switcher/single-pad-gst-sink.hpp"
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
class GTKVideo: public SinglePadGstSink {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(GTKVideo);
  GTKVideo();
  ~GTKVideo();
  GTKVideo(const GTKVideo &) = delete;
  GTKVideo &operator=(const GTKVideo &) = delete;
  void toggle_fullscreen();

 private:
  static guint instances_counter_;
  static std::thread gtk_main_thread_;
  GdkDisplay *display_{nullptr};
  GtkWidget *main_window_{nullptr};
  GtkWidget *video_window_{nullptr};
  GstElement *sink_bin_{nullptr};
  GstElement *queue_{nullptr};
  GstElement *ffmpegcolorspace_{nullptr};
  GstElement *videoflip_{nullptr};
  GstElement *gamma_{nullptr};
  GstElement *videobalance_{nullptr};
  GstElement *xvimagesink_{nullptr};

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

  std::mutex wait_window_mutex_{};
  std::condition_variable wait_window_cond_{};

  std::mutex window_destruction_mutex_{};
  std::condition_variable window_destruction_cond_{};

  bool init_gpipe() final;
  void on_shmdata_connect(std::string shmdata_sochet_path) final;
  bool can_sink_caps(std::string caps) final;

  static gboolean create_ui(void *user_data);
  static void realize_cb(GtkWidget *widget, void *user_data);
  static void delete_event_cb(GtkWidget *widget,
                              GdkEvent *event, void *user_data);
  static void gtk_main_loop_thread();
  static gboolean key_pressed_cb(GtkWidget *widget,
                                 GdkEventKey *event, gpointer data);
  static gboolean get_fullscreen(void *user_data);
  static void set_fullscreen(gboolean fullscreen, void *user_data);
  static gboolean on_destroy_event(GtkWidget *widget,
                                   GdkEvent *event, gpointer user_data);
  static void window_destroyed(gpointer data);
  static gboolean destroy_window(gpointer data);
  static void set_title(const gchar *value, void *user_data);
  static const gchar *get_title(void *user_data);
};

SWITCHER_DECLARE_PLUGIN(GTKVideo);

}  // namespace switcher
#endif
