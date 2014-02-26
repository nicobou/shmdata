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

#include "switcher/video-sink.h"
#include "switcher/custom-property-helper.h"
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gtk/gtk.h>
#include <gdk/gdk.h>
#if defined (GDK_WINDOWING_X11)
#include <gdk/gdkx.h>
#elif defined (GDK_WINDOWING_WIN32)
#include <gdk/gdkwin32.h>
#elif defined (GDK_WINDOWING_QUARTZ)
#include <gdk/gdkquartz.h>
#endif

namespace switcher
{
  class GTKVideo : public VideoSink
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(GTKVideo);
    GTKVideo ();
    ~GTKVideo ();
    GTKVideo (const GTKVideo &) = delete;
    GTKVideo &operator= (const GTKVideo &) = delete;
    void toggle_fullscreen();
    
  private:
    static guint instances_counter_;
    static std::thread gtk_main_thread_;
    GdkDisplay *display_;
    GtkWidget *main_window_;  
    GtkWidget *video_window_; 
    GstElement *sink_bin_;
    GstElement *queue_;
    GstElement *ffmpegcolorspace_;
    GstElement *videoflip_;
    GstElement *gamma_;
    GstElement *videobalance_;
    GstElement *xvimagesink_;

#if HAVE_OSX
    NSView * window_handle_;
#else
    guintptr window_handle_;
#endif
    QuiddityCommand *on_error_command_; //for the runtime error handler
    GdkCursor *blank_cursor_;

    CustomPropertyHelper::ptr custom_props_;
    GParamSpec *fullscreen_prop_spec_;
    gboolean is_fullscreen_;
    GParamSpec *title_prop_spec_;
    gchar *title_;

    std::mutex wait_window_mutex_;
    std::condition_variable wait_window_cond_;

    std::mutex window_destruction_mutex_;
    std::condition_variable window_destruction_cond_;

    bool init_segment ();
    static gboolean create_ui (void *user_data);
    static void realize_cb (GtkWidget *widget, void *user_data);
    static void delete_event_cb (GtkWidget *widget, GdkEvent *event, void *user_data);
    static void gtk_main_loop_thread ();
    static gboolean key_pressed_cb(GtkWidget *widget, GdkEventKey *event, gpointer data);
    static gboolean get_fullscreen (void *user_data);
    static void set_fullscreen (gboolean fullscreen, void *user_data);
    static gboolean on_destroy_event (GtkWidget *widget,
				      GdkEvent  *event,
				      gpointer   user_data);
    static void window_destroyed (gpointer data);
    static gboolean destroy_window (gpointer data);
    static void set_title (const gchar *value, void *user_data);
    static const gchar *get_title (void *user_data);

    //implementation of parent
    void on_shmdata_connect (std::string shmdata_sochet_path);
  };

  SWITCHER_DECLARE_PLUGIN(GTKVideo);

}  // end of namespace

#endif // ifndef
