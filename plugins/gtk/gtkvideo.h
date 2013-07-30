/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
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

#include <gtk/gtk.h>
#include <gdk/gdk.h>
#if defined (GDK_WINDOWING_X11)
#include <gdk/gdkx.h>
#elif defined (GDK_WINDOWING_WIN32)
#include <gdk/gdkwin32.h>
#elif defined (GDK_WINDOWING_QUARTZ)
#include <gdk/gdkquartzwindow.h>
#endif

namespace switcher
{

  class GTKVideo : public VideoSink
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(GTKVideo);
    GTKVideo ();
    ~GTKVideo ();
    void toggle_fullscreen();
    
  private:
    static guint instances_counter_;
    GtkWidget *main_window_;  
    GtkWidget *video_window_; 
    GstElement *xvimagesink_;
    guintptr window_handle_;
    QuiddityCommand *on_error_command_; //for the runtime error handler
    GdkCursor *blank_cursor_;

    CustomPropertyHelper::ptr custom_props_;
    GParamSpec *fullscreen_prop_spec_;
    gboolean is_fullscreen_;

    static void create_ui (ShmdataReader *caller, void *user_data);
    static gboolean expose_cb (GtkWidget *widget, GdkEventExpose *event, void *user_data);
    static void realize_cb (GtkWidget *widget, void *user_data);
    static void delete_event_cb (GtkWidget *widget, GdkEvent *event, void *user_data);
    static gpointer gtk_main_loop_thread (gpointer user_data);
    static gboolean key_pressed_cb(GtkWidget *widget, GdkEventKey *event, gpointer data);
    static gboolean get_fullscreen (void *user_data);
    static void set_fullscreen (gboolean fullscreen, void *user_data);
  };

  SWITCHER_DECLARE_PLUGIN(GTKVideo);

}  // end of namespace

#endif // ifndef
