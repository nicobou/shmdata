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

#include "gtkvideo.h"
#include "switcher/gst-utils.h"
#include "switcher/quiddity-command.h"
#include <gst/gst.h>
#include <gdk/gdkkeysyms.h>
#include <gdk/gdkcursor.h>

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GTKVideo, 
				       "Video Display",
				       "video sink", 
				       "Video window with fullscreen",
				       "GPL",
				       "gtkvideosink",
				       "Nicolas Bouillot");
  
  guint GTKVideo::instances_counter_ = 0;

  bool
  GTKVideo::init ()
  {
    if (!GstUtils::make_element ("xvimagesink", &xvimagesink_))
      return false;
    
    if (!gtk_init_check (NULL, NULL))
      {
	g_debug ("GTKVideo::init, cannot init gtk");
	return false;
      }

    //set the name before registering properties
    set_name (gst_element_get_name (xvimagesink_));
    g_object_set (G_OBJECT (xvimagesink_), "sync", FALSE, NULL);

    on_error_command_ = new QuiddityCommand ();
    on_error_command_->id_ = QuiddityCommand::remove;
    on_error_command_->add_arg (get_nick_name ());

    g_object_set_data (G_OBJECT (xvimagesink_), 
     		       "on-error-command",
     		       (gpointer)on_error_command_);
    
    set_sink_element (xvimagesink_);

    set_on_first_data_hook (GTKVideo::create_ui,this);

    main_window_ = NULL;  
    video_window_ = NULL; 
    is_fullscreen_ = FALSE;
    if (instances_counter_ == 0)
      g_thread_new ("GTKMainLoopThread", GThreadFunc(gtk_main_loop_thread), this);
    instances_counter_++;
        
    custom_props_.reset (new CustomPropertyHelper ());
    fullscreen_prop_spec_ = 
      custom_props_->make_boolean_property ("fullscreen", 
					    "Enable/Disable Fullscreen",
					    (gboolean)FALSE,
					    (GParamFlags) G_PARAM_READWRITE,
					    GTKVideo::set_fullscreen,
					    GTKVideo::get_fullscreen,
					    this);
    register_property_by_pspec (custom_props_->get_gobject (), 
				fullscreen_prop_spec_, 
				"fullscreen",
				"Fullscreen");

    g_object_set (G_OBJECT (xvimagesink_),
		  "force-aspect-ratio", TRUE,
		  "draw-borders", TRUE,
		  NULL);

    
    
    blank_cursor_ = gdk_cursor_new(GDK_BLANK_CURSOR);
    return true;
  }
  
  gpointer
  GTKVideo::gtk_main_loop_thread (gpointer user_data)
  {
    g_debug ("GTKVideo::gtk_main_loop_thread starting");
    gtk_main ();
    return NULL;
  }


  gboolean 
  GTKVideo::key_pressed_cb(GtkWidget *widget, GdkEventKey *event, gpointer data)
{
    GTKVideo *context = static_cast<GTKVideo *>(data);
    QuiddityManager_Impl::ptr manager;

    switch (event->keyval)
    {
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
      manager = context->manager_impl_.lock ();
      if ((bool) manager)
	manager->remove (context->get_nick_name ());
      else
	g_debug ("GTKVideo::key_pressed_cb q pressed, closing window");
      break;
    default:
      break;
    }

    return TRUE;
}

  GTKVideo::GTKVideo ()
  {
  }

  GTKVideo::~GTKVideo ()
  {
    reset_bin ();
    
    // should destroy child widgets too
    if (main_window_ != NULL)
      gtk_widget_destroy (main_window_);
    
    gdk_cursor_destroy (blank_cursor_);
    
    if (instances_counter_ == 0)
      {
     	g_debug ("GTKVideo::~GTKVideo invoking gtk_main_quit");
     	gtk_main_quit ();
      }
    
     if (on_error_command_ != NULL)
       delete on_error_command_;
     instances_counter_ --;
  }
  
  void 
  GTKVideo::realize_cb (GtkWidget *widget, void *user_data) {

     GTKVideo *context = static_cast <GTKVideo *> (user_data);
     GdkWindow *window = gtk_widget_get_window (widget);
    
     if (!gdk_window_ensure_native (window))
       g_debug ("Couldn't create native window needed for GstXOverlay!");
    
     /* Retrieve window handler from GDK */
 #if defined (GDK_WINDOWING_WIN32)
     context->window_handle_ = (guintptr)GDK_WINDOW_HWND (window);
 #elif defined (GDK_WINDOWING_QUARTZ)
     context->window_handle_ = gdk_quartz_window_get_nsview (window);
 #elif defined (GDK_WINDOWING_X11)
     context->window_handle_ = GDK_WINDOW_XID (window);
 #endif

     //Pass it to xvimagesink, which implements XOverlay. Will be done in at bus call
     g_object_set_data (G_OBJECT (context->xvimagesink_), 
			"window-handle",
			(gpointer)&context->window_handle_);
  }
  

  /* This function is called when the main window is closed */
  void 
  GTKVideo::delete_event_cb (GtkWidget *widget, GdkEvent *event, void *user_data) 
  {
    GTKVideo *context = static_cast <GTKVideo *> (user_data);
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock ();
    if ((bool) manager)
      manager->remove (context->get_nick_name ());
    else
      g_debug ("GTKVideo::delete_event_cb cannot remove quiddity");
  }
  

  gboolean 
  GTKVideo::expose_cb (GtkWidget *widget, GdkEventExpose *event, void *user_data) 
  {
    return FALSE;
  }

  void 
  GTKVideo::create_ui (ShmdataReader *caller, void *user_data) 
  {
    GTKVideo *context = static_cast <GTKVideo *> (user_data);
   
    
    context->main_window_ = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    g_signal_connect (G_OBJECT (context->main_window_), 
		      "delete-event", G_CALLBACK (delete_event_cb), context);
    
    context->video_window_ = gtk_drawing_area_new ();
    gtk_widget_set_double_buffered (context->video_window_, FALSE);
    g_signal_connect (context->video_window_, "realize", G_CALLBACK (realize_cb), context);
    g_signal_connect (context->video_window_, "expose_event", G_CALLBACK (expose_cb), context);
    
    gtk_container_add (GTK_CONTAINER (context->main_window_), context->video_window_);
    gtk_window_set_default_size (GTK_WINDOW (context->main_window_), 640, 480);
    
    gtk_widget_set_events(context->main_window_, GDK_KEY_PRESS_MASK);
    g_signal_connect(G_OBJECT(context->main_window_), "key-press-event",
		     G_CALLBACK(GTKVideo::key_pressed_cb), context);
    
    gtk_widget_show_all (context->main_window_);
    
    context->set_fullscreen (context->is_fullscreen_, context);
  }
  

  void 
  GTKVideo::toggle_fullscreen()
  {
    if (is_fullscreen_)
      set_fullscreen (FALSE, this);
    else
      set_fullscreen (TRUE, this);
  }
  
  gboolean 
  GTKVideo::get_fullscreen (void *user_data)
  {
    GTKVideo *context = static_cast<GTKVideo *> (user_data);
    return context->is_fullscreen_;
  }

  void 
  GTKVideo::set_fullscreen (gboolean fullscreen, void *user_data)
  {
    GTKVideo *context = static_cast<GTKVideo *> (user_data);
    
    if (fullscreen)
      {
	if (context->main_window_ != NULL)
	  {
	    gdk_window_set_cursor(GDK_WINDOW(context->video_window_->window), context->blank_cursor_);
	    gtk_window_fullscreen(GTK_WINDOW(context->main_window_));
	    //gtk_widget_set_size_request (context->video_window_,1920,1080);
	  }
	context->is_fullscreen_ = TRUE;
      }
    else
      {
	if (context->main_window_ != NULL)
	  {
	    gdk_window_set_cursor(GDK_WINDOW(context->video_window_->window), NULL);
	    gtk_window_unfullscreen(GTK_WINDOW(context->main_window_));
	  }
	context->is_fullscreen_ = FALSE;
      }
    context->custom_props_->notify_property_changed (context->fullscreen_prop_spec_);
  }

}
