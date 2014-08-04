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

#include "gtkvideo.h"
#include "switcher/gst-utils.h"
#include "switcher/quiddity-command.h"
#include <gst/gst.h>
#include <gdk/gdkkeysyms.h>
#include <gdk/gdkcursor.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

namespace switcher
{
  SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GTKVideo, 
				       "Video Display",
				       "video sink", 
				       "Video window with fullscreen",
				       "LGPL",
				       "gtkvideosink",
				       "Nicolas Bouillot");
  
  guint GTKVideo::instances_counter_ = 0;
  std::thread GTKVideo::gtk_main_thread_ {};

  bool
  GTKVideo::init_gpipe ()
  {
    if (!GstUtils::make_element ("bin",&sink_bin_))
      return false;
    if (!GstUtils::make_element ("queue",&queue_))
      return false;
    if (!GstUtils::make_element ("ffmpegcolorspace", &ffmpegcolorspace_))
      return false;
    if (!GstUtils::make_element ("videoflip", &videoflip_))
      g_warning ("video fliping not available");
    if (!GstUtils::make_element ("gamma", &gamma_))
      g_warning ("gamma control not available");
    if (!GstUtils::make_element ("videobalance", &videobalance_))
      g_warning ("video balance not available");
#if HAVE_OSX
    if (!GstUtils::make_element ("osxvideosink", &xvimagesink_))
      return false;
#else
    if (!GstUtils::make_element ("xvimagesink", &xvimagesink_))
      return false;
#endif
    gst_bin_add_many (GST_BIN (sink_bin_),
		      queue_,
		      ffmpegcolorspace_,
		      xvimagesink_,
		      nullptr);
    gst_element_link (queue_, ffmpegcolorspace_);
    GstElement *next_to_connect_with = ffmpegcolorspace_;
    if (nullptr != videoflip_)
      {
	gst_bin_add (GST_BIN (sink_bin_), videoflip_);
	gst_element_link (next_to_connect_with, videoflip_);
	next_to_connect_with = videoflip_;
      }
    if (nullptr != gamma_)
      {
	gst_bin_add (GST_BIN (sink_bin_), gamma_);
	gst_element_link (next_to_connect_with, gamma_);
	next_to_connect_with = gamma_;
      }
    if (nullptr != videobalance_)
      {
	gst_bin_add (GST_BIN (sink_bin_), videobalance_);
	gst_element_link (next_to_connect_with, videobalance_);
	next_to_connect_with = videobalance_;
      }
    gst_element_link (next_to_connect_with, xvimagesink_);

    GstPad *sink_pad = gst_element_get_static_pad (queue_, 
						   "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new (nullptr, sink_pad);
    gst_pad_set_active(ghost_sinkpad, TRUE);
    gst_element_add_pad (sink_bin_, ghost_sinkpad); 
    gst_object_unref (sink_pad);
    
    g_object_set (G_OBJECT (xvimagesink_), 
		  "sync", FALSE, 
		  "qos", FALSE,
		  nullptr);
    //on_error_command_ = new QuiddityCommand ();
    on_error_command_->id_ = QuiddityCommand::remove;
    on_error_command_->add_arg (get_nick_name ());
    g_object_set_data (G_OBJECT (xvimagesink_), 
     		       "on-error-command",
     		       (gpointer)on_error_command_);
    set_sink_element (sink_bin_);
    is_fullscreen_ = FALSE;

    if (instances_counter_ == 0)
      {
	if (0 == gtk_main_level ()) 
	  {
	    gtk_main_thread_ = std::thread (&GTKVideo::gtk_main_loop_thread);
	    gtk_main_thread_.detach ();
	  }
	else 
	  g_debug ("gtkvideosink: GTK main loop detected, using it");
      }
    instances_counter_++;

    fullscreen_prop_spec_ = 
      gtk_custom_props_->make_boolean_property ("fullscreen", 
    					    "Enable/Disable Fullscreen",
    					    (gboolean)FALSE,
    					    (GParamFlags) G_PARAM_READWRITE,
    					    GTKVideo::set_fullscreen,
    					    GTKVideo::get_fullscreen,
    					    this);
    install_property_by_pspec (gtk_custom_props_->get_gobject (), 
			       fullscreen_prop_spec_, 
			       "fullscreen",
			       "Fullscreen");
    g_object_set (G_OBJECT (xvimagesink_),
    		  "force-aspect-ratio", TRUE,
    		  "draw-borders", TRUE,
    		  nullptr);

    if (nullptr != videoflip_)
      install_property (G_OBJECT (videoflip_),
			"method",
			"method", 
			"Flip Method");
    if (nullptr != gamma_)
      install_property (G_OBJECT (gamma_),
			"gamma",
			"gamma", 
			"Gamma");
    if (nullptr != videobalance_)
      {
	install_property (G_OBJECT (videobalance_),
			  "contrast",
			  "contrast", 
			  "Contrast");
	install_property (G_OBJECT (videobalance_),
			  "brightness",
			  "brightness", 
			  "Brightness");
	install_property (G_OBJECT (videobalance_),
			  "hue",
			  "hue", 
			  "Hue");
	install_property (G_OBJECT (videobalance_),
			  "saturation",
			  "saturation", 
			  "Saturation");
	title_ = g_strdup (get_nick_name ().c_str ());
	title_prop_spec_ = 
	  gtk_custom_props_->make_string_property ("title", 
					       "Window Title",
					       title_,
					       (GParamFlags) G_PARAM_READWRITE,
					       GTKVideo::set_title,
					       GTKVideo::get_title,
					       this);
	install_property_by_pspec (gtk_custom_props_->get_gobject (), 
				   title_prop_spec_, 
				   "title",
				   "Window Title");

      }

    std::unique_lock<std::mutex> lock (wait_window_mutex_);
    gtk_idle_add ((GtkFunction)create_ui,
    		  this);
    wait_window_cond_.wait (lock);
    if (nullptr == display_)
      return false;
    return true;
  }
  
  GTKVideo::GTKVideo () :
    display_ (nullptr),
    main_window_ (nullptr),
    video_window_ (nullptr),
    sink_bin_ (nullptr),
    queue_ (nullptr),
    ffmpegcolorspace_ (nullptr),
    videoflip_ (nullptr),
    gamma_(nullptr),
    videobalance_ (nullptr),
    xvimagesink_ (nullptr),
#if HAVE_OSX
    window_handle_ (nullptr),
#else
    window_handle_ (0),
#endif
    on_error_command_ (new QuiddityCommand ()),
    blank_cursor_ (nullptr),
    gtk_custom_props_ (new CustomPropertyHelper ()),
    fullscreen_prop_spec_ (nullptr),
    is_fullscreen_ (FALSE),
    title_prop_spec_ (nullptr),
    title_ (nullptr),
    wait_window_mutex_(),
    wait_window_cond_(),
    window_destruction_mutex_(),
    window_destruction_cond_()
  {}

  void
  GTKVideo::gtk_main_loop_thread ()
  {
    if (!gtk_init_check (nullptr, nullptr))
      {
    	g_debug ("GTKVideo::init, cannot init gtk");
      }
    g_debug ("GTKVideo::gtk_main_loop_thread starting");
    gtk_main ();
  }

  gboolean 
  GTKVideo::key_pressed_cb(GtkWidget */*widget*/, 
			   GdkEventKey *event, 
			   gpointer data)
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
  
  void  
  GTKVideo::window_destroyed (gpointer user_data)
  {
    GTKVideo *context = static_cast <GTKVideo *> (user_data);
    std::unique_lock<std::mutex> lock (context->window_destruction_mutex_); 
    context->window_destruction_cond_.notify_all ();
  }
  
  gboolean  
  GTKVideo::destroy_window (gpointer user_data)
  {
    GTKVideo *context = static_cast <GTKVideo *> (user_data);
    gtk_widget_destroy (GTK_WIDGET(context->main_window_));
    window_destroyed (context);
    return  FALSE;
  }

  GTKVideo::~GTKVideo ()
  {
    reset_bin ();
    g_idle_remove_by_data (this);
    if (nullptr != title_)
      g_free (title_);
    //destroy child widgets too
    if (main_window_ != nullptr && GTK_IS_WIDGET (main_window_))
      {
	std::unique_lock<std::mutex> lock (window_destruction_mutex_);
	// g_idle_add_full (G_PRIORITY_DEFAULT_IDLE,
	//   		  destroy_window, 
	//   		  this,
	//   		  window_destroyed);
	gtk_idle_add (destroy_window, this);
	window_destruction_cond_.wait (lock);
      }
    if (blank_cursor_ != nullptr)
      gdk_cursor_destroy (blank_cursor_);
    // instances_counter_ --;
    // if (instances_counter_ == 0)
    //   {
    // 	g_debug ("GTKVideo::~GTKVideo invoking gtk_main_quit");
    // 	gtk_main_quit ();
    //   }
    if (on_error_command_ != nullptr)
      delete on_error_command_;
  }
  
  
  void 
  GTKVideo::realize_cb (GtkWidget *widget, void *user_data) 
  {
    GTKVideo *context = static_cast <GTKVideo *> (user_data);
    GdkWindow *window = gtk_widget_get_window (widget);
    
    if (!gdk_window_ensure_native (window))
      g_debug ("Couldn't create native window needed for GstXOverlay!");

    gdk_threads_enter ();
    gdk_display_sync (context->display_);
    //gdk_error_trap_pop ();  

    /* Retrieve window handler from GDK */
#if defined (GDK_WINDOWING_WIN32)
    context->window_handle_ = (guintptr)GDK_WINDOW_HWND (window);
#elif defined (GDK_WINDOWING_QUARTZ)
    context->window_handle_ = gdk_quartz_window_get_nsview (window);
#elif defined (GDK_WINDOWING_X11)
    context->window_handle_ = GDK_WINDOW_XID (window);
#endif
    gdk_threads_leave ();
    std::unique_lock<std::mutex> lock (context->wait_window_mutex_);
    context->wait_window_cond_.notify_all ();
  }
  
  /* This function is called when the main window is closed */
  void 
  GTKVideo::delete_event_cb (GtkWidget */*widget*/, 
			     GdkEvent */*event*/, 
			     void *user_data) 
  {
    GTKVideo *context = static_cast <GTKVideo *> (user_data);
    
    context->reset_bin ();
    gtk_widget_destroy (context->main_window_);
    context->main_window_ = nullptr;
    QuiddityManager_Impl::ptr manager = context->manager_impl_.lock ();
    if ((bool) manager)
      manager->remove (context->get_nick_name ());
    else
      g_debug ("GTKVideo::delete_event_cb cannot remove quiddity");
  }
  
  gboolean 
  GTKVideo::create_ui (void *user_data) 
  {
    GTKVideo *context = static_cast <GTKVideo *> (user_data);
    context->display_ =  gdk_display_get_default ();
    if (nullptr == context->display_)
      {
	g_debug ("gtkvideo: no default display, cannot create window");
	std::unique_lock<std::mutex> lock (context->wait_window_mutex_);
	context->wait_window_cond_.notify_all ();
	return FALSE;
      }

    context->main_window_ = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    g_signal_connect (G_OBJECT (context->main_window_), 
     		      "delete-event", G_CALLBACK (delete_event_cb), context);
    context->video_window_ = gtk_drawing_area_new ();
    gtk_widget_set_double_buffered (context->video_window_, FALSE);
    GdkColor color;
    gdk_color_parse ("black", &color);
    gtk_widget_modify_bg(context->video_window_, 
     			 GTK_STATE_NORMAL, 
     			 &color);
    g_signal_connect (context->video_window_, "realize", G_CALLBACK (realize_cb), context);
    gtk_container_add (GTK_CONTAINER (context->main_window_), context->video_window_);
    gtk_window_set_default_size (GTK_WINDOW (context->main_window_), 640, 480);
    gtk_window_set_title (GTK_WINDOW (context->main_window_), context->title_);
    context->blank_cursor_ = gdk_cursor_new(GDK_BLANK_CURSOR);
    gtk_widget_set_events (context->main_window_, GDK_KEY_PRESS_MASK );
    g_signal_connect(G_OBJECT(context->main_window_), 
		     "key-press-event",
		     G_CALLBACK(GTKVideo::key_pressed_cb), 
		     context);
    gtk_widget_show_all ((GtkWidget *)context->main_window_);
    return FALSE;
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
     	if (context->main_window_ != nullptr)
     	  {
     	    gdk_window_set_cursor(GDK_WINDOW(context->video_window_->window), 
				  context->blank_cursor_);
     	    gtk_window_fullscreen(GTK_WINDOW(context->main_window_));
     	  }
     	context->is_fullscreen_ = TRUE;
      }
    else
      {
     	if (context->main_window_ != nullptr)
     	  {
     	    gdk_window_set_cursor(GDK_WINDOW(context->video_window_->window), nullptr);
     	    gtk_window_unfullscreen(GTK_WINDOW(context->main_window_));
     	  }
     	context->is_fullscreen_ = FALSE;
      }
    context->gtk_custom_props_->notify_property_changed (context->fullscreen_prop_spec_);
  }

  void 
  GTKVideo::on_shmdata_connect (std::string /*shmdata_sochet_path*/) 
  {
    gdk_threads_enter ();
    g_object_set_data (G_OBJECT (xvimagesink_), 
       		       "window-handle",
		       (gpointer)&window_handle_);
    gdk_threads_leave ();
  }

  void 
  GTKVideo::set_title (const gchar *value, void *user_data)
  {
    GTKVideo *context = static_cast <GTKVideo *> (user_data);
    if (nullptr != context->title_)
      g_free (context->title_);
    context->title_ = g_strdup (value);
    gtk_window_set_title (GTK_WINDOW (context->main_window_), context->title_);
    context->gtk_custom_props_->notify_property_changed (context->title_prop_spec_);
   }
  
  const gchar *
  GTKVideo::get_title (void *user_data)
  {
    GTKVideo *context = static_cast <GTKVideo *> (user_data);
    return context->title_;
  }

  bool 
  GTKVideo::can_sink_caps (std::string caps) 
  {
    return GstUtils::can_sink_caps ("ffmpegcolorspace",
				    caps);
  };

}
