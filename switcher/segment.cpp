/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "switcher/segment.h"
//TODO remove this include
#include "switcher/video-sink.h"

namespace switcher
{

  Segment::Segment()
  {
    bin_ = gst_element_factory_make ("bin", NULL);

    //registering set_runtime
    std::vector<GType> set_runtime_arg_types;
    set_runtime_arg_types.push_back (G_TYPE_POINTER);
    register_method("set_runtime",(void *)&Segment::set_runtime_wrapped, set_runtime_arg_types,(gpointer)this);
    
    //registering connect
    std::vector<GType> connect_arg_types;
    connect_arg_types.push_back (G_TYPE_STRING);
    connect_arg_types.push_back (G_TYPE_POINTER);
    register_method("connect",(void *)&Segment::connect_wrapped, connect_arg_types,(gpointer)this);

  }
  
  void 
  Segment::set_runtime_wrapped (gpointer arg, gpointer user_data)
  {
     Runtime *runtime = static_cast<Runtime *>(arg);
     Segment *context = static_cast<Segment*>(user_data);
     
     if (runtime == NULL) 
       {
	 g_printerr ("Segment::set_runtime_wrapped Error: runtime is NULL\n");
	 return;
       }
     if (context == NULL) 
       {
	 g_printerr ("Segment::set_runtime_wrapped Error: segment is NULL\n");
	 return;
       }
     context->set_runtime(runtime);

     g_print ("%s is attached to runtime %s\n",context->get_name().c_str(),runtime->get_name().c_str());

  }
  
  void
  Segment::set_runtime (Runtime *runtime)
  {
    runtime_ = runtime;
    gst_bin_add (GST_BIN (runtime_->get_pipeline ()),bin_);
    gst_element_sync_state_with_parent (bin_);
  }
  
  GstElement *
  Segment::get_bin()
  {
    return bin_;
  }

  std::vector<std::string> 
  Segment::get_src_connectors ()
  {
    return connectors_.get_keys ();
  }

  
  gboolean
  Segment::connect_wrapped (gpointer connector_name, gpointer segment, gpointer user_data)
  {
    //std::string connector = static_cast<std::string>(connector_name);
    Segment *seg = static_cast<Segment*>(segment);
    Segment *context = static_cast<Segment*>(user_data);
    
    if (context->connect ((char *)connector_name,seg))
      return TRUE;
    else
      return FALSE;
  }

  bool
  Segment::connect (char *connector_name, Segment *segment)
  {
    VideoSink *xv = static_cast<VideoSink*>(segment);
    //gst_element_link (connectors_.lookup("video")->get_src_element(), xv->get_sink ());
    
    g_print ("connected\n");
    return true;
  }

  

}
