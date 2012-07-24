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

namespace switcher
{

  Segment::Segment()
  {
    bin_ = gst_element_factory_make ("bin", NULL);

    std::vector<GType> arg_types;
    arg_types.push_back (G_TYPE_POINTER);
    register_method("set_runtime",(void *)&Segment::set_runtime_wrapped, arg_types,(gpointer)this);
  }
  
  void 
  Segment::set_runtime_wrapped (void *arg, gpointer user_data)
  {
    Segment *context = static_cast<Segment*>(user_data);
    Runtime::ptr runtime = *(static_cast<Runtime::ptr *>(arg));
    
    context->runtime_ = runtime;
    context->attach_to_runtime ();

    
    g_print ("coucou\n");
  }
  
  // void
  // Segment::set_runtime (Runtime::ptr runtime)
  // {
  //   runtime_ = runtime;
  //   attach_to_runtime ();
  // }
  
  void 
  Segment::attach_to_runtime ()
  {
    gst_bin_add (GST_BIN (runtime_->get_pipeline ()),bin_);
    gst_element_sync_state_with_parent (bin_);
  }
  
  GstElement *
  Segment::get_bin()
  {
    return bin_;
  }
}
