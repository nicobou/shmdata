/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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


#ifndef __SWITCHER_FILE_SDP_H__
#define __SWITCHER_FILE_SDP_H__

#include "base-source.h"
#include "gst-element-cleaner.h"
#include <memory>

namespace switcher
{

  class FileSDP : public BaseSource, public GstElementCleaner
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(FileSDP);
    FileSDP ();
    FileSDP (const FileSDP &) = delete;
    FileSDP &operator= (const FileSDP &) = delete;
    bool to_shmdata (std::string uri);

  private: 
   GstElement *filesrc_;
   GstElement *sdpdemux_;
   int media_counter_;
   bool init_segment ();
   static void pad_added_cb (GstElement* object, GstPad* pad, gpointer user_data);
   static gboolean to_shmdata_wrapped (gpointer uri, gpointer user_data);
   static void no_more_pads_cb (GstElement* object, gpointer user_data);
  };

}  // end of namespace

#endif // ifndef
