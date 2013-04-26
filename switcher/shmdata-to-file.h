/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
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


#ifndef __SWITCHER_SHMDATA_TO_FILE_H__
#define __SWITCHER_SHMDATA_TO_FILE_H__

#include <gst/gst.h>
#include <memory>
#include "segment.h"
#include "custom-property-helper.h"
 
namespace switcher
{

  class ShmdataToFile : public Segment
  {
  public:
    typedef std::shared_ptr<ShmdataToFile> ptr;
    ~ShmdataToFile ();

    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

    //local streams
    bool add_shmdata (std::string shmdata_socket_path,
			  std::string file_location);
    bool remove_shmdata (std::string shmdata_socket_path);
    
    static void set_recording (gboolean mute, void *user_data);
    static gboolean get_recording (void *user_data);

  private:
    //custom properties:
     CustomPropertyHelper::ptr custom_prop_;  
     GParamSpec *recording_param_; 
     gboolean recording_; 

     bool make_recorders ();
     bool clean_recorders ();
     StringMap <std::string> file_names_;
     StringMap <GstElement *> shmdata_recorders_;

     //wrapper for registering the data_stream functions
     static gboolean add_shmdata_wrapped (gpointer shmdata_socket_path, 
					  gpointer file_location,
					  gpointer user_data);
     static gboolean remove_shmdata_wrapped (gpointer shmdata_socket_path, 
					     gpointer user_data);
  };
}  // end of namespace

#endif // ifndef
