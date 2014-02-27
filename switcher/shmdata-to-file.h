/*
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


#ifndef __SWITCHER_SHMDATA_TO_FILE_H__
#define __SWITCHER_SHMDATA_TO_FILE_H__

#include <gst/gst.h>
#include "segment.h"
#include "custom-property-helper.h"
#include <unordered_map>
 
namespace switcher
{

  class ShmdataToFile : public Segment
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(ShmdataToFile);
    ShmdataToFile ();
    ~ShmdataToFile ();
    ShmdataToFile (const ShmdataToFile &);
    ShmdataToFile &operator= (const ShmdataToFile &);

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

     bool init_segment ();
     bool make_recorders ();
     bool clean_recorders ();
     std::unordered_map <std::string, std::string> file_names_;
     std::unordered_map <std::string, GstElement *> shmdata_recorders_;

     //wrapper for registering the data_stream functions
     static gboolean add_shmdata_wrapped (gpointer shmdata_socket_path, 
					  gpointer file_location,
					  gpointer user_data);
     static gboolean remove_shmdata_wrapped (gpointer shmdata_socket_path, 
					     gpointer user_data);
  };
}  // end of namespace

#endif // ifndef
