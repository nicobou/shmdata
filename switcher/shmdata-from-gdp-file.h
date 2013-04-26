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


#ifndef __SWITCHER_SHMDATA_FROM_GDP_FILE_H__
#define __SWITCHER_SHMDATA_FROM_GDP_FILE_H__

#include <gst/gst.h>
#include <memory>
#include "segment.h"
#include "custom-property-helper.h"
#include "quiddity-manager.h"
 
namespace switcher
{

  class ShmdataFromGDPFile : public Quiddity
  {
  public:
    typedef std::shared_ptr<ShmdataFromGDPFile> ptr;
    ~ShmdataFromGDPFile ();

    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

    //local streams
    bool add_file (std::string shmwriter_path,
		   std::string file_path);
    bool remove_file (std::string shmwriter_path);

    static void set_playing (gboolean playing, void *user_data);
    static gboolean get_playing (void *user_data);
    static void rewind (gpointer user_data);
    
  private:
    //custom properties:
     CustomPropertyHelper::ptr custom_prop_;  
     GParamSpec *playing_param_; 
     gboolean playing_; 
    
     bool make_players ();
     bool clean_players ();
     StringMap <std::string> shmdata_names_;
     QuiddityManager::ptr manager_;

     static gboolean event_probe_cb (GstPad *pad, GstEvent * event, gpointer user_data);
     //wrappers
     static gboolean add_file_wrapped (gpointer file_path,
				       gpointer shmdata_socket_path, 
				       gpointer user_data);
     static gboolean remove_file_wrapped (gpointer file_path, 
					  gpointer user_data);
  };
}  // end of namespace

#endif // ifndef
