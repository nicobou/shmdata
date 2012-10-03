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


#ifndef __SWITCHER_RTPSESSION_H__
#define __SWITCHER_RTPSESSION_H__

#include "switcher/segment.h"
#include <gst/gst.h>
#include <memory>

namespace switcher
{

  class RtpSession : public Segment
  {
  public:
    typedef std::tr1::shared_ptr<RtpSession> ptr;
    RtpSession ();

    static BaseEntityDocumentation get_documentation ();

    static void attach_data_stream(ShmdataReader *caller, void *rtpsession_instance); //will be call by shmdata reader
    bool add_data_stream (std::string shmdata_socket_path);
    //wrapper for registering function
    static gboolean add_data_stream_wrapped (gpointer shmdata_socket_path, gpointer user_data);
    
  private:
    static BaseEntityDocumentation doc_;
    GstElement *rtpsession_;

    
    // //other signals (to emit to the rtpbin)
    /* clear-pt-map */
    /* get-internal-session */
    /* request_pt_map  */
    /* reset_sync  */
    /* payload_type_change */
    
    //internal rtpbin signals
     static void on_bye_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
     static void on_bye_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
     static void on_new_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
     static void on_npt_stop (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
     static void on_sender_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
     static void on_ssrc_active  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
     static void on_ssrc_collision (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
     static void on_ssrc_sdes  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
     static void on_ssrc_validated (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);  
     static void on_timeout  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
  };

}  // end of namespace

#endif // ifndef
