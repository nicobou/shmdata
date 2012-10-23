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
#include <gst/sdp/gstsdpmessage.h>
#include <memory>
#include <switcher/quiddity-manager.h>

namespace switcher
{

  class RtpSession : public Segment
  {
  public:
    typedef std::tr1::shared_ptr<RtpSession> ptr;
    RtpSession ();
    RtpSession (QuiddityLifeManager::ptr life_manager);
    static QuiddityDocumentation get_documentation ();

    //local streams
    bool add_data_stream (std::string shmdata_socket_path);
    bool remove_data_stream (std::string shmdata_socket_path);
    
    //remote dest
    bool add_unicast_udp_dest (std::string shmdata_socket_path, std::string host, std::string port);

    //wrapper for registering the data_stream functions
    static gboolean add_data_stream_wrapped (gpointer shmdata_socket_path, 
					     gpointer user_data);
    static gboolean remove_data_stream_wrapped (gpointer shmdata_socket_path, 
						gpointer user_data);
    static gboolean add_unicast_udp_dest_wrapped (gpointer shmdata_name, 
						  gpointer host, 
						  gpointer port, 
						  gpointer user_data);
    //will be call by shmdata reader
    static void attach_data_stream(ShmdataReader *caller, void *rtpsession_instance); 
    
  private:
    void make_rtpsession ();
    GstElement *rtpsession_;
    //a counter used for setting id of internal streams
    guint next_id_;
    //QuiddityManager::ptr internal_manager_;

    //local streams
    StringMap <std::string> internal_id_; //maps shmdata path with internal id 
    StringMap <std::string> rtp_id_; //maps shmdata path with rtp id
    StringMap <QuiddityManager::ptr> quiddity_managers_;
    StringMap <GstElementCleaner::ptr> funnels_; //maps internal id with funnel cleaner

    StringMap<ShmdataWriter::ptr> internal_shmdata_writers_;
    StringMap<ShmdataReader::ptr> internal_shmdata_readers_;

    static void make_data_stream_available (GstElement* typefind, 
					    guint probability, 
					    GstCaps *caps, 
					    gpointer user_data);
    static gboolean sink_factory_filter (GstPluginFeature * feature, gpointer data);
    static gint sink_compare_ranks (GstPluginFeature *f1, GstPluginFeature *f2);

    //sdp
    GstSDPMessage *sdp_description_;
    void make_sdp_init ();

    //quiddity
    static QuiddityDocumentation doc_;

    //internal rtpbin signals
    static void on_bye_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
    static void on_bye_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
    static void on_new_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
    static void on_npt_stop (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
    static void on_sender_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
    static void on_ssrc_active (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
    static void on_ssrc_collision (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
    static void on_ssrc_sdes (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
    static void on_ssrc_validated (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data);  
    static void on_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data); 
    static void on_pad_added (GstElement *gstelement, GstPad *new_pad, gpointer user_data);
    static void on_pad_removed (GstElement *gstelement, GstPad *new_pad, gpointer user_data);
    static void on_no_more_pad (GstElement *gstelement, gpointer user_data);
  };
}  // end of namespace

#endif // ifndef
