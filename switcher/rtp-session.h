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


#ifndef __SWITCHER_RTPSESSION_H__
#define __SWITCHER_RTPSESSION_H__

#include <gst/gst.h>
#include <gst/sdp/gstsdpmessage.h>
#include <memory>
#include "segment.h"
#include "quiddity-manager.h"
#include "rtp-destination.h"
#include "custom-property-helper.h"
 
namespace switcher
{

  class RtpSession : public Segment
  {
  public:
    typedef std::shared_ptr<RtpSession> ptr;
    ~RtpSession ();

    bool init ();
    QuiddityDocumentation get_documentation ();
    static QuiddityDocumentation doc_;

    //local streams
    bool add_data_stream (std::string shmdata_socket_path);
    bool remove_data_stream (std::string shmdata_socket_path);
    
    //remote dest (using user defined "nick_name")
    bool add_destination (std::string dest_name,
			  std::string host_name);
    bool remove_destination (std::string dest_name);
    static gchar *get_destinations_json (void *user_data);

    //sending
    bool add_udp_stream_to_dest (std::string shmdata_socket_path, 
				 std::string dest_name, 
				 std::string port);
    bool remove_udp_stream_to_dest (std::string shmdata_socket_path, 
				    std::string dest_name);
    bool write_sdp_file (std::string dest_name);

 
    //wrapper for registering the data_stream functions
    static gboolean add_data_stream_wrapped (gpointer shmdata_socket_path, 
					     gpointer user_data);
    static gboolean remove_data_stream_wrapped (gpointer shmdata_socket_path, 
						gpointer user_data);
    static gboolean add_destination_wrapped (gpointer desst_name,
					     gpointer host_name,
					     gpointer user_data);
    static gboolean remove_destination_wrapped (gpointer nick_name, 
     						gpointer user_data); 
    static gboolean add_udp_stream_to_dest_wrapped (gpointer shmdata_name, 
						    gpointer dest_name, 
						    gpointer port, 
						    gpointer user_data);
    static gboolean remove_udp_stream_to_dest_wrapped (gpointer shmdata_socket_path, 
						       gpointer dest_name, 
						       gpointer user_data);
    static gboolean write_sdp_file_wrapped (gpointer nick_name,
					    gpointer user_data);
    
    //will be called by shmdata reader
    static void attach_data_stream(ShmdataReader *caller, void *rtpsession_instance); 
    
  private:
    GstElement *rtpsession_;
    //a counter used for setting id of internal streams
    guint next_id_;

    //custom properties:
    CustomPropertyHelper::ptr custom_props_; 
    GParamSpec *destination_description_json_;
    gchar *destinations_json_;

    //local streams
    StringMap <std::string> internal_id_; //maps shmdata path with internal id 
    StringMap <std::string> rtp_ids_; //maps shmdata path with rtp id
    StringMap <QuiddityManager::ptr> quiddity_managers_;
    StringMap <GstElementCleaner::ptr> funnels_; //maps internal id with funnel cleaner

    StringMap<ShmdataWriter::ptr> internal_shmdata_writers_;
    StringMap<ShmdataReader::ptr> internal_shmdata_readers_;

    //destinations
    StringMap <RtpDestination::ptr> destinations_;

    static void make_data_stream_available (GstElement* typefind, 
					    guint probability, 
					    GstCaps *caps, 
					    gpointer user_data);
    static gboolean sink_factory_filter (GstPluginFeature * feature, gpointer data);
    static gint sink_compare_ranks (GstPluginFeature *f1, GstPluginFeature *f2);

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
    
    static gboolean data_probe (GstPad * pad,GstMiniObject * mini_obj, gpointer user_data);


  };
}  // end of namespace

#endif // ifndef
