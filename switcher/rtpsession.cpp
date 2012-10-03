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

#include "switcher/rtpsession.h"

namespace switcher
{
  RtpSession::RtpSession ()
  {
    rtpsession_ = gst_element_factory_make ("gstrtpbin",NULL);
   
    
    gst_bin_add (GST_BIN (bin_), rtpsession_);
    //set the name before registering properties
    name_ = gst_element_get_name (rtpsession_);
    
  }

  BaseEntityDocumentation RtpSession::doc_ ("RTP session", "rtpsession",
					 "RTP session manager");

  BaseEntityDocumentation 
  RtpSession::get_documentation ()
  {
    return doc_;
  }
  
  void
  on_bye_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_bye_ssrc\n");
  }

  void
  on_bye_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_bye_timeout\n");
  }

  void
  on_new_ssrc (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_new_ssrc\n");
  }

  void
  on_npt_stop (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_npt_stop\n");
  }

  void
  on_sender_timeout (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_sender_timeout\n");
  }

  void
  on_ssrc_active  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_ssrc_active\n");
  }

  void
  on_ssrc_collision (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_ssrc_active\n");
  }

  void
  on_ssrc_sdes  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_ssrc_sdes\n");
  }

  void
  on_ssrc_validated (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_ssrc_validated\n");
  }
 
  void
  on_timeout  (GstElement *rtpbin, guint session, guint ssrc, gpointer user_data)
  {
    g_print ("on_timeout\n");
  }


}
