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

#if HAVE_OSX
#include <sys/socket.h>
#endif

#include <glib/gstdio.h>  // writing sdp file
#include <chrono>
#include "./scope-exit.hpp"
#include "./rtp-session.hpp"
#include "./gst-utils.hpp"
#include "./json-builder.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(RtpSession,
                                     "RTP Session",
                                     "network",
                                     "RTP session manager",
                                     "LGPL",
                                     "rtpsession",
                                     "Nicolas Bouillot");

RtpSession::RtpSession():
    custom_props_(new CustomPropertyHelper()) {
}

RtpSession::~RtpSession() {
  GstUtils::clean_element(rtpsession_);
}

bool RtpSession::init_gpipe() {
  if (!GstUtils::make_element("gstrtpbin", &rtpsession_))
    return false;
  g_object_set(G_OBJECT(rtpsession_), "ntp-sync", TRUE, nullptr);

  g_object_set(G_OBJECT(get_bin()), "async-handling", TRUE, nullptr);

  g_signal_connect(G_OBJECT(rtpsession_), "on-bye-ssrc",
                   (GCallback) on_bye_ssrc, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-bye-timeout",
                   (GCallback) on_bye_timeout, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-new-ssrc",
                   (GCallback) on_new_ssrc, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-npt-stop",
                   (GCallback) on_npt_stop, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-sender-timeout",
                   (GCallback) on_sender_timeout, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-ssrc-active",
                   (GCallback) on_ssrc_active, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-ssrc-collision",
                   (GCallback) on_ssrc_collision, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-ssrc-sdes",
                   (GCallback) on_ssrc_sdes, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-ssrc-validated",
                   (GCallback) on_ssrc_validated, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "on-timeout",
                   (GCallback) on_timeout, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "pad-added",
                   (GCallback) on_pad_added, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "pad-removed",
                   (GCallback) on_pad_removed, (gpointer) this);
  g_signal_connect(G_OBJECT(rtpsession_), "no-more-pads",
                   (GCallback) on_no_more_pad, (gpointer) this);

  gst_bin_add(GST_BIN(get_bin()), rtpsession_);
  GstUtils::sync_state_with_parent(rtpsession_);

  install_method("Add Data Stream",
                 "add_data_stream",
                 "add a data stream to the RTP session (sending)",
                 "succes or fail",
                 Method::make_arg_description("Shmdata Path",
                                              "socket",
                                              "shmdata socket path to add to the session",
                                              nullptr),
                 (Method::method_ptr) &add_data_stream_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr), this);

  install_method("Remove Data Stream",
                 "remove_data_stream",
                 "remove a data stream from the RTP session (sending)",
                 "success or fail",
                 Method::make_arg_description("Shmdata Path",
                                              "socket",
                                              "shmdata socket path to remove from the session",
                                              nullptr),
                 (Method::method_ptr) &remove_data_stream_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr),
                 this);

  install_method("Add Destination",
                 "add_destination",
                 "add a destination (two destinations can share the same host name)",
                 "success or fail",
                 Method::make_arg_description("Name",
                                              "name",
                                              "a destination name (user defined)",
                                              "Host Name or IP",
                                              "host_name",
                                              "the host name of the destination",
                                              nullptr),
                 (Method::method_ptr) &add_destination_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   G_TYPE_STRING,
                                                   nullptr),
                 this);

  install_method("Remove Destination",
                 "remove_destination",
                 "remove a destination",
                 "success or fail",
                 Method::make_arg_description("Name",
                                              "name",
                                              "the destination name",
                                              nullptr),
                 (Method::method_ptr) &remove_destination_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr), this);

  install_method("Add UDP Stream",
                 "add_udp_stream_to_dest",
                 "stream RTP to a port with udp",
                 "success or fail",
                 Method::make_arg_description("Shmdata Path", "socket",
                                              "local socket path of the shmdata",
                                              "Destination", "dest",
                                              "name of the destination",
                                              "Port", "port",
                                              "destination port",
                                              nullptr),
                 (Method::method_ptr) &add_udp_stream_to_dest_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   G_TYPE_STRING,
                                                   G_TYPE_STRING,
                                                   nullptr),
                 this);

  install_method("Remove UDP Stream",
                 "remove_udp_stream_to_dest",
                 "remove destination",
                 "succes or fail",
                 Method::make_arg_description("Shmdata Path", "socket",
                                              "local socket path of the shmdata",
                                              "Destination", "dest_name",
                                              "destination name",
                                              nullptr),
                 (Method::method_ptr) &remove_udp_stream_to_dest_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   G_TYPE_STRING,
                                                   nullptr), this);

  install_method("Write SDP File",
                 "write_sdp_file",
                 "print sdp for the given destination",
                 "success or fail",
                 Method::make_arg_description("Destination",
                                              "name",
                                              "the name of the destination",
                                              nullptr),
                 (Method::method_ptr) &write_sdp_file_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr), this);

  destination_description_json_ =
      custom_props_->make_string_property("destinations-json",
                                          "json formated description of destinations",
                                          "", (GParamFlags) G_PARAM_READABLE,
                                          nullptr,
                                          RtpSession::get_destinations_json,
                                          this);

  install_property_by_pspec(custom_props_->get_gobject(),
                            destination_description_json_,
                            "destinations-json", "Destinations");

  mtu_at_add_data_stream_spec_ =
      custom_props_->make_int_property("mtu-at-add-data-stream",
                                       "MTU that will be set during add_data_stream invokation",
                                       0, 15000, 1400,
                                       (GParamFlags) G_PARAM_READWRITE,
                                       RtpSession::set_mtu_at_add_data_stream,
                                       RtpSession::get_mtu_at_add_data_stream,
                                       this);

  install_property_by_pspec(custom_props_->get_gobject(),
                            mtu_at_add_data_stream_spec_,
                            "mtu-at-add-data-stream",
                            "MTU At Add Data Stream");
  return true;
}

gboolean
RtpSession::write_sdp_file_wrapped(gpointer nick_name,
                                   gpointer user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);

  if (context->write_sdp_file((char *) nick_name))
    return TRUE;
  else
    return FALSE;
}

bool RtpSession::write_sdp_file(std::string dest_name) {
  auto it = destinations_.find(dest_name);
  if (destinations_.end() == it) {
    g_warning("%s does not exists, cannot write sdp file",
         dest_name.c_str());
    return false;
  }
  std::string sdp_file = make_file_name(dest_name);
  sdp_file.append(".sdp");
  // remove old one if present
  g_remove(sdp_file.c_str());
  return it->second->write_to_file(std::move(sdp_file));
}

// function used as a filter for selecting the appropriate rtp payloader
gboolean
RtpSession::sink_factory_filter(GstPluginFeature *feature,
                                gpointer data) {
  const gchar *klass;

  GstCaps *caps = (GstCaps *) data;

  // searching element factories only
  if (!GST_IS_ELEMENT_FACTORY(feature))
    return FALSE;

  klass = gst_element_factory_get_klass(GST_ELEMENT_FACTORY(feature));
  if (!(g_strrstr(klass, "Payloader") && g_strrstr(klass, "RTP")))
    return FALSE;

  if (!gst_element_factory_can_sink_caps
      (GST_ELEMENT_FACTORY(feature), caps))
    return FALSE;

  return TRUE;
}

// sorting factory by rank
gint
RtpSession::sink_compare_ranks(GstPluginFeature *f1,
                               GstPluginFeature *f2) {
  gint diff;

  diff = gst_plugin_feature_get_rank(f2) - gst_plugin_feature_get_rank(f1);
  if (diff != 0)
    return diff;
  return g_strcmp0(gst_plugin_feature_get_name(f2),
                   gst_plugin_feature_get_name(f1));
}

// this is a typefind function, called when type of input stream from a shmdata is found
void
RtpSession::make_data_stream_available(GstElement *typefind,
                                       guint /*probability */ ,
                                       GstCaps *caps,
                                       gpointer user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);
  GstElement *pay = nullptr;
  GList *list = gst_registry_feature_filter(gst_registry_get_default(),
                                            (GstPluginFeatureFilter)
                                            sink_factory_filter,
                                            FALSE, caps);
  list = g_list_sort(list, (GCompareFunc) sink_compare_ranks);

  // bypassing jpeg for high dimensions
  bool jpeg_payloader = true;
  GstStructure *caps_structure = gst_caps_get_structure(caps, 0);
  // check jpeg dimension are suported by jpeg payloader
  if (g_str_has_prefix(gst_structure_get_name(caps_structure), "image/jpeg")) {
    gint width = 0, height = 0;
    if (gst_structure_get_int(caps_structure, "height", &height)) {
      if (height <= 0 || height > 2040)
        jpeg_payloader = false;
    }
    if (gst_structure_get_int(caps_structure, "width", &width)) {
      if (width <= 0 || width > 2040)
        jpeg_payloader = false;
    }
  }

  if (list != nullptr && jpeg_payloader)
    pay = gst_element_factory_create(GST_ELEMENT_FACTORY(list->data),
                                     nullptr);
  else
    GstUtils::make_element("rtpgstpay", &pay);

  ShmdataReader *reader = static_cast<ShmdataReader *>(
      g_object_get_data(G_OBJECT(typefind), "shmdata-reader"));
  reader->add_element_to_cleaner(pay);
  
  g_debug("using %s payloader for %s",
          GST_ELEMENT_NAME(pay),
          reader->get_path().c_str());
  
  // add capture and payloading to the pipeline and link
  gst_bin_add_many(GST_BIN(context->get_bin()), pay, nullptr);
  gst_element_link(typefind, pay);
  GstUtils::sync_state_with_parent(pay);
  g_object_set(G_OBJECT(pay),
               "mtu",
               (guint)context->mtu_at_add_data_stream_,
               nullptr);

  // now link all to the rtpbin, start by getting an RTP sinkpad for session "%d"
  GstPad *sinkpad = gst_element_get_request_pad(context->rtpsession_,
                                                "send_rtp_sink_%d");
  On_scope_exit {gst_object_unref(sinkpad);}; 
  GstPad *srcpad = gst_element_get_static_pad(pay, "src");
  On_scope_exit {gst_object_unref(srcpad);}; 
  if (gst_pad_link(srcpad, sinkpad) != GST_PAD_LINK_OK)
    g_warning("failed to link payloader to rtpbin");
  
  // get name for the newly created pad
  gchar *rtp_sink_pad_name = gst_pad_get_name(sinkpad);
  On_scope_exit{g_free(rtp_sink_pad_name);};
  gchar **rtp_session_array = g_strsplit_set(rtp_sink_pad_name, "_", 0);
  On_scope_exit{g_strfreev(rtp_session_array);};
  context->make_udp_sinks(reader->get_path(),
                          rtp_session_array[3]);
  std::unique_lock<std::mutex> lock(context->stream_mutex_);
  context->stream_cond_.notify_one();
  // FIXME We also want to receive RTCP, request an RTCP sinkpad for given session and
  // link it to a funnel for future linking with network connections
}

void
RtpSession::attach_data_stream(ShmdataReader *caller,
                               void *user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);
  GstElement *funnel, *typefind;
  GstUtils::make_element("funnel", &funnel);
  GstUtils::make_element("typefind", &typefind);
  // give caller to typefind in order to register telement to remove
  g_object_set_data(G_OBJECT(typefind), "shmdata-reader",
                    (gpointer) caller);
  g_signal_connect(typefind, "have-type",
                   G_CALLBACK(RtpSession::make_data_stream_available),
                   context);
  gst_bin_add_many(GST_BIN(context->get_bin()), funnel, typefind, nullptr);
  gst_element_link(funnel, typefind);
  GstUtils::sync_state_with_parent(funnel);
  GstUtils::sync_state_with_parent(typefind);
  caller->set_sink_element(funnel);
  caller->add_element_to_cleaner(funnel);
  caller->add_element_to_cleaner(typefind);
}

gboolean
RtpSession::add_destination_wrapped(gpointer nick_name,
                                    gpointer host_name,
                                    gpointer user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);

  if (context->add_destination((char *) nick_name, (char *) host_name))
    return TRUE;
  else
    return FALSE;
}

bool
RtpSession::add_destination(std::string nick_name, std::string host_name)
{
  if (destinations_.end() != destinations_.find(nick_name)) {
    g_debug("RtpSession: a destination named %s already exists, cannot add",
         nick_name.c_str());
    return false;
  }
  RtpDestination::ptr dest = std::make_shared<RtpDestination>(this);
  dest->set_name(nick_name);
  dest->set_host_name(host_name);
  destinations_[nick_name] = dest;
  return true;
}

gboolean
RtpSession::remove_destination_wrapped(gpointer nick_name,
                                       gpointer user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);

  if (context->remove_destination((char *) nick_name))
    return TRUE;
  else
    return FALSE;
}

bool RtpSession::remove_destination(std::string nick_name) {
  auto it = destinations_.find(nick_name);
  if (destinations_.end() == it) {
    g_warning
        ("RtpSession: destination named %s does not exists, cannot remove",
         nick_name.c_str());
    return false;
  }
  destinations_.erase(it);
  return true;
}

gboolean
RtpSession::add_udp_stream_to_dest_wrapped(gpointer shmdata_name,
                                           gpointer nick_name,
                                           gpointer port,
                                           gpointer user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);

  if (context->add_udp_stream_to_dest
      ((char *) shmdata_name, (char *) nick_name, (char *) port))
    return TRUE;
  else
    return FALSE;
}

bool
RtpSession::add_udp_stream_to_dest(std::string shmpath,
                                   std::string nick_name,
                                   std::string port) {
  auto ds_it = data_streams_.find(shmpath);
  if (data_streams_.end() == ds_it) {
    g_warning("RtpSession is not connected to %s",
              shmpath.c_str());
    return false;
  }
  std::string id = std::to_string(ds_it->second->id);

  auto destination_it = destinations_.find(nick_name);
  if (destinations_.end() == destination_it) {
    g_warning("RtpSession does not contain a destination named %s",
              nick_name.c_str());
    return false;
  }
  
  gint rtp_port = atoi(port.c_str());
  if (rtp_port % 2 != 0) {
    g_warning("rtp destination port %s must be even, not odd",
              port.c_str());
    return false;
  }

  // rtp stream (sending)
  RtpDestination::ptr dest = destinations_[nick_name];
  dest->add_stream(shmpath, port);
  g_signal_emit_by_name (ds_it->second->udp_rtp_sink,
                         "add",
                         dest->get_host_name().c_str(),
                         rtp_port,
                         nullptr);

  // rtcp stream (sending)
  g_signal_emit_by_name (ds_it->second->udp_rtcp_sink,
                         "add",
                         dest->get_host_name().c_str(),
                         rtp_port + 1,
                         nullptr);
  return true;
}

gboolean
RtpSession::remove_udp_stream_to_dest_wrapped(gpointer
                                              shmpath,
                                              gpointer dest_name,
                                              gpointer user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);
  if (context->remove_udp_stream_to_dest((char *) shmpath,
                                         (char *) dest_name))
    return TRUE;
  return FALSE;
}

bool
RtpSession::remove_udp_stream_to_dest(std::string shmpath,
                                      std::string dest_name) {
  auto ds_it = data_streams_.find(shmpath);
  if (data_streams_.end() == ds_it) {
    g_warning("RtpSession is not connected to %s",
              shmpath.c_str());
    return false;
  }

  RtpDestination::ptr dest = destinations_[dest_name];
  if (!(bool) dest) {
    g_warning("RTP: destination %s does not exist",
              dest_name.c_str());
    return false;
  }
  std::string port = dest->get_port(shmpath);
  if (!dest->remove_stream(shmpath))
    return false;

  gint rtp_port = atoi(port.c_str());
  if (rtp_port % 2 != 0) {
    g_warning("rtp destination port %s must be even, not odd",
              port.c_str());
    return false;
  }

  // rtp stream (sending)
  g_signal_emit_by_name (ds_it->second->udp_rtp_sink,
                         "remove",
                         dest->get_host_name().c_str(),
                         rtp_port,
                         nullptr);

  // rtcp stream (sending)
  g_signal_emit_by_name (ds_it->second->udp_rtcp_sink,
                         "remove",
                         dest->get_host_name().c_str(),
                         rtp_port + 1,
                         nullptr);
  return true;
}

gboolean
RtpSession::add_data_stream_wrapped(gpointer connector_name,
                                    gpointer user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);

  if (context->add_data_stream((char *) connector_name))
    return TRUE;
  else
    return FALSE;
}

bool RtpSession::add_data_stream(std::string shmpath) {
  remove_data_stream(shmpath);
  std::unique_lock<std::mutex> lock(stream_mutex_);
  DataStream::ptr ds (new DataStream(rtpsession_));
  ds->id = next_id_;
  next_id_++;
  data_streams_[shmpath] = std::move(ds);
  
  ShmdataReader::ptr reader;
  reader.reset(new ShmdataReader());
  reader->set_path(shmpath.c_str());
  reader->set_g_main_context(get_g_main_context());
  reader->set_bin(get_bin());
  reader->set_on_first_data_hook(attach_data_stream, this);
  reader->start();
  g_debug("%s waiting for data in shm %s",
          __FUNCTION__,
          shmpath.c_str());
  stream_cond_.wait_for(lock, std::chrono::seconds(1));
  // testing if stream has been added
  if (nullptr == data_streams_[shmpath]->udp_rtp_sink) {
    remove_data_stream(shmpath);
    return false;
  }
  register_shmdata(reader);
  return true;
}

gboolean
RtpSession::remove_data_stream_wrapped(gpointer connector_name,
                                       gpointer user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);
  if (context->remove_data_stream((char *) connector_name))
    return TRUE;
  else
    return FALSE;
}

bool RtpSession::remove_data_stream(std::string shmpath) {
  for (auto &it : destinations_) {
    if (it.second->has_shmdata(shmpath))
      it.second->remove_stream(shmpath);
  }

  auto ds_it = data_streams_.find(shmpath);
  if (data_streams_.end() == ds_it) {
    g_debug("RTP remove_data_stream: %s not present",
            shmpath.c_str());
    return false;
  }
  prune_tree("rtp_caps." + shmpath);
  data_streams_.erase(ds_it);
  g_debug("data_stream %s removed", shmpath.c_str());
  return true;
}

void RtpSession::on_bye_ssrc(GstElement * /*rtpbin */ ,
                             guint /*session */ ,
                             guint /*ssrc */ ,
                             gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_bye_ssrc");
}

void RtpSession::on_bye_timeout(GstElement * /*rtpbin */ ,
                                guint /*session */ ,
                                guint /*ssrc */ ,
                                gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_bye_timeout");
}

void RtpSession::on_new_ssrc(GstElement * /*rtpbin */ ,
                             guint /*session */ ,
                             guint /*ssrc */ ,
                             gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_new_ssrc");
}

void RtpSession::on_npt_stop(GstElement * /*rtpbin */ ,
                             guint /*session */ ,
                             guint /*ssrc */ ,
                             gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_npt_stop");
}

void RtpSession::on_sender_timeout(GstElement * /*rtpbin */ ,
                                   guint /*session */ ,
                                   guint /*ssrc */ ,
                                   gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_sender_timeout");
}

void RtpSession::on_ssrc_active(GstElement * /*rtpbin */ ,
                                guint /*session */ ,
                                guint /*ssrc */ ,
                                gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_ssrc_active");
}

void RtpSession::on_ssrc_collision(GstElement * /*rtpbin */ ,
                                   guint /*session */ ,
                                   guint /*ssrc */ ,
                                   gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_ssrc_active");
}

void RtpSession::on_ssrc_sdes(GstElement * /*rtpbin */ ,
                              guint /*session */ ,
                              guint /*ssrc */ ,
                              gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_ssrc_sdes");
}

void RtpSession::on_ssrc_validated(GstElement * /*rtpbin */ ,
                                   guint /*session */ ,
                                   guint /*ssrc */ ,
                                   gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_ssrc_validated");
}

void RtpSession::on_timeout(GstElement * /*rtpbin */ ,
                            guint /*session */ ,
                            guint /*ssrc */ ,
                            gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_timeout");
}

void RtpSession::on_pad_added(GstElement * /*gstelement */ ,
                              GstPad *new_pad, gpointer user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_pad_added, name: %s, direction: %d",
          gst_pad_get_name(new_pad), gst_pad_get_direction(new_pad));
  // gchar *bidule = g_strdup ("bidule");
  context->signal_emit("truc", "bidule");
}

void RtpSession::on_pad_removed(GstElement * /*gstelement */ ,
                                GstPad *new_pad, gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_pad_removed");
}

void RtpSession::on_no_more_pad(GstElement * /*gstelement */ ,
                                gpointer /*user_data */ ) {
  // RtpSession *context = static_cast<RtpSession *>(user_data);
  g_debug("on_no_more_pad");
}

const gchar *RtpSession::get_destinations_json(void *user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);

  JSONBuilder::ptr destinations_json = std::make_shared<JSONBuilder>();
  destinations_json->reset();
  destinations_json->begin_object();
  destinations_json->set_member_name("destinations");
  destinations_json->begin_array();
  for (auto &it : context->destinations_)
    destinations_json->add_node_value(it.second->get_json_root_node());
  destinations_json->end_array();
  destinations_json->end_object();
  context->destinations_json_ = destinations_json->get_string(true);
return context->destinations_json_.c_str();
}

void
RtpSession::set_mtu_at_add_data_stream(const gint value, void *user_data)
{
  RtpSession *context = static_cast<RtpSession *>(user_data);
  context->mtu_at_add_data_stream_ = value;
}

gint RtpSession::get_mtu_at_add_data_stream(void *user_data) {
  RtpSession *context = static_cast<RtpSession *>(user_data);
  return context->mtu_at_add_data_stream_;
}

void RtpSession::on_rtp_caps(std::string shmdata_path, std::string caps) {
  graft_tree("rtp_caps." + std::move(shmdata_path),
             data::Tree::make(std::move(caps)));
}

// this is a typefind function used in order to get rtp payloader caps
void RtpSession::on_rtppayloder_caps(GstElement *typefind,
                                     guint /*probability */ ,
                                     GstCaps *caps,
                                     gpointer /*user_data*/) {
  RtpSession *context = static_cast<RtpSession *>(
      g_object_get_data(G_OBJECT(typefind), "rtp_session"));
  gchar *shmpath = static_cast<char *>(
      g_object_get_data(G_OBJECT(typefind), "shmdata_path"));
  On_scope_exit{g_free(shmpath);};
  gchar *caps_str = gst_caps_to_string(caps);
  On_scope_exit{g_free(caps_str);};
  context->on_rtp_caps(std::string(shmpath),
                       std::string(caps_str));
}

#if HAVE_OSX
void RtpSession::set_udp_sock(GstElement *udpsink) {
  // turnaround for OSX: create sender socket
  int sock;
  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    g_warning("udp sink: cannot create socket");
    return false;
  }
  guint bc_val = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bc_val, sizeof(bc_val)) < 0) {
    g_warning("udp sink: cannot set broadcast to socket");
    return false;
  }
  g_object_set(G_OBJECT(udpsink), "sockfd", sock, nullptr);
}
#endif

bool RtpSession::make_udp_sinks(const std::string &shmpath,
                                const std::string &rtp_id) {
  auto it = data_streams_.find(shmpath);
  if(data_streams_.end() == it) {
    g_warning("cannot make udp sink");
    return false;
  }
  {  // RTP
    GstPad *src_pad = gst_element_get_static_pad(
        rtpsession_,
        std::string("send_rtp_src_" + rtp_id).c_str());
    //saving for latter cleaning
    it->second->rtp_static_pad = src_pad;

    GstElement *udpsink_bin, *typefind, *udpsink;
    if (!GstUtils::make_element("bin", &udpsink_bin)
        || !GstUtils::make_element("typefind", &typefind)
        || !GstUtils::make_element("multiudpsink", &udpsink))
      return false;
    // saving for latter cleaning
    it->second->udp_rtp_bin = udpsink_bin;
    // saving for latter controling
    it->second->udp_rtp_sink = udpsink;
    
    g_object_set (G_OBJECT (udpsink_bin), "async-handling", TRUE, nullptr);
    g_object_set(G_OBJECT(udpsink), "sync", FALSE, nullptr);
    // saving shmpath and this for use when type find will throw "have-type" 
    g_object_set_data(G_OBJECT(typefind), "shmdata_path",
                      (gpointer) g_strdup(shmpath.c_str()));
    g_object_set_data(G_OBJECT(typefind), "rtp_session",
                      (gpointer) this);
    g_signal_connect(typefind,
                     "have-type",
                     G_CALLBACK(RtpSession::on_rtppayloder_caps),
                     nullptr);
#if HAVE_OSX
    set_udp_sock(udpsink);
#endif
    gst_bin_add_many(GST_BIN(udpsink_bin),
                     typefind, udpsink, nullptr);
    gst_element_link(typefind, udpsink);
    gst_bin_add(GST_BIN(get_bin()), udpsink_bin);
    GstPad *sink_pad = gst_element_get_static_pad(typefind, "sink");
    GstPad *ghost_sinkpad = gst_ghost_pad_new(nullptr, sink_pad);
    gst_pad_set_active(ghost_sinkpad, TRUE);
    gst_element_add_pad(udpsink_bin, ghost_sinkpad);
    gst_object_unref(sink_pad);
    if (GST_PAD_LINK_OK != gst_pad_link (src_pad, ghost_sinkpad))
      g_warning ("linking with multiudpsink bin failled");
    GstUtils::sync_state_with_parent(udpsink_bin);
    GstUtils::wait_state_changed(udpsink_bin);
  }
  {  // RTCP
    GstPad *rtcp_src_pad =
        gst_element_get_request_pad(
            rtpsession_,
            std::string("send_rtcp_src_" + rtp_id).c_str());
    //saving for latter cleaning
    it->second->rtcp_requested_pad = rtcp_src_pad;
    if (!GstUtils::make_element("multiudpsink", &it->second->udp_rtcp_sink))
      return false;
    // saving for latter controling
    GstElement *udpsink = it->second->udp_rtcp_sink;
    g_object_set(G_OBJECT(udpsink), "sync", FALSE, nullptr);
#if HAVE_OSX
    set_udp_sock(udpsink);
#endif
    gst_bin_add(GST_BIN(get_bin()), udpsink);
    GstUtils::sync_state_with_parent(udpsink);
    GstPad *sink_pad = gst_element_get_static_pad(udpsink, "sink");
    On_scope_exit{gst_object_unref(sink_pad);};
    if (GST_PAD_LINK_OK != gst_pad_link (rtcp_src_pad, sink_pad))
      g_warning ("linking with multiudpsink bin failled");
  }
  return true;
}

RtpSession::DataStream_t::~DataStream_t() {
  if(nullptr != rtp_static_pad)
    gst_object_unref(rtp_static_pad);
  GstUtils::clean_element(udp_rtp_sink);
  GstUtils::clean_element(udp_rtp_bin);
  if(nullptr != rtcp_requested_pad)
    gst_element_release_request_pad(rtp, rtcp_requested_pad);
  GstUtils::clean_element(udp_rtcp_sink);
}

}
