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

#ifndef __SWITCHER_RTPSESSION_H__
#define __SWITCHER_RTPSESSION_H__

#ifdef HAVE_CONFIG_H
#include "../config.h"
#endif

#include <gst/gst.h>
#include <gst/sdp/gstsdpmessage.h>
#include <memory>
#include <map>
#include <string>
#include <mutex>
#include <condition_variable>
#include "./gst-pipeliner.hpp"
#include "./quiddity-manager.hpp"
#include "./rtp-destination.hpp"
#include "./custom-property-helper.hpp"
#include "./gst-shmdata-subscriber.hpp"

namespace switcher {
class RtpSession: public Quiddity {
  friend RtpDestination;
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(RtpSession);
  RtpSession(const std::string &);
  ~RtpSession() = default;
  RtpSession(const RtpSession &) = delete;
  RtpSession &operator=(const RtpSession &) = delete;

  // local streams
  bool add_data_stream(const std::string &shmdata_socket_path);
  bool remove_data_stream(const std::string &shmdata_socket_path);

  // remote dest (using user defined "nick_name")
  bool add_destination(std::string dest_name, std::string host_name);
  bool remove_destination(std::string dest_name);

  // destination property
  static const gchar *get_destinations_json(void *user_data);
  // MTU property
  static void set_mtu_at_add_data_stream(const gint value, void *user_data);
  static gint get_mtu_at_add_data_stream(void *user_data);

  // sending
  bool add_udp_stream_to_dest(std::string shmdata_socket_path,
                              std::string dest_name, std::string port);
  bool remove_udp_stream_to_dest(std::string shmdata_socket_path,
                                 std::string dest_name);
  bool write_sdp_file(std::string dest_name);

  // // will be called by shmdata reader
  // static void attach_data_stream(ShmdataReader *caller,
  //                                void *rtpsession_instance);

 private:
  using DataStream = struct DataStream_t {
    using ptr = std::unique_ptr<DataStream_t>;
    DataStream_t() = delete;
    DataStream_t(GstElement *rtpsession): rtp (rtpsession) {}
    DataStream_t(DataStream_t &) = delete;
    DataStream_t &operator=(const DataStream_t &) = delete;
    ~DataStream_t();
    guint id{};
    // RTP session
    GstElement *rtp;
    // shm
    GstElement *shmdatasrc{nullptr};
    std::unique_ptr<GstShmdataSubscriber> shm_sub{nullptr};
    // UDP/RTP
    GstPad *rtp_static_pad{nullptr};
    GstElement *udp_rtp_bin{nullptr};
    GstElement *udp_rtp_sink{nullptr};
    // UDP/RTCP
    GstPad *rtcp_requested_pad{nullptr};
    GstElement *udp_rtcp_sink{nullptr};
  };

  std::unique_ptr<GstPipeliner> gst_pipeline_;
  GstElement *rtpsession_{nullptr};
  // a counter used for setting id of internal streams
  // this value is arbitrary and can be changed
  guint next_id_{79};

  // custom properties:
  CustomPropertyHelper::ptr custom_props_;
  GParamSpec *destination_description_json_{nullptr};
  std::string destinations_json_{};
  GParamSpec *mtu_at_add_data_stream_spec_{nullptr};
  gint mtu_at_add_data_stream_{1400};

  // data streams
  std::map<std::string, DataStream::ptr> data_streams_{};
  std::mutex stream_mutex_{};
  std::condition_variable stream_cond_{};
  bool stream_added_{false};
  
  // destinations
  std::map<std::string, RtpDestination::ptr> destinations_{};

  bool init() final;
  void on_rtp_caps(const std::string &shmdata_path, std::string caps);
  // return RTP internal pad 
  std::string make_rtp_payloader(GstElement *shmdatasrc,
                          const std::string &caps);
  static gboolean sink_factory_filter(GstPluginFeature *feature,
                                      gpointer data);
  static gint sink_compare_ranks(GstPluginFeature *f1,
                                 GstPluginFeature *f2);
  // internal rtpbin signals
  static void on_bye_ssrc(GstElement *rtpbin, guint session, guint ssrc,
                          gpointer user_data);
  static void on_bye_timeout(GstElement *rtpbin, guint session,
                             guint ssrc, gpointer user_data);
  static void on_new_ssrc(GstElement *rtpbin, guint session, guint ssrc,
                          gpointer user_data);
  static void on_npt_stop(GstElement *rtpbin, guint session, guint ssrc,
                          gpointer user_data);
  static void on_sender_timeout(GstElement *rtpbin, guint session,
                                guint ssrc, gpointer user_data);
  static void on_ssrc_active(GstElement *rtpbin, guint session,
                             guint ssrc, gpointer user_data);
  static void on_ssrc_collision(GstElement *rtpbin, guint session,
                                guint ssrc, gpointer user_data);
  static void on_ssrc_sdes(GstElement *rtpbin, guint session, guint ssrc,
                           gpointer user_data);
  static void on_ssrc_validated(GstElement *rtpbin, guint session,
                                guint ssrc, gpointer user_data);
  static void on_timeout(GstElement *rtpbin, guint session, guint ssrc,
                         gpointer user_data);
  static void on_pad_added(GstElement * gstelement, GstPad *new_pad,
                           gpointer user_data);
  static void on_pad_removed(GstElement * gstelement, GstPad *new_pad,
                             gpointer user_data);
  static void on_no_more_pad(GstElement *gstelement, gpointer user_data);

  // wrapper for registering the data_stream functions
  static gboolean add_data_stream_wrapped(gpointer shmdata_socket_path,
                                          gpointer user_data);
  static gboolean remove_data_stream_wrapped(gpointer shmdata_socket_path,
                                             gpointer user_data);
  static gboolean add_destination_wrapped(gpointer desst_name,
                                          gpointer host_name,
                                          gpointer user_data);
  static gboolean remove_destination_wrapped(gpointer nick_name,
                                             gpointer user_data);
  static gboolean add_udp_stream_to_dest_wrapped(gpointer shmdata_name,
                                                 gpointer dest_name,
                                                 gpointer port,
                                                 gpointer user_data);
  static gboolean remove_udp_stream_to_dest_wrapped(gpointer
                                                    shmdata_socket_path,
                                                    gpointer dest_name,
                                                    gpointer user_data);
  static gboolean write_sdp_file_wrapped(gpointer nick_name,
                                         gpointer user_data);
  static void on_rtppayloder_caps(GstElement *typefind,
                                  guint probability,
                                  GstCaps *caps,
                                  gpointer user_data);
  bool make_udp_sinks(const std::string &shmpath,
                      const std::string &rtp_id);
#if HAVE_OSX
  static void set_udp_sock(GstElement *udpsink);
#endif
};

}  // namespace switcher
#endif
