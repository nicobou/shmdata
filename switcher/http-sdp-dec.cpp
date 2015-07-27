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

#include <glib/gprintf.h>
#include <memory>
#include "./http-sdp-dec.hpp"
#include "./gst-utils.hpp"
#include "./scope-exit.hpp"
#include "./std2.hpp"
#include "./g-source-wrapper.hpp"
#include "./gst-shmdata-subscriber.hpp"
#include "./shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    HTTPSDPDec,
    "httpsdpdec",
    "HTTP/SDP Player",
    "network",
    "writer",
    "decode an sdp-described stream delivered through http and make shmdatas",
    "LGPL",
    "Nicolas Bouillot");

HTTPSDPDec::HTTPSDPDec(const std::string &):
    gst_pipeline_(std2::make_unique<GstPipeliner>(nullptr, nullptr)),
    souphttpsrc_("souphttpsrc"),
    sdpdemux_("sdpdemux") {
}

bool HTTPSDPDec::init() {
  if (!souphttpsrc_ || !sdpdemux_)
    return false;
  install_method("To Shmdata",
                 "to_shmdata",
                 "get streams from sdp description over http, "
                 "accept also base64 encoded SDP string",
                 "success or fail",
                 Method::make_arg_description("URL",
                                              "url",
                                              "URL to the sdp file, or a base64 encoded SDP description",
                                              nullptr),
                 (Method::method_ptr)to_shmdata_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr), this);
  return true;
}

void HTTPSDPDec::init_httpsdpdec() {
  if (!UGstElem::renew(souphttpsrc_))
    g_warning("error renewing souphttpsrc_"); 
  if (!UGstElem::renew(sdpdemux_))
    g_warning("error renewing sdpdemux_"); 
  g_signal_connect(GST_BIN(sdpdemux_.get_raw()),
                   "element-added",
                   (GCallback) HTTPSDPDec::on_new_element_in_sdpdemux,
                   nullptr);
  g_object_set(G_OBJECT(sdpdemux_.get_raw()),
               "latency", 0, nullptr);
  g_object_set(G_OBJECT(sdpdemux_.get_raw()),
               "async-handling", TRUE, nullptr);
  g_signal_connect(G_OBJECT(sdpdemux_.get_raw()),
                   "pad-added", (GCallback)httpsdpdec_pad_added_cb, this);
}

void HTTPSDPDec::destroy_httpsdpdec() {
  shm_subs_.clear();
  prune_tree(".shmdata.writer");
  make_new_error_handler();
  gst_pipeline_ = std2::make_unique<GstPipeliner>(nullptr, nullptr);
  counter_.reset_counter_map();
}

void
HTTPSDPDec::on_new_element_in_sdpdemux(GstBin */*bin*/,
                                       GstElement */*element*/,
                                       gpointer /*user_data*/) {
  // FIXME add that in uridecodebin
  //g_object_set(G_OBJECT(element), "ntp-sync", TRUE, nullptr);
}

void HTTPSDPDec::make_new_error_handler() {
      on_error_.emplace_back(
          std2::make_unique<GSourceWrapper>([&](){uri_to_shmdata();},
                                            retry_delay_,
                                            true));
      // cleaning old sources
      if (2 < on_error_.size())
        on_error_.pop_front();
}

void HTTPSDPDec::configure_shmdatasink(GstElement *element,
                                       const std::string &media_type,
                                       const std::string &media_label){
  auto count = counter_.get_count(media_label + media_type);
  std::string media_name = media_type;
  if (count != 0)
    media_name.append("-" + std::to_string(count));
  std::string shmpath;
  if (media_label.empty())
    shmpath = make_file_name(media_name);
  else
    shmpath = make_file_name(media_label + "-" + media_name);
  
  g_object_set(G_OBJECT(element), "socket-path", shmpath.c_str(), nullptr);
  shm_subs_.emplace_back(
      std2::make_unique<GstShmdataSubscriber>(
          element,
          [this, shmpath]( const std::string &caps){
            this->graft_tree(".shmdata.writer." + shmpath,
                             ShmdataUtils::make_tree(caps,
                                                     ShmdataUtils::get_category(caps),
                                                     0));
          },
          [this, shmpath](GstShmdataSubscriber::num_bytes_t byte_rate){
            auto tree = this->prune_tree(".shmdata.writer." + shmpath, false);
            if (!tree)
              return;
            tree->graft(".byte_rate",
                        data::Tree::make(byte_rate));
            this->graft_tree(".shmdata.writer." + shmpath, tree);
          }));
}

void HTTPSDPDec::httpsdpdec_pad_added_cb(GstElement */*object */,
                                         GstPad *pad,
                                         gpointer user_data) {
  HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
  std::unique_ptr<DecodebinToShmdata> decodebin = 
      std2::make_unique<DecodebinToShmdata>(
          context->gst_pipeline_.get(),
          [context](GstElement *el, const std::string &media_type, const std::string &media_label){
            context->configure_shmdatasink(el, media_type, media_label);
          });
  if(!decodebin->invoke_with_return<gboolean>([context](GstElement *el) {
        return gst_bin_add(GST_BIN(context->gst_pipeline_->get_pipeline()), el);
      })){
      g_warning("decodebin cannot be added to pipeline");
    }
  GstPad *sinkpad =
      decodebin->invoke_with_return<GstPad *>([](GstElement *el){
          return gst_element_get_static_pad(el, "sink");
        });
  On_scope_exit {gst_object_unref(GST_OBJECT(sinkpad));};
  GstUtils::check_pad_link_return(gst_pad_link(pad, sinkpad));
  auto caps = gst_pad_get_allowed_caps(pad);
  On_scope_exit{gst_caps_unref(caps);};
  auto structure = gst_caps_get_structure(caps, 0);
  auto media_label = gst_structure_get_string (structure, "media-label");
  if (nullptr != media_label)
    decodebin->set_media_label(gst_structure_get_string (structure, "media-label"));
  decodebin->invoke([](GstElement *el) {
      GstUtils::sync_state_with_parent(el);
    });
  context->decodebins_.push_back(std::move(decodebin));
}

gboolean HTTPSDPDec::to_shmdata_wrapped(gpointer uri, gpointer user_data) {
  HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
  if (!context->to_shmdata((char *)uri))
    return FALSE;
  return TRUE;
}

bool HTTPSDPDec::to_shmdata(std::string uri) {
  if (uri.find("http://") == 0) {
    is_dataurisrc_ = false;
    souphttpsrc_.mute("souphttpsrc");
    uri_ = std::move(uri);
  } else {
    is_dataurisrc_ = true;
    souphttpsrc_.mute("dataurisrc");
    uri_ =  std::string("data:application/sdp;base64," + uri);
  }
  on_error_.clear();
  uri_to_shmdata();
  return true;
}

void HTTPSDPDec::uri_to_shmdata() {
  destroy_httpsdpdec();
  prune_tree(".shmdata.writer");
  init_httpsdpdec();
  g_object_set_data(G_OBJECT(sdpdemux_.get_raw()),
                    "on-error-gsource",
                    (gpointer)on_error_.back().get());
  g_debug("httpsdpdec: to_shmdata set uri %s", uri_.c_str());
  if(!is_dataurisrc_)   // for souphttpsrc
    g_object_set(G_OBJECT(souphttpsrc_.get_raw()),
                 "location", uri_.c_str(), nullptr);
  else  // for dataurisrc
    g_object_set(G_OBJECT(souphttpsrc_.get_raw()),
                 "uri", uri_.c_str(), nullptr);
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   souphttpsrc_.get_raw(),
                   sdpdemux_.get_raw(),
                   nullptr);
  gst_element_link(souphttpsrc_.get_raw(), sdpdemux_.get_raw());
  gst_pipeline_->play(true);
}

}  // namespace switcher
