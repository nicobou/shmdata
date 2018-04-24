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

#include "./http-sdp-dec.hpp"
#include <glib/gprintf.h>
#include <memory>
#include "./g-source-wrapper.hpp"
#include "./gst-shmdata-subscriber.hpp"
#include "./gst-utils.hpp"
#include "./information-tree-json.hpp"
#include "./scope-exit.hpp"
#include "./shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    HTTPSDPDec,
    "httpsdpdec",
    "HTTP/SDP Player",
    "network",
    "writer",
    "Decode an SDP-described stream delivered through http and make shmdata",
    "LGPL",
    "Nicolas Bouillot");

HTTPSDPDec::HTTPSDPDec(quid::Config&& conf)
    : Quiddity(std::forward<quid::Config>(conf)),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)),
      souphttpsrc_("souphttpsrc"),
      sdpdemux_("sdpdemux"),
      decompress_streams_id_(
          pmanage<MPtr(&PContainer::make_bool)>("decompress",
                                                [this](const bool& val) {
                                                  decompress_streams_ = val;
                                                  return true;
                                                },
                                                [this]() { return decompress_streams_; },
                                                "Decompress",
                                                "Decompress received streams",
                                                decompress_streams_)),
      to_shm_id_(mmanage<MPtr(&MContainer::make_method<std::function<bool(std::string)>>)>(
          "to_shmdata",
          JSONSerializer::deserialize(
              R"(
                  {
                   "name" : "To Shmdata",
                   "description" : "get streams from sdp description over http, accept also base64 encoded SDP string",
                   "arguments" : [
                     {
                        "long name" : "URL",
                        "description" : "URL to the sdp file, or a base64 encoded SDP description",
                     }
                   ]
                  }
              )"),
          [this](const std::string& uri) { return to_shmdata(uri); })) {
  if (!souphttpsrc_ || !sdpdemux_) {
    is_valid_ = false;
    return;
  }
}

void HTTPSDPDec::init_httpsdpdec() {
  if (!UGstElem::renew(souphttpsrc_)) warning("error renewing souphttpsrc_");
  if (!UGstElem::renew(sdpdemux_)) warning("error renewing sdpdemux_");
  g_signal_connect(GST_BIN(sdpdemux_.get_raw()),
                   "element-added",
                   (GCallback)HTTPSDPDec::on_new_element_in_sdpdemux,
                   nullptr);
  g_object_set(G_OBJECT(sdpdemux_.get_raw()), "latency", 0, nullptr);
  g_object_set(G_OBJECT(sdpdemux_.get_raw()), "async-handling", TRUE, nullptr);
  g_signal_connect(
      G_OBJECT(sdpdemux_.get_raw()), "pad-added", (GCallback)httpsdpdec_pad_added_cb, this);
}

void HTTPSDPDec::destroy_httpsdpdec() {
  shm_subs_.clear();
  prune_tree(".shmdata.writer");
  make_new_error_handler();
  gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);
  counter_.reset_counter_map();
}

void HTTPSDPDec::on_new_element_in_sdpdemux(GstBin* /*bin*/,
                                            GstElement* /*element*/,
                                            gpointer /*user_data*/) {
  // FIXME add that in uridecodebin
  // g_object_set(G_OBJECT(element), "ntp-sync", TRUE, nullptr);
}

void HTTPSDPDec::make_new_error_handler() {
  on_error_.emplace_back(
      std::make_unique<GSourceWrapper>([&]() { uri_to_shmdata(); }, retry_delay_, true));
  // cleaning old sources
  if (2 < on_error_.size()) on_error_.pop_front();
}

void HTTPSDPDec::configure_shmdatasink(GstElement* element,
                                       const std::string& media_type,
                                       const std::string& media_label) {
  auto count = counter_.get_count(media_label + media_type);
  std::string media_name = media_type;
  if (count != 0) media_name.append("-" + std::to_string(count));
  std::string shmpath;
  if (media_label.empty())
    shmpath = make_shmpath(media_name);
  else
    shmpath = make_shmpath(media_label + "-" + media_name);

  g_object_set(G_OBJECT(element), "socket-path", shmpath.c_str(), nullptr);
  shm_subs_.emplace_back(std::make_unique<GstShmdataSubscriber>(
      element,
      [this, shmpath](const std::string& caps) {
        this->graft_tree(
            ".shmdata.writer." + shmpath,
            ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
      },
      ShmdataStat::make_tree_updater(this, ".shmdata.writer." + shmpath)));
}

void HTTPSDPDec::httpsdpdec_pad_added_cb(GstElement* /*object */, GstPad* pad, gpointer user_data) {
  HTTPSDPDec* context = static_cast<HTTPSDPDec*>(user_data);
  std::unique_ptr<DecodebinToShmdata> decodebin = std::make_unique<DecodebinToShmdata>(
      context->gst_pipeline_.get(),
      [context](GstElement* el, const std::string& media_type, const std::string& media_label) {
        context->configure_shmdatasink(el, media_type, media_label);
      },
      [context]() { context->warning("discarding uncomplete custom frame due to a network loss"); },
      context->decompress_streams_);
  if (!decodebin->invoke_with_return<gboolean>([context](GstElement* el) {
        return gst_bin_add(GST_BIN(context->gst_pipeline_->get_pipeline()), el);
      })) {
    context->warning("decodebin cannot be added to pipeline");
  }
  GstPad* sinkpad = decodebin->invoke_with_return<GstPad*>(
      [](GstElement* el) { return gst_element_get_static_pad(el, "sink"); });
  On_scope_exit { gst_object_unref(GST_OBJECT(sinkpad)); };
  GstUtils::check_pad_link_return(gst_pad_link(pad, sinkpad));
  auto caps = gst_pad_get_allowed_caps(pad);
  On_scope_exit { gst_caps_unref(caps); };
  auto structure = gst_caps_get_structure(caps, 0);
  auto media_label = gst_structure_get_string(structure, "media-label");
  if (media_label) decodebin->set_media_label(StringUtils::base64_decode(media_label));
  decodebin->invoke([](GstElement* el) { GstUtils::sync_state_with_parent(el); });
  context->decodebins_.push_back(std::move(decodebin));
}

bool HTTPSDPDec::to_shmdata(std::string uri) {
  if (uri.find("http://") == 0) {
    is_dataurisrc_ = false;
    souphttpsrc_.mute("souphttpsrc");
    uri_ = std::move(uri);
  } else {
    is_dataurisrc_ = true;
    souphttpsrc_.mute("dataurisrc");
    uri_ = std::string("data:application/sdp;base64," + uri);
  }
  on_error_.clear();
  uri_to_shmdata();
  return true;
}

void HTTPSDPDec::uri_to_shmdata() {
  destroy_httpsdpdec();
  prune_tree(".shmdata.writer");
  init_httpsdpdec();
  g_object_set_data(
      G_OBJECT(sdpdemux_.get_raw()), "on-error-gsource", (gpointer)on_error_.back().get());
  debug("httpsdpdec: to_shmdata set uri %", uri_);
  if (!is_dataurisrc_)  // for souphttpsrc
    g_object_set(G_OBJECT(souphttpsrc_.get_raw()), "location", uri_.c_str(), nullptr);
  else  // for dataurisrc
    g_object_set(G_OBJECT(souphttpsrc_.get_raw()), "uri", uri_.c_str(), nullptr);
  gst_bin_add_many(
      GST_BIN(gst_pipeline_->get_pipeline()), souphttpsrc_.get_raw(), sdpdemux_.get_raw(), nullptr);
  gst_element_link(souphttpsrc_.get_raw(), sdpdemux_.get_raw());
  gst_pipeline_->play(true);
}

}  // namespace switcher
