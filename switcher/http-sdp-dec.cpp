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

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    HTTPSDPDec,
    "HTTP/SDP Player",
    "network",
    "decode an sdp-described stream deliver through http and make shmdatas",
    "LGPL",
    "httpsdpdec",
    "Nicolas Bouillot");

HTTPSDPDec::HTTPSDPDec():
    souphttpsrc_("souphttpsrc"),
    sdpdemux_("sdpdemux"),
    on_error_(std2::make_unique<GSourceWrapper>(
        [&](){uri_to_shmdata();},
        retry_delay_)) {
}

bool HTTPSDPDec::init_gpipe() {
  if (!souphttpsrc_ || !sdpdemux_)
    return false;

  install_method("To Shmdata",
                 "to_shmdata",
                 "get raw streams from an sdp description distributed over http and write them to shmdatas",
                 "success or fail",
                 Method::make_arg_description("URL",
                                              "url",
                                              "the url to the sdp file",
                                              nullptr),
                 (Method::method_ptr)to_shmdata_wrapped,
                 G_TYPE_BOOLEAN,
                 Method::make_arg_type_description(G_TYPE_STRING,
                                                   nullptr), this);
  return true;
}

void HTTPSDPDec::init_httpsdpdec() {
  UGstElem::renew(souphttpsrc_);
  UGstElem::renew(sdpdemux_);
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
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  clean_on_error();
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  clear_shmdatas();
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  reset_bin();
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  reset_counter_map();
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
}

void
HTTPSDPDec::on_new_element_in_sdpdemux(GstBin */*bin*/,
                                       GstElement *element,
                                       gpointer /*user_data*/) {
  // FIXME add that in uridecodebin
  g_object_set(G_OBJECT(element), "ntp-sync", TRUE, nullptr);
}

void HTTPSDPDec::clean_on_error() {
  GSourceWrapper::uptr on_error =
      std2::make_unique<GSourceWrapper>([&](){uri_to_shmdata();},
                                        retry_delay_);
  std::swap(on_error, on_error_);
}

void HTTPSDPDec::httpsdpdec_pad_added_cb(GstElement * /*object */ ,
                                         GstPad *pad, gpointer user_data) {
  HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
  GstPipeliner *gpipe = static_cast<GstPipeliner *>(user_data);
  std::unique_ptr<DecodebinToShmdata> decodebin = 
      std2::make_unique<DecodebinToShmdata>(gpipe);

  decodebin->
      invoke_with_return<gboolean>(std::bind(gst_bin_add,
                                             GST_BIN(context->get_bin()),
                                             std::placeholders::_1));
  
  // GstPad *sinkpad = gst_element_get_static_pad (decodebin, "sink");
  auto get_pad = std::bind(gst_element_get_static_pad,
                           std::placeholders::_1,
                           "sink");
  GstPad *sinkpad =
      decodebin->invoke_with_return<GstPad *>(std::move(get_pad));
  On_scope_exit {gst_object_unref(GST_OBJECT(sinkpad));};
  
  GstUtils::check_pad_link_return(gst_pad_link(pad, sinkpad));
  decodebin->invoke(std::bind(GstUtils::sync_state_with_parent,
                              std::placeholders::_1));
  context->decodebins_.push_back(std::move(decodebin));
}

void HTTPSDPDec::source_setup_cb(GstElement */*httpsdpdec */,
                                 GstElement *source,
                                 gpointer /*user_data */) {
  g_debug("source %s %s\n", GST_ELEMENT_NAME(source),
          G_OBJECT_CLASS_NAME(G_OBJECT_GET_CLASS(source)));
}

gboolean HTTPSDPDec::to_shmdata_wrapped(gpointer uri, gpointer user_data) {
  HTTPSDPDec *context = static_cast<HTTPSDPDec *>(user_data);
  if (context->to_shmdata((char *)uri))
    return TRUE;
  else
    return FALSE;
}

bool HTTPSDPDec::to_shmdata(std::string uri) {
  uri_ = uri;
  uri_to_shmdata();
  return true;
}

void HTTPSDPDec::uri_to_shmdata() {
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  destroy_httpsdpdec();
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  reset_bin();
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  init_httpsdpdec();
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  g_print("%s %p\n", __FUNCTION__, on_error_.get());
  g_object_set_data(G_OBJECT(sdpdemux_.get_raw()),
                    "on-error-gsource",
                    (gpointer)on_error_.get());
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  g_debug("httpsdpdec: to_shmdata set uri %s", uri_.c_str());
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  g_object_set(G_OBJECT(souphttpsrc_.get_raw()),
               "location", uri_.c_str(), nullptr);
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  gst_bin_add_many(GST_BIN(get_bin()),
                   souphttpsrc_.get_raw(),
                   sdpdemux_.get_raw(),
                   nullptr);
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  gst_element_link(souphttpsrc_.get_raw(), sdpdemux_.get_raw());
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  GstUtils::sync_state_with_parent(souphttpsrc_.get_raw());
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
  GstUtils::sync_state_with_parent(sdpdemux_.get_raw());
  g_print("%s line %d\n", __FUNCTION__, __LINE__);
}
}
