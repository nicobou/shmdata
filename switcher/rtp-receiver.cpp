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

#include "./rtp-session2.hpp"
#include "./rtp-receiver.hpp"
#include "./scope-exit.hpp"

namespace switcher { 

RTPReceiver::RTPReceiver(RtpSession2 *session,
                         const std::string &rtpshmpath):
    session_(session),
    rtpshmpath_(rtpshmpath),
    shmdatasrc_(gst_element_factory_make("shmdatasrc", nullptr)),
    typefind_(gst_element_factory_make("typefind", nullptr)),
    decodebin_(
        session_->gst_pipeline_.get(),
        [this](GstElement *el, const std::string &media_type, const std::string &media_label){
          // TODO
          g_print("%s %d\n", __FUNCTION__, __LINE__);
          g_object_set(G_OBJECT(el), "socket-path", "/tmp/truc", nullptr);
          //configure_shmdatasink(el, media_type, media_label);
        }){
  if (nullptr == shmdatasrc_ || nullptr == typefind_){
    g_warning("RTPReceiver failled to create GStreamer element");
    return;
  }
          g_print("%s %d\n", __FUNCTION__, __LINE__);
  // monitoring rtp-session new pads for received rtp packet
  g_signal_connect(session_->rtpsession_, "pad-added", G_CALLBACK(on_pad_added), this);
  g_signal_connect(session_->rtpsession_, "request-pt-map", (GCallback)request_pt_map, this);
  // configuring shmdatasrc and typefind
  g_signal_connect(typefind_, "have-type", G_CALLBACK(on_caps), this);
          g_print("%s %d\n", __FUNCTION__, __LINE__);
  g_object_set(G_OBJECT(shmdatasrc_),
               "socket-path", rtpshmpath_.c_str(),
               "copy-buffers", TRUE,
               nullptr);
          g_print("%s %d\n", __FUNCTION__, __LINE__);
  gst_bin_add_many(GST_BIN(session_->gst_pipeline_->get_pipeline()),
                   shmdatasrc_, typefind_, nullptr);
          g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (!gst_element_link(shmdatasrc_, typefind_)){
    g_warning("RTPReceiver: shmdatasrc not linked with typefind");
    return;
  }
          g_print("%s %d\n", __FUNCTION__, __LINE__);
  GstUtils::sync_state_with_parent(shmdatasrc_);
  GstUtils::sync_state_with_parent(typefind_);
          g_print("%s %d\n", __FUNCTION__, __LINE__);
}

RTPReceiver::~RTPReceiver(){
          g_print("%s %d\n", __FUNCTION__, __LINE__);
  if (shmdatasrc_)
    GstUtils::clean_element(shmdatasrc_);
  if (rtp_sink_pad_)
    gst_element_release_request_pad(session_->rtpsession_, rtp_sink_pad_);
  if (shmsrc_caps_)
    gst_caps_unref(shmsrc_caps_);
  g_print("%s %d\n", __FUNCTION__, __LINE__);
  
}

void RTPReceiver::on_caps(GstElement *typefind,
                          guint /*probability */ ,
                          GstCaps *caps,
                          gpointer user_data){
  RTPReceiver *context = static_cast<RTPReceiver *>(user_data);
          g_print("%s %d\n", __FUNCTION__, __LINE__);
  std::string rtp_id;
  if (nullptr != context->shmsrc_caps_)
    g_warning("BUG in RTPReceiver %s %d", __FUNCTION__, __LINE__);
  context->shmsrc_caps_ = gst_caps_copy(caps);
  g_print("lllllllllllllllllll %s\n", gst_caps_to_string(context->shmsrc_caps_ ));
          
  // link the payloader with the rtpbin
  { context->rtp_sink_pad_ =
        gst_element_get_request_pad(context->session_->rtpsession_, "recv_rtp_sink_%u");
    On_scope_exit {gst_object_unref(context->rtp_sink_pad_);}; 
    GstPad *srcpad = gst_element_get_static_pad(context->typefind_, "src");
    On_scope_exit {gst_object_unref(srcpad);}; 
          g_print("%s %d\n", __FUNCTION__, __LINE__);
    if (gst_pad_link(srcpad, context->rtp_sink_pad_) != GST_PAD_LINK_OK)
      g_warning("(BUG) failed to link typefind to rtpbin");
    gchar *rtp_sink_pad_name = gst_pad_get_name(context->rtp_sink_pad_);
    On_scope_exit{g_free(rtp_sink_pad_name);};
    gchar **rtpsession_array = g_strsplit_set(rtp_sink_pad_name, "_", 0);
    On_scope_exit{g_strfreev(rtpsession_array);};
    rtp_id = std::string(rtpsession_array[3]);
          g_print("%s %d\n", __FUNCTION__, __LINE__);
    context->rtp_src_pad_prefix_ = std::string("recv_rtp_src_") + rtp_id;
    // TODO send RTCP
  }
}

void RTPReceiver::on_pad_added(GstElement *object,
                               GstPad *pad,
                               gpointer user_data) {
  RTPReceiver *context = static_cast<RTPReceiver *>(user_data);
          g_print("%s %d\n", __FUNCTION__, __LINE__);
  gchar *name_cstr = gst_pad_get_name(pad);
  On_scope_exit{g_free(name_cstr);};
  auto name = std::string(name_cstr);
  g_print("--------------- %s -- %s \n",
          context->rtp_src_pad_prefix_.c_str(),
          std::string(name).c_str());
  if (0 != name.compare(0, 13,"recv_rtp_src_"))
    return;
  if (0 == std::string(name).compare(0, context->rtp_src_pad_prefix_.size(),
                                     context->rtp_src_pad_prefix_)) {
          g_print("%s %d\n", __FUNCTION__, __LINE__);
    if(!context->decodebin_.invoke_with_return<bool>([&](GstElement *el) {
          if (!gst_bin_add(GST_BIN(context->session_->gst_pipeline_->get_pipeline()), el)) {
            g_warning("RTPReceiver decodebin cannot be added to pipeline");
            return false;
          }
          GstPad *sinkpad = gst_element_get_static_pad(el, "sink");
          On_scope_exit {gst_object_unref(sinkpad);}; 
          if (gst_pad_link(pad, sinkpad) != GST_PAD_LINK_OK){
            g_warning("(BUG) failed to link rtpbin to decodebin");
            return false;
          }
          g_print("%s %d\n", __FUNCTION__, __LINE__);
          GstUtils::sync_state_with_parent(el);
          g_print("%s %d\n", __FUNCTION__, __LINE__);
          return true;
        })){
      g_warning("BUG when adding decodebin");
    }
  }
}

GstCaps *RTPReceiver::request_pt_map(GstElement *sess, guint session, guint pt, gpointer user_data){
  RTPReceiver *context = static_cast<RTPReceiver *>(user_data);
  g_print("+++++++++++++++++++++++ %s %u %u\n", __FUNCTION__, session, pt);
  return context->shmsrc_caps_;
}


}  // namespace switcher
