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

#include "./decodebin-to-shmdata.hpp"
#include "./scope-exit.hpp"
#include <glib/gprintf.h>

namespace switcher {
DecodebinToShmdata::DecodebinToShmdata(
    GstPipeliner *gpipe,
    on_configure_t on_gstshm_configure):
    decodebin_("decodebin"),
    gpipe_(gpipe),
    on_gstshm_configure_(on_gstshm_configure){
  // set async property
  auto set_prop = std::bind(g_object_set,
                            std::placeholders::_1,
                            "async-handling",
                            TRUE,
                            nullptr);
  decodebin_.g_invoke(std::move(set_prop));
  // pad added callback
  auto pad_added = std::bind(GstUtils::g_signal_connect_function,
                             std::placeholders::_1,
                             "pad-added",
                             (GCallback) DecodebinToShmdata::on_pad_added,
                             (gpointer) this);
  cb_ids_.push_back(decodebin_.g_invoke_with_return<gulong>
                    (std::move(pad_added)));
  // autoplug callback
  auto autoplug = std::bind(GstUtils::g_signal_connect_function,
                            std::placeholders::_1,
                            "autoplug-select",
                            (GCallback)
                            DecodebinToShmdata::on_autoplug_select,
                            (gpointer) this);
  cb_ids_.push_back(decodebin_.g_invoke_with_return<gulong>
                    (std::move(autoplug)));
}

DecodebinToShmdata::~DecodebinToShmdata() {
  std::unique_lock<std::mutex> lock(thread_safe_);
  // // for (auto &it: cb_ids_)
  // //   if (0 != it)
  // // decodebin_.g_invoke (std::bind (g_signal_handler_disconnect,
  // // std::placeholders::_1,
  // // it));
}

void DecodebinToShmdata::on_pad_added(GstElement *object,
                                 GstPad *pad,
                                 gpointer user_data) {
  DecodebinToShmdata *context =
      static_cast<DecodebinToShmdata *>(user_data);
  std::unique_lock<std::mutex> lock(context->thread_safe_);
  GstCaps *rtpcaps =
      gst_caps_from_string("application/x-rtp, media=(string)application");
  On_scope_exit {
    gst_caps_unref(rtpcaps);
  };
  GstCaps *padcaps = gst_pad_get_current_caps(pad);
  On_scope_exit {
    gst_caps_unref(padcaps);
  };
  if (gst_caps_can_intersect(rtpcaps, padcaps)) {
      // asking rtpbin to send an event when a packet is lost (do-lost property)
      GstUtils::set_element_property_in_bin(object, "gstrtpbin", "do-lost",
                                            TRUE);
      g_message("custom rtp stream found");
      GstElement *rtpgstdepay;
      GstUtils::make_element("rtpgstdepay", &rtpgstdepay);
      // adding a probe for discarding uncomplete packets
      // FIXME (test if drop buffer is necessary)
      // GstPad *depaysrcpad = gst_element_get_static_pad(rtpgstdepay, "src");
      // gst_pad_add_buffer_probe(depaysrcpad,
      //                          G_CALLBACK
      //                          (DecodebinToShmdata::gstrtpdepay_buffer_probe_cb),
      //                          context);
      // gst_object_unref(depaysrcpad);
      // was this: gst_bin_add (GST_BIN (context->bin_), rtpgstdepay);
      gst_bin_add(GST_BIN(GST_ELEMENT_PARENT(object)), rtpgstdepay);
      GstPad *sinkpad = gst_element_get_static_pad(rtpgstdepay, "sink");
      // adding a probe for handling loss messages from rtpbin
      // gst_pad_add_event_probe(sinkpad, (GCallback)
      //                         DecodebinToShmdata::gstrtpdepay_event_probe_cb,
      //                         context);
      GstUtils::check_pad_link_return(gst_pad_link(pad, sinkpad));
      gst_object_unref(sinkpad);
      GstPad *srcpad = gst_element_get_static_pad(rtpgstdepay, "src");
      GstUtils::sync_state_with_parent(rtpgstdepay);
      gst_element_get_state(rtpgstdepay, nullptr, nullptr,
                          GST_CLOCK_TIME_NONE);
    context->pad_to_shmdata_writer(GST_ELEMENT_PARENT(object), srcpad);
    gst_object_unref(srcpad);
    return;
  }
  context->pad_to_shmdata_writer(GST_ELEMENT_PARENT(object), pad);
}

int DecodebinToShmdata::on_autoplug_select(GstElement * /*bin */ ,
                                           GstPad *pad,
                                           GstCaps *caps,
                                           GstElementFactory *factory,
                                           gpointer /*user_data */ ) {
  //     typedef enum {
  //   GST_AUTOPLUG_SELECT_TRY,
  //   GST_AUTOPLUG_SELECT_EXPOSE,
  //   GST_AUTOPLUG_SELECT_SKIP
  // } GstAutoplugSelectResult;
  if (g_strcmp0(GST_OBJECT_NAME(factory), "rtpgstdepay") == 0) {
    int return_val = 1;
    const GValue *val =
        gst_structure_get_value(gst_caps_get_structure(gst_pad_get_current_caps(pad), 0),
                                "caps");
    gsize taille = 256;
    guchar *string_caps = g_base64_decode(g_value_get_string(val),
                                          &taille);
    gchar *string_caps_char = g_strdup_printf("%s", string_caps);
    if (g_str_has_prefix(string_caps_char, "audio/")
        || g_str_has_prefix(string_caps_char, "video/")
        || g_str_has_prefix(string_caps_char, "image/"))
      return_val = 0;
    g_free(string_caps_char);
    g_free(string_caps);
    return return_val;        // expose
  }
  return 0;                   // try
}

gboolean DecodebinToShmdata::gstrtpdepay_buffer_probe_cb(GstPad * /*pad */ ,
                                                         GstMiniObject *
                                                         /*mini_obj */ ,
                                                         gpointer user_data) {
  DecodebinToShmdata *context =
      static_cast<DecodebinToShmdata *>(user_data);
  std::unique_lock<std::mutex> lock(context->thread_safe_);
  if (true == context->discard_next_uncomplete_buffer_) {
    g_debug("discarding uncomplete custom frame due to a network loss");
    context->discard_next_uncomplete_buffer_ = false;
    return FALSE;             // drop buffer
  }
  return TRUE;                // pass buffer
}

gboolean DecodebinToShmdata::gstrtpdepay_event_probe_cb(GstPad * /*pad */ ,
                                                        GstEvent *event,
                                                        gpointer user_data){
  DecodebinToShmdata *context =
      static_cast<DecodebinToShmdata *>(user_data);
  std::unique_lock<std::mutex> lock(context->thread_safe_);
  if (GST_EVENT_TYPE(event) == GST_EVENT_CUSTOM_DOWNSTREAM) {
    const GstStructure *s;
    s = gst_event_get_structure(event);
    // g_debug ("event probed (%s)\n", gst_structure_get_name (s));
    if (gst_structure_has_name(s, "GstRTPPacketLost"))
      context->discard_next_uncomplete_buffer_ = true;
    return FALSE;
  }
  return TRUE;
}

void DecodebinToShmdata::pad_to_shmdata_writer(GstElement *bin, GstPad *pad){
  // looking for type of media
  std::string padname;
  {
    GstCaps *padcaps = gst_pad_get_current_caps(pad);
    if (nullptr == padcaps){
      padname = "custom";
    } else {
      On_scope_exit {
        gst_caps_unref(padcaps);
      };
      gchar *stringcaps = gst_caps_to_string(padcaps);
      On_scope_exit {
        g_free(stringcaps);
      };
      if (0 == g_strcmp0("ANY", stringcaps))
        padname = "custom";
      else
        padname = gst_structure_get_name(gst_caps_get_structure(padcaps, 0));
    }
  }
  g_debug("decodebin-to-shmdata new pad name is %s\n", padname.c_str());
  GstElement *shmdatasink;
  GstUtils::make_element("shmdatasink", &shmdatasink);
  gst_bin_add(GST_BIN(bin), shmdatasink);
  // probing eos
  GstPad *sinkpad = gst_element_get_static_pad(shmdatasink, "sink");
  On_scope_exit{gst_object_unref(sinkpad);};
  if (nullptr == main_pad_)
    main_pad_ = sinkpad;  // saving first pad for looping
  // FIXME
  // gst_pad_add_event_probe(srcpad, (GCallback) eos_probe_cb, this);
  if (GST_PAD_LINK_OK != gst_pad_link(pad, sinkpad))
    g_warning("pad link failed in decodebin-to-shmdata");
  std::string media_name(media_label_);
  //std::string media_name("custom");
  {  // giving a name to the stream
    gchar **padname_splitted = g_strsplit_set(padname.c_str(), "/", -1);
    On_scope_exit {g_strfreev(padname_splitted);};
    if (nullptr == padname_splitted[0])
      media_name = "unknown";
    else
      media_name = padname_splitted[0];
    g_debug("decodebin-to-shmdata: new media of type %s \n", media_name.c_str());
  }
  if(!on_gstshm_configure_)
    g_warning("decodebin-to-shmdata has no configuration function for shmdatasink");
  else
    on_gstshm_configure_(shmdatasink, media_name, media_label_);
  GstUtils::sync_state_with_parent(shmdatasink);
}

gboolean DecodebinToShmdata::eos_probe_cb(GstPad *pad,
                                 GstEvent *event,
                                 gpointer user_data) {
  DecodebinToShmdata *context =
      static_cast<DecodebinToShmdata *>(user_data);
  std::unique_lock<std::mutex> lock(context->thread_safe_);
  if (GST_EVENT_TYPE(event) == GST_EVENT_EOS) {
    if (context->main_pad_ == pad)
      // FIXME
      // GstUtils::g_idle_add_full_with_context(context->
      //                                        gpipe_->get_g_main_context(),
      //                                        G_PRIORITY_DEFAULT_IDLE,
      //                                        (GSourceFunc)
      //                                        DecodebinToShmdata::rewind,
      //                                        (gpointer) context, nullptr);
    return FALSE;
  }
  if (GST_EVENT_TYPE(event) == GST_EVENT_FLUSH_START ||
      GST_EVENT_TYPE(event) == GST_EVENT_FLUSH_STOP)
    return FALSE;
  return TRUE;
}

void DecodebinToShmdata::invoke(std::function<void(GstElement *)> command) {
  std::unique_lock<std::mutex> lock(thread_safe_);
  decodebin_.invoke(command);
}

gboolean DecodebinToShmdata::rewind(gpointer user_data) {
  DecodebinToShmdata *context =
      static_cast<DecodebinToShmdata *>(user_data);
  GstQuery *query;
  gboolean res;
  query = gst_query_new_segment(GST_FORMAT_TIME);
  res = gst_element_query(context->gpipe_->get_pipeline(), query);
  gdouble rate = -2.0;
  gint64 start_value = -2.0;
  gint64 stop_value = -2.0;
  if (res) {
    gst_query_parse_segment(query, &rate, nullptr, &start_value,
                            &stop_value);
    // g_print ("rate = %f start = %"GST_TIME_FORMAT" stop = %"GST_TIME_FORMAT"\n",
    //        rate,
    //        GST_TIME_ARGS (start_value),
    //        GST_TIME_ARGS (stop_value));
  } else {
    g_debug("duration query failed...");
  }
  gst_query_unref(query);
  gboolean ret;
  ret =
      gst_element_seek(GST_ELEMENT(gst_pad_get_parent(context->main_pad_)),
                       rate, GST_FORMAT_TIME,
                       (GstSeekFlags) (GST_SEEK_FLAG_FLUSH |
                                       GST_SEEK_FLAG_ACCURATE),
                       // | GST_SEEK_FLAG_SKIP
                       // | GST_SEEK_FLAG_KEY_UNIT,  // using key unit is breaking synchronization
                       GST_SEEK_TYPE_SET,
                       0.0 * GST_SECOND,
                       GST_SEEK_TYPE_NONE, GST_CLOCK_TIME_NONE);
  if (!ret)
    g_debug("looping error\n");
  g_debug("finish looping");
  return FALSE;
}

void DecodebinToShmdata::set_media_label(std::string label) {
  g_debug("new media label %s", label.c_str());
  media_label_ = std::move(label);
}

}  // namespace switcher
