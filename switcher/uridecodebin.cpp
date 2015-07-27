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

#include "./scope-exit.hpp"
#include "./quiddity-command.hpp"
#include "./uridecodebin.hpp"
#include "./gst-utils.hpp"
#include "./shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    Uridecodebin,
    "urisrc",
    "URI/URL Player",
    "network",
    "writer",
    "URI decoding to shmdatas",
    "LGPL",
    "Nicolas Bouillot");

Uridecodebin::Uridecodebin(const std::string &):
    gst_pipeline_(std2::make_unique<GstPipeliner>(
        [this](GstMessage *msg){
          this->bus_async(msg);
        },
        nullptr)),
    custom_props_(std::make_shared<CustomPropertyHelper>()) {
}

void Uridecodebin::bus_async(GstMessage *msg){
  if (GST_MESSAGE_TYPE(msg) != GST_MESSAGE_EOS)
    return;
  if (loop_)
    gst_pipeline_->seek(0);
}

bool Uridecodebin::init() {  
  if (!GstUtils::make_element("uridecodebin", &uridecodebin_))
    return false;
  uri_spec_ =
      custom_props_->make_string_property("uri",
                                          "URI To Be Redirected Into Shmdata(s)",
                                          "",
                                          (GParamFlags) G_PARAM_READWRITE,
                                          Uridecodebin::set_uri,
                                          Uridecodebin::get_uri,
                                          this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            uri_spec_, "uri", "URI");
  loop_prop_ =
      custom_props_->make_boolean_property("loop",
                                           "loop media",
                                           (gboolean) FALSE,
                                           (GParamFlags) G_PARAM_READWRITE,
                                           Uridecodebin::set_loop,
                                           Uridecodebin::get_loop, this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            loop_prop_, "loop", "Looping");
  return true;
}

void Uridecodebin::init_uridecodebin() {
  if (!GstUtils::make_element("uridecodebin", &uridecodebin_)) {
    g_warning("cannot create uridecodebin");
    return;
  }
  // discard_next_uncomplete_buffer_ = false;
  rtpgstcaps_ =
      gst_caps_from_string("application/x-rtp, media=(string)application");
  g_signal_connect(G_OBJECT(uridecodebin_),
                   "pad-added",
                   (GCallback) Uridecodebin::uridecodebin_pad_added_cb,
                   (gpointer) this);
  g_signal_connect(G_OBJECT(uridecodebin_),
                   "unknown-type",
                   (GCallback) Uridecodebin::unknown_type_cb,
                   (gpointer) this);
  g_signal_connect(G_OBJECT(uridecodebin_),
                   "autoplug-continue",
                   (GCallback) Uridecodebin::autoplug_continue_cb,
                   (gpointer) this);
  g_signal_connect(G_OBJECT(uridecodebin_),
                   "autoplug-select",
                   (GCallback) Uridecodebin::autoplug_select_cb,
                   (gpointer) this);
  g_object_set(G_OBJECT(uridecodebin_),
               // "ring-buffer-max-size",(guint64)200000000,
               // "download",TRUE,
               // "use-buffering",TRUE,
               // "ring-buffer-max-size", 4294967295,
               "expose-all-streams", TRUE,
               "async-handling", TRUE,
               // "buffer-duration",9223372036854775807,
               nullptr);
}

void Uridecodebin::destroy_uridecodebin() {
  gst_pipeline_ = std2::make_unique<GstPipeliner>(
      [this](GstMessage *msg){
        this->bus_async(msg);
      },
      nullptr);
  clean_on_error_command();
  prune_tree(".shmdata.writer.");
}

void Uridecodebin::clean_on_error_command() {
  if (on_error_command_ != nullptr) {
    delete on_error_command_;
    on_error_command_ = nullptr;
  }
}

void
Uridecodebin::unknown_type_cb(GstElement *bin,
                              GstPad *pad,
                              GstCaps *caps, gpointer user_data) {
  Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  g_warning("Uridecodebin unknown type: %s (%s)\n",
            gst_caps_to_string(caps), gst_element_get_name(bin));
  context->pad_to_shmdata_writer(context->gst_pipeline_->get_pipeline(), pad);
}

gboolean sink_factory_filter(GstPluginFeature *feature, gpointer data) {
  GstCaps *caps = (GstCaps *) data;
  if (!GST_IS_ELEMENT_FACTORY(feature))
    return FALSE;
  const GList *static_pads =
      gst_element_factory_get_static_pad_templates(GST_ELEMENT_FACTORY
                                                   (feature));
  int not_any_number = 0;
  for (GList *item = (GList *) static_pads; item; item = item->next) {
    GstStaticPadTemplate *padTemplate = (GstStaticPadTemplate *) item->data;
    GstPadTemplate *pad = gst_static_pad_template_get(padTemplate);
    GstCaps *padCaps = gst_pad_template_get_caps(pad);
    if (!gst_caps_is_any(padCaps))
      not_any_number++;
  }
  if (not_any_number == 0)
    return FALSE;
  if (!gst_element_factory_list_is_type
      (GST_ELEMENT_FACTORY(feature), GST_ELEMENT_FACTORY_TYPE_DECODABLE))
    return FALSE;
  if (!gst_element_factory_can_sink_all_caps(GST_ELEMENT_FACTORY(feature), caps))
    return FALSE;
  return TRUE;
}

int Uridecodebin::autoplug_continue_cb(GstElement * /*bin */ ,
                                       GstPad * /*pad */ ,
                                       GstCaps *caps,
                                       gpointer /*user_data */ ) {
  GList *list = gst_registry_feature_filter(gst_registry_get(),
                                            (GstPluginFeatureFilter)
                                            sink_factory_filter,
                                            FALSE, caps);
  int length = g_list_length(list);
  gst_plugin_feature_list_free(list);
  if (length == 0)
    return 0;
  return 1;
}

int Uridecodebin::autoplug_select_cb(GstElement * /*bin */ ,
                                     GstPad * /*pad */ ,
                                     GstCaps *caps,
                                     GstElementFactory *factory,
                                     gpointer /*user_data */ ) {
  g_debug("uridecodebin autoplug select %s, (factory %s)",
          gst_caps_to_string(caps), GST_OBJECT_NAME(factory));
  //     typedef enum {
  //   GST_AUTOPLUG_SELECT_TRY,
  //   GST_AUTOPLUG_SELECT_EXPOSE,
  //   GST_AUTOPLUG_SELECT_SKIP
  // } GstAutoplugSelectResult;
  if (g_strcmp0(GST_OBJECT_NAME(factory), "rtpgstdepay") == 0)
    return 1;                 // expose
  return 0;                   // try
}

void Uridecodebin::release_buf(void *user_data) {
  GstBuffer *buf = static_cast<GstBuffer *>(user_data);
  gst_buffer_unref(buf);
}

void Uridecodebin::pad_to_shmdata_writer(GstElement *bin, GstPad *pad) {
  // detecting type of media
  std::string padname;
  {  // determining padname
    GstCaps *padcaps = gst_pad_get_current_caps(pad);
    On_scope_exit{gst_caps_unref(padcaps);};
    gchar *padcapsstr = gst_caps_to_string(padcaps);
    On_scope_exit{g_free(padcapsstr);};
    if (0 == g_strcmp0("ANY", padcapsstr)) {
      padname = "custom";
    } else {
      padname = gst_structure_get_name(gst_caps_get_structure(padcaps, 0));
    }
  }
  g_debug("uridecodebin new pad name is %s", padname.c_str());
  GstElement *shmdatasink = nullptr;
  GstUtils::make_element("shmdatasink", &shmdatasink);
  gst_bin_add(GST_BIN(bin), shmdatasink);
  GstPad *sinkpad = gst_element_get_static_pad(shmdatasink, "sink");
  On_scope_exit{gst_object_unref(sinkpad);};
  if (GST_PAD_LINK_OK != gst_pad_link(pad, sinkpad))
    g_warning("pad link failed in decodebin-to-shmdata");

  // giving a name to the stream
  gchar **padname_splitted = g_strsplit_set(padname.c_str(), "/", -1);
  On_scope_exit{g_strfreev(padname_splitted);};

  // counting
  auto count = counter_.get_count(padname_splitted[0]);
  std::string media_name = std::string(padname_splitted[0]) + "-" + std::to_string(count);
  g_debug("uridecodebin: new media %s\n", media_name.c_str());
  std::string shmpath = make_file_name(media_name);
  g_object_set(G_OBJECT(shmdatasink), "socket-path", shmpath.c_str(), nullptr);
  shm_subs_.emplace_back(
      std2::make_unique<GstShmdataSubscriber>(
          shmdatasink,
          [this, shmpath]( const std::string &caps){
            this->graft_tree(".shmdata.writer." + shmpath,
                             ShmdataUtils::make_tree(caps,
                                                     ShmdataUtils::get_category(caps),
                                                     0));
          },
          [this, shmpath](GstShmdataSubscriber::num_bytes_t byte_rate){
            this->graft_tree(".shmdata.writer." + shmpath + ".byte_rate",
                             data::Tree::make(byte_rate));
          }));
  GstUtils::sync_state_with_parent(shmdatasink);
}

gboolean Uridecodebin::gstrtpdepay_buffer_probe_cb(GstPad */*pad */,
                                                   GstMiniObject */*mini_obj */ ,
                                                   gpointer user_data) {
  Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  if (context->discard_next_uncomplete_buffer_ == true) {
    g_debug("discarding uncomplete custom frame due to a network loss");
    context->discard_next_uncomplete_buffer_ = false;
    return FALSE;  // drop buffer
  }
  return TRUE;  // pass buffer
}

gboolean Uridecodebin::gstrtpdepay_event_probe_cb(GstPad * /*pad */ ,
                                                  GstEvent *event,
                                                  gpointer user_data) {
  Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
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

void
Uridecodebin::uridecodebin_pad_added_cb(GstElement *object,
                                        GstPad *pad,
                                        gpointer user_data)
{
  Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  GstCaps *newcaps = gst_pad_get_current_caps(pad);
  On_scope_exit{gst_caps_unref(newcaps);};
  if (gst_caps_can_intersect(context->rtpgstcaps_, newcaps)) {
    // asking rtpbin to send an event when a packet is lost (do-lost property)
    GstUtils::set_element_property_in_bin(object,
                                          "gstrtpbin",
                                          "do-lost",
                                          TRUE);
    g_message("custom rtp stream found");
    GstElement *rtpgstdepay = nullptr;
    GstUtils::make_element("rtpgstdepay", &rtpgstdepay);

    // adding a probe for discarding uncomplete packets
    GstPad *depaysrcpad = gst_element_get_static_pad(rtpgstdepay, "src");
    On_scope_exit{gst_object_unref(depaysrcpad);};
    // FIXME
    // gst_pad_add_buffer_probe(depaysrcpad,
    //                          G_CALLBACK
    //                          (Uridecodebin::gstrtpdepay_buffer_probe_cb),
    //                          context);
    gst_bin_add(GST_BIN(context->gst_pipeline_->get_pipeline()), rtpgstdepay);
    GstPad *sinkpad = gst_element_get_static_pad(rtpgstdepay, "sink");
    On_scope_exit{gst_object_unref(sinkpad);};
    // adding a probe for handling loss messages from rtpbin
    // FIXME
    // gst_pad_add_event_probe(sinkpad,
    //                         (GCallback) Uridecodebin::gstrtpdepay_event_probe_cb,
    //                         context);
    GstUtils::check_pad_link_return(gst_pad_link(pad, sinkpad));
    GstPad *srcpad = gst_element_get_static_pad(rtpgstdepay, "src");
    GstUtils::sync_state_with_parent(rtpgstdepay);
    On_scope_exit{gst_object_unref (srcpad);};
    gst_element_get_state(rtpgstdepay, nullptr, nullptr,
                          GST_CLOCK_TIME_NONE);
    context->pad_to_shmdata_writer(context->gst_pipeline_->get_pipeline(), srcpad);
  } else {
    context->pad_to_shmdata_writer(context->gst_pipeline_->get_pipeline(), pad);
  }
}

bool Uridecodebin::to_shmdata() {
  if (uri_.empty()) {
    g_warning("no uri to decode");
    return false;
  }
  counter_.reset_counter_map();
  destroy_uridecodebin();
  init_uridecodebin();
  g_debug("to_shmdata set uri %s", uri_.c_str());
  g_object_set(G_OBJECT(uridecodebin_), "uri", uri_.c_str(), nullptr);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), uridecodebin_);
  gst_pipeline_->play(true);
  //GstUtils::sync_state_with_parent(uridecodebin_);
  return true;
}

void Uridecodebin::set_loop(gboolean loop, void *user_data) {
  Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  context->loop_ = loop;
}

gboolean Uridecodebin::get_loop(void *user_data) {
  Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  return context->loop_;
}

void Uridecodebin::set_uri(const gchar *value, void *user_data) {
  Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  context->uri_ = value;
  context->to_shmdata();
  //context->query_position_and_length();
  context->custom_props_->notify_property_changed(context->uri_spec_);
}

const gchar *Uridecodebin::get_uri(void *user_data) {
  Uridecodebin *context = static_cast<Uridecodebin *>(user_data);
  return context->uri_.c_str();
}

}  // namespace switcher
