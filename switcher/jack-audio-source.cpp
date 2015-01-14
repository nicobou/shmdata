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

#include "./jack-audio-source.hpp"
#include <gst/gst.h>
#include "./gst-utils.hpp"
namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(JackAudioSource,
                                     "Jack Audio Device",
                                     "audio",
                                     "get audio from jack",
                                     "LGPL",
                                     "jacksrc",
                                     "Nicolas Bouillot");

JackAudioSource::JackAudioSource(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper>()) {
}

bool JackAudioSource::init_gpipe() {
  if (false == make_elements())
    return false;
  init_startable(this);

  num_channels_spec_ =
      custom_props_->make_int_property("channels",
                                       "number of channels",
                                       1,
                                       64,
                                       num_channels_,
                                       (GParamFlags) G_PARAM_READWRITE,
                                       JackAudioSource::set_num_channels,
                                       JackAudioSource::get_num_channels,
                                       this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            num_channels_spec_, "channels", "Channels");

    connect_physical_port_prop_ =
        custom_props_->make_boolean_property("connect",
                                             "automatically connect to physical ports",
                                             (gboolean) TRUE,
                                             (GParamFlags)G_PARAM_READWRITE,
                                             JackAudioSource::set_connect_phys,
                                             JackAudioSource::get_connect_phys,
                                             this);
    install_property_by_pspec(custom_props_->get_gobject(),
                              connect_physical_port_prop_,
                              "connect",
                              "automatically connect to physical ports");
  
  client_name_ = g_strdup(get_name().c_str());

  client_name_spec_ =
      custom_props_->make_string_property("jack-client-name",
                                          "the jack client name",
                                          "switcher",
                                          (GParamFlags) G_PARAM_READWRITE,
                                          JackAudioSource::set_client_name,
                                          JackAudioSource::get_client_name,
                                          this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            client_name_spec_,
                            "client-name", "Client Name");

  // g_object_set (G_OBJECT (jackaudiosrc_),
  //   "is-live", TRUE,
  //   "samplesperbuffer",512,
  //   nullptr);

  return true;
}

JackAudioSource::~JackAudioSource() {
  GstUtils::clean_element(jackaudiosrc_);  
  GstUtils::clean_element(audioconvert_);  
  GstUtils::clean_element(capsfilter_);  
  GstUtils::clean_element(jackaudiosrc_bin_);  
  if (nullptr != client_name_)
    g_free(client_name_);
}

bool JackAudioSource::start() {
  make_elements();
  set_raw_audio_element(jackaudiosrc_bin_);
  disable_property("channels");
  disable_property("client-name");
  disable_property("connect");
  return true;
}

bool JackAudioSource::stop() {
  enable_property("channels");
  enable_property("client-name");
  enable_property("connect");
  reset_bin();
  return true;
}

bool JackAudioSource::make_elements() {
  if (!GstUtils::make_element("jackaudiosrc", &jackaudiosrc_))
    return false;
  if (!GstUtils::make_element("audioconvert", &audioconvert_))
    return false;
  if (!GstUtils::make_element("capsfilter", &capsfilter_))
    return false;
  if (!GstUtils::make_element("bin", &jackaudiosrc_bin_))
    return false;

  if (!connect_phys_)
    g_object_set(G_OBJECT(jackaudiosrc_), "connect", 0, nullptr);
  
  // using caps compatible with L16 RTP payload
  GstCaps *caps = gst_caps_new_simple("audio/x-raw-int",
                                      "width", G_TYPE_INT, 16,
                                      "depth", G_TYPE_INT, 16,
                                      "signed", G_TYPE_BOOLEAN, TRUE,
                                      "endianness", G_TYPE_INT, 4321,
                                      "channels", G_TYPE_INT,
                                      num_channels_,
                                      nullptr);
  g_object_set(G_OBJECT(capsfilter_), "caps", caps, nullptr);
  gst_caps_unref(caps);

  g_object_set(G_OBJECT(jackaudiosrc_),
               "client-name", client_name_, nullptr);

  gst_bin_add_many(GST_BIN(jackaudiosrc_bin_),
                   jackaudiosrc_, audioconvert_, capsfilter_, nullptr);

  gst_element_link_many(jackaudiosrc_, audioconvert_, capsfilter_, nullptr);

  GstPad *src_pad = gst_element_get_static_pad(capsfilter_, "src");
  GstPad *ghost_srcpad = gst_ghost_pad_new(nullptr, src_pad);
  gst_pad_set_active(ghost_srcpad, TRUE);
  gst_element_add_pad(jackaudiosrc_bin_, ghost_srcpad);
  gst_object_unref(src_pad);

  return true;
}

void JackAudioSource::set_num_channels(const gint value, void *user_data) {
  JackAudioSource *context = static_cast<JackAudioSource *>(user_data);
  context->num_channels_ = value;
  GObjectWrapper::notify_property_changed(context->gobject_->get_gobject(),
                                          context->num_channels_spec_);
}

gint JackAudioSource::get_num_channels(void *user_data) {
  JackAudioSource *context = static_cast<JackAudioSource *>(user_data);
  return context->num_channels_;
}

void JackAudioSource::set_client_name(const gchar *value, void *user_data) {
  JackAudioSource *context = static_cast<JackAudioSource *>(user_data);
  if (nullptr != context->client_name_)
    g_free(context->client_name_);
  context->client_name_ = g_strdup(value);
  context->custom_props_->
      notify_property_changed(context->client_name_spec_);
}

const gchar *JackAudioSource::get_client_name(void *user_data) {
  JackAudioSource *context = static_cast<JackAudioSource *>(user_data);
  return context->client_name_;
}

gboolean JackAudioSource::get_connect_phys(void *user_data) {
  JackAudioSource *context = static_cast<JackAudioSource *>(user_data);
  if (!context->connect_phys_)
    return FALSE;
  return TRUE;
}

void JackAudioSource::set_connect_phys(gboolean connect, void *user_data) {
  JackAudioSource *context = static_cast<JackAudioSource *>(user_data);
  if (connect)
    context->connect_phys_ = true;
  else
    context->connect_phys_ = false;
  context->custom_props_->notify_property_changed(context->connect_physical_port_prop_);
}
}  // namespace swittcher
