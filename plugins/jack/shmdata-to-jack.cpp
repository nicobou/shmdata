/*
 * This file is part of switcher-jack.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#include "switcher/gst-utils.hpp"
#include "switcher/scope-exit.hpp"
#include "./shmdata-to-jack.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(ShmdataToJack,
                                     "Shmdata to Jack",
                                     "audio",
                                     "send audio data to jack",
                                     "LGPL",
                                     "jacksink2",
                                     "Nicolas Bouillot");
bool ShmdataToJack::init_gpipe() {
  jack_client_ = JackClient(get_name().c_str());
  if (!jack_client_) {
    g_warning("JackClient cannot be instancied");
    return false;
  }
  if (false == make_elements())
    return false;
  init_startable(this);
  install_property(G_OBJECT(volume_), "volume", "volume", "Volume");
  install_property(G_OBJECT(volume_), "mute", "mute", "Mute");
  return true;
}

ShmdataToJack::ShmdataToJack(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper>()) {
}

void
ShmdataToJack::on_handoff_cb(GstElement *object,
                             GstBuffer *buf,
                             GstPad *pad,
                             gpointer user_data) {
  ShmdataToJack *context = static_cast<ShmdataToJack *>(user_data);
  GstCaps *caps = gst_pad_get_negotiated_caps(pad);
  if (nullptr == caps)
    return;
  On_scope_exit {gst_caps_unref(caps);};
  gchar *string_caps = gst_caps_to_string(caps);
  On_scope_exit {if (nullptr != string_caps) g_free(string_caps);};
  g_print("on handoff, negotiated caps is %s\n", string_caps);
  const GValue *val =
      gst_structure_get_value(gst_caps_get_structure(caps, 0),
                              "channels");
  const int channels = g_value_get_int(val);
  g_print("on handoff, channel number %d\n", channels);
  if (channels != context->channels_) {
    context->channels_ = channels;
    jack_client_t *cl = context->jack_client_.get_raw();
    // unregistering previous ports
    for (auto &it: context->output_ports_) {
      jack_port_unregister(cl, it);
    }
    // registering new ports
    for (int i = 0; i < channels; i++) {
      context->output_ports_.
          emplace_back(jack_port_register(cl,
                                          std::string("output_" + std::to_string(i)).c_str(),
                                          JACK_DEFAULT_AUDIO_TYPE,
                                          JackPortIsOutput,
                                          0));
    }
  }
}

bool ShmdataToJack::make_elements() {
  GError *error = nullptr;
  std::string description(std::string(" audioconvert ! audioresample ! volume ! ")
                          + " audioconvert ! "
                          + " capsfilter caps=\"audio/x-raw-float, "
                          "endianness=(int)1234, width=(int)32, rate="
                          + std::to_string(jack_client_.get_sample_rate())
                          + "\" !" 
                          + " fakesink silent=true signal-handoffs=true sync=false");
  GstElement *jacksink = gst_parse_bin_from_description(description.c_str(), TRUE, &error);
  if (error != nullptr) {
    g_warning("%s", error->message);
    g_error_free(error);
    return false;
  }
  g_object_set(G_OBJECT(jacksink), "async-handling", TRUE, nullptr);
  GstElement *volume = GstUtils::get_first_element_from_factory_name(GST_BIN(jacksink),
                                                                     "volume");
  if (nullptr != volume_) {
    GstUtils::apply_property_value(G_OBJECT(volume_),
                                   G_OBJECT(volume),
                                   "volume");
    GstUtils::apply_property_value(G_OBJECT(volume_),
                                   G_OBJECT(volume),
                                   "mute");
  }
  if (handoff_handler_ > 0 && nullptr != fakesink_)
    g_signal_handler_disconnect(G_OBJECT(fakesink_),
                                handoff_handler_);
  GstElement *fakesink = GstUtils::get_first_element_from_factory_name(GST_BIN(jacksink),
                                                                       "fakesink");
  handoff_handler_ = g_signal_connect(fakesink,
                                      "handoff",
                                      (GCallback)on_handoff_cb,
                                      this);
  if (nullptr != jacksink_) 
    GstUtils::clean_element(jacksink_);
  jacksink_ = jacksink;
  volume_ = volume;
  fakesink_ = fakesink;
  return true;
}

bool ShmdataToJack::start() {
  if (false == make_elements())
    return false;
  set_sink_element(jacksink_);
  reinstall_property(G_OBJECT(volume_),
                     "volume", "volume", "Volume");
  reinstall_property(G_OBJECT(volume_),
                     "mute", "mute", "Mute");
  return true;
}

bool ShmdataToJack::stop() {
  if (!make_elements())
    return false;
  reset_bin();
  reinstall_property(G_OBJECT(volume_),
                     "volume", "volume", "Volume");
  reinstall_property(G_OBJECT(volume_),
                     "mute", "mute", "Mute");
  return true;
}

void ShmdataToJack::on_shmdata_disconnect() {
  stop();
}

void ShmdataToJack::on_shmdata_connect(std::string /* shmdata_sochet_path */ ) {
  if (is_started()) {
    stop();
    set_sink_element_no_connect(jacksink_);
  }
}

bool ShmdataToJack::can_sink_caps(std::string caps) {
  return GstUtils::can_sink_caps("audioconvert", caps);
}

}
