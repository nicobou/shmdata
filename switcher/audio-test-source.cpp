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

#include "./audio-test-source.hpp"
#include <gst/gst.h>
#include "./gst-utils.hpp"
namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(AudioTestSource,
                                     "Sine",
                                     "audio",
                                     "Creates audio test signals",
                                     "LGPL",
                                     "audiotestsrc",
                                     "Nicolas Bouillot");

AudioTestSource::AudioTestSource(const std::string &):
    audiotestsrc_("audiotestsrc") {
}

bool AudioTestSource::init_gpipe() {
  init_startable(this);
  return make_audiotestsrc();
}

bool AudioTestSource::make_audiotestsrc() {
  uninstall_property("volume");
  uninstall_property("freq");
  uninstall_property("samplesperbuffer");
  uninstall_property("wave");

  UGstElem audiotest("audiotestsrc");
  if (!audiotest)
    return false;

  g_object_set(G_OBJECT(audiotest.get_raw()), "is-live", TRUE, nullptr);

  if (audiotestsrc_) {
    GstUtils::apply_property_value(G_OBJECT(audiotestsrc_.get_raw()),
                                   G_OBJECT(audiotest.get_raw()), "volume");
    GstUtils::apply_property_value(G_OBJECT(audiotestsrc_.get_raw()),
                                   G_OBJECT(audiotest.get_raw()), "freq");
    GstUtils::apply_property_value(G_OBJECT(audiotestsrc_.get_raw()),
                                   G_OBJECT(audiotest.get_raw()), "samplesperbuffer");
    GstUtils::apply_property_value(G_OBJECT(audiotestsrc_.get_raw()),
                                   G_OBJECT(audiotest.get_raw()), "wave");
  }
  else
    g_object_set(G_OBJECT(audiotest.get_raw()), "samplesperbuffer", 512, nullptr);

  audiotestsrc_ = std::move(audiotest);

  // registering
  install_property(G_OBJECT(audiotestsrc_.get_raw()), "volume", "volume", "Volume");
  install_property(G_OBJECT(audiotestsrc_.get_raw()), "freq", "freq", "Frequency");
  // install_property (G_OBJECT (audiotestsrc_.get_raw()),
  //        "samplesperbuffer",
  //        "samplesperbuffer",
  //        "Samples Per Buffer");
  install_property(G_OBJECT(audiotestsrc_.get_raw()), "wave", "wave", "Signal Form");

  return true;
}

AudioTestSource::~AudioTestSource() {
}

bool AudioTestSource::start() {
  set_raw_audio_element(audiotestsrc_.get_raw());
  return true;
}

bool AudioTestSource::stop() {
  make_audiotestsrc();
  unset_raw_audio_element();
  return true;
}
}
