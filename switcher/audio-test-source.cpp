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

#include <gst/gst.h>
#include <thread>
#include "./shmdata-utils.hpp" 
#include "./audio-test-source.hpp"
#include "./std2.hpp"
#include "./information-tree-basic-serializer.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    AudioTestSource,
    "audiotestsrc",
    "Sine",
    "audio",
    "writer",
    "Creates audio test signals",
    "LGPL",
    "Nicolas Bouillot");

AudioTestSource::AudioTestSource(const std::string &):
  gst_pipeline_(std2::make_unique<GstPipeliner>(nullptr, nullptr)){
    init_startable(this);
  }

bool AudioTestSource::init() {
  shmpath_ = make_file_name("audio");  // FIXME make_file name should work in ctor...
  g_object_set(G_OBJECT(audiotestsrc_.get_raw()), "is-live", TRUE, nullptr);
  g_object_set(G_OBJECT(audiotestsrc_.get_raw()), "samplesperbuffer", 512, nullptr);
  g_object_set(G_OBJECT(shmdatasink_.get_raw()),
               "socket-path", shmpath_.c_str(),
               nullptr);
  // registering
  install_property(G_OBJECT(audiotestsrc_.get_raw()), "volume", "volume", "Volume");
  install_property(G_OBJECT(audiotestsrc_.get_raw()), "freq", "freq", "Frequency");
  install_property(G_OBJECT(audiotestsrc_.get_raw()), "wave", "wave", "Signal Form");
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   audiotestsrc_.get_raw(),
                   shmdatasink_.get_raw(),
                   nullptr);
  gst_element_link(audiotestsrc_.get_raw(), shmdatasink_.get_raw());
  if (!audiotestsrc_) {
    g_warning("audiotestsrc creation failed");
    return false;
  }
  if (!shmdatasink_) {
    g_warning("shmdatasink creation failed");
    return false;
  }
  return true;
}

bool AudioTestSource::start() {
  shm_sub_ = std2::make_unique<GstShmdataSubscriber>(
      shmdatasink_.get_raw(),
      [this]( const std::string &caps){
        this->graft_tree(".shmdata.writer." + shmpath_,
                         ShmdataUtils::make_tree(caps,
                                                 ShmdataUtils::get_category(caps),
                                                 0));
      },
      [this](GstShmdataSubscriber::num_bytes_t byte_rate){
        this->graft_tree(".shmdata.writer." + shmpath_ + ".byte_rate",
                         data::Tree::make(byte_rate));
      });
  gst_pipeline_->play(true);
  return true;
}

bool AudioTestSource::stop() {
  shm_sub_.reset();
  this->prune_tree(".shmdata.writer." + shmpath_);
  gst_pipeline_->play(false);
  return true;
}
}
