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
#include "switcher/gst-utils.hpp"
#include "switcher/std2.hpp"
#include "switcher/shmdata-utils.hpp"
#include "switcher/scope-exit.hpp"
#include "./video-test-source.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(
    VideoTestSource,
    "videotestsrc",
    "Video Pattern",
    "video",
    "writer",
    "Creates a test video stream",
    "LGPL",
    "Nicolas Bouillot");

VideoTestSource::VideoTestSource(const std::string &):
    custom_props_(std::make_shared<CustomPropertyHelper>()),
    gst_pipeline_(std2::make_unique<GstPipeliner>(nullptr, nullptr)){
  init_startable(this);
}

bool VideoTestSource::init() {
  if(!videotestsrc_ || !shmdatasink_)
    return false;
  shmpath_ = make_file_name("video");
  codecs_ = std2::make_unique<GstVideoCodec>(static_cast<Quiddity *>(this),
                                             custom_props_.get(),
                                             shmpath_);
  g_object_set(G_OBJECT(videotestsrc_.get_raw()), "is-live", TRUE, nullptr);
  g_object_set(G_OBJECT(shmdatasink_.get_raw()), "socket-path", shmpath_.c_str(), nullptr);
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   shmdatasink_.get_raw(), videotestsrc_.get_raw(),
                   nullptr);
  gst_element_link(videotestsrc_.get_raw(), shmdatasink_.get_raw());
  install_property(G_OBJECT(videotestsrc_.get_raw()),
                   "pattern", "pattern", "Video Pattern");
  return true;
}

bool VideoTestSource::start() {
  if (!gst_pipeline_)
    return false;
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
                         data::Tree::make(std::to_string(byte_rate)));
      });
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()),
               "async-handling", TRUE,
               nullptr);
  gst_pipeline_->play(true);
  codecs_->start();
  reinstall_property(G_OBJECT(videotestsrc_.get_raw()),
                     "pattern", "pattern", "Video Pattern");
  return true;
}

bool VideoTestSource::stop() {
  shm_sub_.reset(nullptr);
  prune_tree(".shmdata.writer." + shmpath_);
  if (!UGstElem::renew(videotestsrc_, {"is-live", "pattern"})
      || !UGstElem::renew(shmdatasink_, {"socket-path"})) {
    g_warning("error initializing gst element for videotestsrc");
    gst_pipeline_.reset();
    return false;
  }
  gst_pipeline_ = std2::make_unique<GstPipeliner>(nullptr, nullptr);
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   shmdatasink_.get_raw(), videotestsrc_.get_raw(),
                   nullptr);
  gst_element_link(videotestsrc_.get_raw(), shmdatasink_.get_raw());
  codecs_->stop();
  reinstall_property(G_OBJECT(videotestsrc_.get_raw()),
                     "pattern", "pattern", "Video Pattern");
  return true;
}

}  // namespace switcher
