/*
 * This file is part of switcher-gst.
 *
 * switcher-gst is free software; you can redistribute it and/or
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

#include "gst-quid.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstQuid,
                                     "gst",
                                     "Gst Quid",
                                     "video",
                                     "",
                                     "Plugin implementing a simple Gst pipeline",
                                     "LGPL",
                                     "Hantz-Carly F. Vius");

GstQuid::GstQuid(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      started_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "started_id_",
          [this](bool val) { return play(val); },
          [this]() { return started_; },
          "Pipeline state",
          "This property tells whether the pipeline is started or not",
          started_)) {
  debug("GstQuid::GstQuid");
}

bool GstQuid::remake_elements() {
  debug("GstQuid::remake_elements");
  if (!(gst::UGstElem::renew(src_) && gst::UGstElem::renew(sink_))) {
    error("GstQuid::remake_elements: Could not renew GStreamer elements");
    return false;
  }

  return true;
}

bool GstQuid::play(bool b) {
  if (started_ == b) {
    debug("GstQuid::play: Nothing to do.");
    return true;
  }

  bool res;
  if (b) {
    res = start_pipeline();
  } else {
    res = stop_pipeline();
  }

  if (!res) {
    debug("GstQuid::play: Couldn't change pipeline state.");
    return false;
  }

  started_ = b;
  return true;
}

bool GstQuid::stop_pipeline() {
  if (pipeline_ == nullptr) {
    debug("GstQuid::stop_pipeline: Pipeline not initialized. Nothing to do.");
    return true;
  }

  pipeline_->play(false);
  debug("GstQuid::stop_pipeline: Pipeline stopped.");

  return true;
}

bool GstQuid::start_pipeline() {
  if (pipeline_ != nullptr) {
    debug("GstQuid::create_pipeline: Pipeline already created. Stopping");
    pipeline_->play(false);
  }

  pipeline_ = std::make_unique<gst::Pipeliner>(nullptr, nullptr);
  if (!remake_elements()) {
    error("GstQuid::create_pipeline: Could not remake GStreamer elements");
    return false;
  }

  debug("GstQuid::create_pipeline: Pipeline elements made");

  gst_bin_add_many(GST_BIN(pipeline_->get_pipeline()), src_.get_raw(), sink_.get_raw(), nullptr);

  gst_element_link(src_.get_raw(), sink_.get_raw());

  if (!static_cast<bool>(pipeline_.get())) {
    error("GstQuid::create_pipeline: Pipeline not found");
    return false;
  }

  pipeline_->play(true);

  debug("GstQuid::create_pipeline: Pipeline started");
  return true;
}

}  // namespace quiddities
}  // namespace switcher
