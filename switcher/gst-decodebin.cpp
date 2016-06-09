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

#include "./gst-decodebin.hpp"
#include "./scope-exit.hpp"
#include "switcher/shmdata-utils.hpp"
#include "switcher/std2.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GstDecodebin,
                                     "decoder",
                                     "Decoder",
                                     "other",
                                     "writer/reader",
                                     "Generic shmdata decoder",
                                     "LGPL",
                                     "Nicolas Bouillot");

GstDecodebin::GstDecodebin(const std::string&)
    : gst_pipeline_(std2::make_unique<GstPipeliner>(nullptr, nullptr)),
      shmsrc_("shmdatasrc"),
      shmcntr_(static_cast<Quiddity*>(this)) {}

bool GstDecodebin::init() {
  if (!shmsrc_) return false;

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) {
        return this->on_shmdata_connect(shmpath);
      },
      [this](const std::string&) { return this->on_shmdata_disconnect(); },
      [this]() { return this->on_shmdata_disconnect(); },
      [this](const std::string& caps) { return this->can_sink_caps(caps); },
      1);
  return true;
}

bool GstDecodebin::on_shmdata_disconnect() {
  shmw_sub_.reset(nullptr);
  shmr_sub_.reset(nullptr);
  prune_tree(".shmdata.writer");
  if (!UGstElem::renew(shmsrc_)) return false;
  gst_pipeline_ = std2::make_unique<GstPipeliner>(nullptr, nullptr);
  counter_.reset_counter_map();

  return true;
}

void GstDecodebin::configure_shmdatasink(GstElement* element,
                                         const std::string& media_type,
                                         const std::string& media_label) {
  auto count = counter_.get_count(media_label + media_type);
  std::string media_name = media_type;
  if (count != 0) media_name.append("-" + std::to_string(count));
  std::string shmpath;
  if (media_label.empty())
    shmpath = make_file_name(media_name);
  else
    shmpath = make_file_name(media_label + "-" + media_name);

  g_object_set(G_OBJECT(element), "socket-path", shmpath.c_str(), nullptr);
  shmw_sub_ = std2::make_unique<GstShmdataSubscriber>(
      element,
      [this, shmpath](const std::string& caps) {
        this->graft_tree(
            ".shmdata.writer." + shmpath,
            ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), 0));
      },
      [this, shmpath](GstShmdataSubscriber::num_bytes_t byte_rate) {
        this->graft_tree(".shmdata.writer." + shmpath + ".byte_rate",
                         InfoTree::make(byte_rate));
      });
}

bool GstDecodebin::on_shmdata_connect(const std::string& shmpath) {
  // creating shmdata reader
  g_object_set(G_OBJECT(shmsrc_.get_raw()),
               "copy-buffers",
               TRUE,
               "socket-path",
               shmpath.c_str(),
               nullptr);
  shmr_sub_ = std2::make_unique<GstShmdataSubscriber>(
      shmsrc_.get_raw(),
      [this, shmpath](const std::string& caps) {
        this->graft_tree(
            ".shmdata.reader." + shmpath,
            ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), 0));
      },
      [this, shmpath](GstShmdataSubscriber::num_bytes_t byte_rate) {
        this->graft_tree(".shmdata.reader." + shmpath + ".byte_rate",
                         InfoTree::make(byte_rate));
      });

  // creating decodebin
  std::unique_ptr<DecodebinToShmdata> decodebin =
      std2::make_unique<DecodebinToShmdata>(
          gst_pipeline_.get(),
          [this](GstElement* el,
                 const std::string& media_type,
                 const std::string& media_label) {
            configure_shmdatasink(el, media_type, media_label);
          });
  // adding to pipeline
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), shmsrc_.get_raw());
  if (!decodebin->invoke_with_return<gboolean>([this](GstElement* el) {
        return gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), el);
      })) {
    g_warning("decodebin cannot be added to pipeline");
  }
  // get pads and link
  GstPad* pad = gst_element_get_static_pad(shmsrc_.get_raw(), "src");
  On_scope_exit { gst_object_unref(GST_OBJECT(pad)); };
  GstPad* sinkpad = decodebin->invoke_with_return<GstPad*>(
      [](GstElement* el) { return gst_element_get_static_pad(el, "sink"); });
  On_scope_exit { gst_object_unref(GST_OBJECT(sinkpad)); };
  if (!GstUtils::check_pad_link_return(gst_pad_link(pad, sinkpad)))
    return false;
  // set media label if relevant
  auto caps = gst_pad_get_allowed_caps(pad);
  if (0 < gst_caps_get_size(caps)) {
    On_scope_exit { gst_caps_unref(caps); };
    auto structure = gst_caps_get_structure(caps, 0);
    auto media_label = gst_structure_get_string(structure, "media-label");
    if (nullptr != media_label)
      decodebin->set_media_label(
          gst_structure_get_string(structure, "media-label"));
  }
  // save the decodebin
  decoder_ = std::move(decodebin);
  // start
  gst_pipeline_->play(true);
  return true;
}

bool GstDecodebin::can_sink_caps(const std::string& caps) {
  return GstUtils::can_sink_caps("decodebin", caps);
}

}  // namespace switcher
