/*
 * This file is part of switcher-video-snapshot.
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

#include "video_snapshot.hpp"

#include <filesystem>

#include "switcher/infotree/json-serializer.hpp"
#include "switcher/utils/file-utils.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(VideoSnapshot,
                                     "videosnapshot",
                                     "Video Snapshot",
                                     "Take snapshot from a video shmdata",
                                     "LGPL",
                                     "Nicolas Bouillot");

const std::string VideoSnapshot::kConnectionSpec(R"(
{
"follower":
  [
    {
      "label": "video",
      "description": "Video stream",
      "can_do": ["video/x-raw"]
    }
  ]
}
)");

VideoSnapshot::VideoSnapshot(quiddity::Config&& conf)
    : Quiddity(
          std::forward<quiddity::Config>(conf),
          {kConnectionSpec,
           [this](const std::string& shmpath, claw::sfid_t) { return on_shmdata_connect(shmpath); },
           [this](claw::sfid_t sfid) { return on_shmdata_disconnect(); }}),
      snap_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "shot",
          [this](bool val) {
            if (!val || !pmanage<MPtr(&property::PBag::enabled)>(snap_id_)) return false;
            g_object_set(G_OBJECT(valve_), "drop", FALSE, nullptr);
            return true;
          },
          [this]() { return false; },
          "Take Snapshot",
          "Triger the saving of a frame",
          false)),
      last_image_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "last_image",
          nullptr,
          [this]() { return last_image_; },
          "Last image written",
          "Path of the last jpeg file written",
          last_image_)),
      img_dir_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "imgdir",
          [this](const std::string& val) {
            img_dir_ = val;
            if (!img_dir_.empty() && img_dir_.back() != '/') img_dir_ += '/';
            auto file_prepared = fileutils::prepare_writable_dir(val);
            if (!file_prepared.first) {
              warning("error preparing % directory for writing: %", val, file_prepared.second);
              return false;
            }
            return true;
          },
          [this]() { return img_dir_; },
          "Image Directory",
          "Directory where to store jpeg files to be produced. If empty, the "
          "path will be /tmp/<videosnapshot name>_xxxxx.jpg",
          img_dir_)),
      img_name_(std::filesystem::path(get_nickname()).filename()),
      img_name_id_(pmanage<MPtr(&property::PBag::make_string)>(
          "imgname",
          [this](const std::string& val) {
            img_name_ = val;
            return true;
          },
          [this]() { return img_name_; },
          "Image Name",
          "Name of the jpeg files to be produced. You can use printf format "
          "for numbering files (for instance %05d). If empty, the name will "
          "take the input shmdata name with option file number and jpg "
          "extension",
          img_name_)),
      num_files_id_(pmanage<MPtr(&property::PBag::make_bool)>(
          "num_files",
          [this](const bool num_files) {
            num_files_ = num_files;
            return true;
          },
          [this]() { return num_files_; },
          "Number Files",
          "Automatically number produced files",
          num_files_)),
      jpg_quality_id_(pmanage<MPtr(&property::PBag::make_unsigned_int)>(
          "quality",
          [this](const unsigned int val) {
            jpg_quality_ = val;
            return true;
          },
          [this]() { return jpg_quality_; },
          "JPEG quality",
          "Quality of the produced jpeg image",
          jpg_quality_,
          0,
          100)),
      gst_pipeline_(std::make_unique<gst::Pipeliner>(
          [this](GstMessage* msg) {
            if (msg->type != GST_MESSAGE_ELEMENT) return;
            const GstStructure* s = gst_message_get_structure(msg);
            auto name = std::string(gst_structure_get_name(s));
            if (name != "GstMultiFileSink") return;
            on_new_file(gst_structure_get_string(s, "filename"));
          },
          nullptr)) {
  pmanage<MPtr(&property::PBag::disable)>(snap_id_, property::PBag::disabledWhenDisconnectedMsg);
}

void VideoSnapshot::make_gst_pipeline(const std::string& shmpath) {
  // checking if videoconvert can be multi-threaded
  auto nb_threads_gst_element = gst::utils::get_nthreads_property_value();
  std::string nthreads_property{};
  if (nb_threads_gst_element > 0) {
    nthreads_property += "n-threads=" + std::to_string(nb_threads_gst_element);
  }

  std::string extension = ".jpg";
  if (num_files_ && std::string::npos == img_name_.find('%')) extension = "_%d.jpg";

  // make pipeline string description
  std::string description(
      std::string("shmdatasrc copy-buffers=true do-timestamp=true socket-path=") + shmpath +
      " ! queue ! valve ! videoconvert " + nthreads_property +
      " ! jpegenc quality=" + std::to_string(jpg_quality_) +
      " ! multifilesink post-messages=true location=\"" + img_dir_ + img_name_ + extension + "\"");

  GError* err = nullptr;
  GstElement* bin = gst_parse_bin_from_description(description.c_str(), TRUE, &err);
  if (err != nullptr) {
    this->error("%", std::string(err->message));
    g_error_free(err);
    return;
  }
  valve_ = gst::utils::get_first_element_from_factory_name(GST_BIN(bin), "valve");
  g_object_set(G_OBJECT(valve_), "drop", TRUE, nullptr);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), bin);
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);
  gst_pipeline_->play(true);
}

void VideoSnapshot::on_new_file(const std::string& filename) {
    std::unique_lock<std::mutex> lock(mtx_);
    g_object_set(G_OBJECT(valve_), "drop", TRUE, nullptr);
    {
      auto image_lock = pmanage<MPtr(&property::PBag::get_lock)>(last_image_id_);
      last_image_ = filename;
    }
    pmanage<MPtr(&property::PBag::notify)>(last_image_id_);
    message("image % written", filename);
}

bool VideoSnapshot::on_shmdata_disconnect() {
  std::unique_lock<std::mutex> lock(mtx_);
  pmanage<MPtr(&property::PBag::disable)>(snap_id_, property::PBag::disabledWhenDisconnectedMsg);
  gst_pipeline_.reset();
  return true;
}

bool VideoSnapshot::on_shmdata_connect(const std::string& shmpath) {
  std::unique_lock<std::mutex> lock(mtx_);
  pmanage<MPtr(&property::PBag::enable)>(snap_id_);
  make_gst_pipeline(shmpath);
  return true;
}

}  // namespace quiddities
}  // namespace switcher
