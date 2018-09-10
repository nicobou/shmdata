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

#include "./file-decoder.hpp"
#include "./gst-utils.hpp"
#include "./scope-exit.hpp"
#include "./shmdata-utils.hpp"

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(FileDecoder,
                                     "filesrc",
                                     "file Player",
                                     "file",
                                     "writer",
                                     "File decoding to one or more shmdata",
                                     "LGPL",
                                     "Nicolas Bouillot");

FileDecoder::FileDecoder(quid::Config&& conf)
    : Quiddity(std::forward<quid::Config>(conf)),
      location_id_(pmanage<MPtr(&PContainer::make_string)>("location",
                                                           [this](const std::string& val) {
                                                             location_ = val;
                                                             return load_file(location_);
                                                           },
                                                           [this]() { return location_; },
                                                           "File location",
                                                           "Location of the file to decode",
                                                           "")),
      play_id_(pmanage<MPtr(&PContainer::make_bool)>("play",
                                                     [this](const bool& val) {
                                                       play_ = val;
                                                       if (gst_pipeline_)
                                                         gst_pipeline_->play(play_);
                                                       return true;
                                                     },
                                                     [this]() { return play_; },
                                                     "Play",
                                                     "Play/pause the player",
                                                     play_)),
      loop_id_(pmanage<MPtr(&PContainer::make_bool)>("loop",
                                                     [this](const bool& val) {
                                                       loop_ = val;
                                                       if (gst_pipeline_)
                                                         gst_pipeline_->loop(loop_);
                                                       return true;
                                                     },
                                                     [this]() { return loop_; },
                                                     "Looping",
                                                     "Loop media",
                                                     loop_)),
      decompress_streams_id_(
          pmanage<MPtr(&PContainer::make_bool)>("decompress",
                                                [this](const bool& val) {
                                                  decompress_streams_ = val;
                                                  return true;
                                                },
                                                [this]() { return decompress_streams_; },
                                                "Decompress",
                                                "Decompress received streams",
                                                decompress_streams_)) {
  register_writer_suffix(".*");
}

bool FileDecoder::load_file(const std::string& path) {
  // cleaning previous
  shm_subs_.clear();
  counter_.reset_counter_map();
  gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr);
  gst_pipeline_->loop(loop_);
  decodebin_.reset();

  // creating new decoder
  filesrc_ = gst_element_factory_make("filesrc", nullptr);
  g_object_set(G_OBJECT(filesrc_), "location", path.c_str(), nullptr);
  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), filesrc_);
  decodebin_ = std::make_unique<DecodebinToShmdata>(
      gst_pipeline_.get(),
      [this](GstElement* el, const std::string& media_type, const std::string& media_label) {
        configure_shmdatasink(el, media_type, media_label);
      },
      [this]() { warning("discarding uncomplete custom frame due to a network loss"); },
      decompress_streams_);
  if (!decodebin_->invoke_with_return<gboolean>([this](GstElement* el) {
        return gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), el);
      })) {
    warning("decodebin cannot be added to pipeline");
  }
  GstPad* srcpad = gst_element_get_static_pad(filesrc_, "src");
  On_scope_exit { gst_object_unref(GST_OBJECT(srcpad)); };
  GstPad* sinkpad = decodebin_->invoke_with_return<GstPad*>(
      [](GstElement* el) { return gst_element_get_static_pad(el, "sink"); });
  On_scope_exit { gst_object_unref(GST_OBJECT(sinkpad)); };
  GstUtils::check_pad_link_return(gst_pad_link(srcpad, sinkpad));
  if (play_) gst_pipeline_->play(true);
  return true;
}

void FileDecoder::configure_shmdatasink(GstElement* element,
                                        const std::string& media_type,
                                        const std::string& media_label) {
  auto count = counter_.get_count(media_label + media_type);
  std::string media_name = media_type;
  if (count != 0) media_name.append("-" + std::to_string(count));
  std::string shmpath;
  if (media_label.empty())
    shmpath = make_shmpath(media_name);
  else
    shmpath = make_shmpath(media_label + "-" + media_name);

  g_object_set(G_OBJECT(element), "socket-path", shmpath.c_str(), nullptr);
  shm_subs_.emplace_back(std::make_unique<GstShmTreeUpdater>(
      this, element, shmpath, GstShmTreeUpdater::Direction::writer));
}

}  // namespace switcher
