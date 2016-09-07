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

#ifndef __SWITCHER_GST_PIXEL_FORMAT_CONVERTER_H__
#define __SWITCHER_GST_PIXEL_FORMAT_CONVERTER_H__

#include <memory>
#include "switcher/gst-pipeliner.hpp"
#include "switcher/safe-bool-idiom.hpp"
#include "switcher/unique-gst-element.hpp"

namespace switcher {
class GstPixelFormatConverter : public SafeBoolIdiom {
 public:
  using uptr = std::unique_ptr<GstPixelFormatConverter>;
  GstPixelFormatConverter(const std::string& shmpath_to_convert,
                          const std::string& shmpath_converted,
                          const std::string& format_name);
  GstPixelFormatConverter() = delete;
  ~GstPixelFormatConverter() = default;
  GstPixelFormatConverter(const GstPixelFormatConverter&) = delete;
  GstPixelFormatConverter& operator=(const GstPixelFormatConverter&) = delete;

  static bool can_sink_caps(const std::string& caps);

  // for external shm subscribers...
  GstElement* get_shmsink() { return shm_converted_.get_raw(); }
  GstElement* get_shmsrc() { return shmsrc_.get_raw(); }

 private:
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  UGstElem shmsrc_{"shmdatasrc"};
  UGstElem queue_codec_element_{"queue"};
  UGstElem color_space_codec_element_{"videoconvert"};
  UGstElem capsfilter_{"capsfilter"};
  UGstElem shm_converted_{"shmdatasink"};
  static const size_t default_initial_shmsize_{67108864};

  // safe bool idiom:
  bool is_valid_{false};

  std::string get_caps_str(const std::string& format_name) const;
  bool safe_bool_idiom() const final { return is_valid_; }
};

}  // namespace switcher
#endif
