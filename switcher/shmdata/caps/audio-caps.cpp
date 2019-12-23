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

#include "./audio-caps.hpp"
#include <gst/gst.h>
#include "../../utils/scope-exit.hpp"
#include "../../utils/string-utils.hpp"

namespace switcher {

bool AudioCaps::safe_bool_idiom() const {
  return (is_unsigned_ || is_signed_ || is_float_) && samplerate_ != 0 && format_size_ != 0 &&
         channels_ != 0 && (is_le_ || is_be_);
}

AudioCaps::AudioCaps(const std::string& str_caps) {
  GstCaps* caps = gst_caps_from_string(str_caps.c_str());
  On_scope_exit {
    if (nullptr != caps) gst_caps_unref(caps);
  };

  GstStructure* s = gst_caps_get_structure(caps, 0);
  if (nullptr == s) {
    error_ = "Cannot get gst structure from caps";
    return;
  }

  auto format = gst_structure_get_string(s, "format");
  if (!format) {
    error_ = "Cannot get format from shmdata description";
    return;
  }

  int samplerate = 0;
  if (!gst_structure_get_int(s, "rate", &samplerate)) {
    error_ = "Cannot get rate from shmdata description";
    return;
  }
  samplerate_ = static_cast<unsigned int>(samplerate);

  int channels = 0;
  if (!gst_structure_get_int(s, "channels", &channels)) {
    error_ = "Cannot get channels from shmdata description";
    return;
  }
  channels_ = static_cast<unsigned int>(channels);

  std::string str_format(format);
  auto format_type = str_format.substr(0, 1);
  if (format_type == "U")
    is_unsigned_ = true;
  else if (format_type == "S")
    is_signed_ = true;
  else if (format_type == "F")
    is_float_ = true;
  str_format = str_format.substr(1, str_format.size());
  if (StringUtils::starts_with(str_format, "8"))
    format_size_ = 8;
  else if (StringUtils::starts_with(str_format, "16"))
    format_size_ = 16;
  else if (StringUtils::starts_with(str_format, "32"))
    format_size_ = 32;
  else if (StringUtils::starts_with(str_format, "64"))
    format_size_ = 64;
  else {
    format_size_ = 0;
    error_ = "audio formats size unknown: " + str_format;
  }
  format_size_in_bytes_ = format_size_ / 8;

  if (str_format.size() < 2) return;  // and safe bool will return false
  auto endian = std::string(str_format.end() - 2, str_format.end());
  if (endian == "LE") is_le_ = true;
  if (endian == "BE") is_be_ = true;
}

unsigned int AudioCaps::channels() const { return channels_; }

bool AudioCaps::is_unsigned() const { return is_unsigned_; }

bool AudioCaps::is_signed() const { return is_signed_; }

bool AudioCaps::is_float() const { return is_float_; }

unsigned int AudioCaps::samplerate() const { return samplerate_; }

unsigned int AudioCaps::format_size() const { return format_size_; }

unsigned int AudioCaps::format_size_in_bytes() const { return format_size_in_bytes_; }

std::string AudioCaps::error_msg() const { return error_; }

void AudioCaps::set_samplerate(unsigned int rate) { samplerate_ = rate; }

void AudioCaps::set_float() {
  is_float_ = true;
  is_signed_ = false;
  is_unsigned_ = false;
  format_size_ = 32;
  format_size_in_bytes_ = 4;
}

std::string AudioCaps::get() const {
  std::string format_type = "S";
  if (is_unsigned_) format_type = "U";
  if (is_float_) format_type = "F";
  std::string endian = "LE";
  if (is_be_) endian = "BE";
  return "audio/x-raw, format=(string)" + format_type + std::to_string(format_size_) + endian +
         ", layout=(string)interleaved, rate=(int)" + std::to_string(samplerate_) +
         ", channels=(int)" + std::to_string(channels_) +
         ", channel-mask=(bitmask)0x0000000000000000";
}
}  // namespace switcher
