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

#ifndef __SWITCHER_AUDIO_CAPS_H__
#define __SWITCHER_AUDIO_CAPS_H__

#include <string>
#include "./safe-bool-idiom.hpp"

namespace switcher {

class AudioCaps : public SafeBoolIdiom {
 public:
  AudioCaps() = delete;
  AudioCaps(const std::string& caps);

  std::string get() const;
  unsigned int channels() const;
  bool is_unsigned() const;
  bool is_signed() const;
  bool is_float() const;
  unsigned int samplerate() const;
  unsigned int format_size() const;
  unsigned int format_size_in_bytes() const;
  std::string error_msg() const;

  void set_samplerate(unsigned int rate);
  void set_float();

 private:
  bool safe_bool_idiom() const;
  bool is_unsigned_{false};
  bool is_signed_{false};
  bool is_float_{false};
  bool is_le_{false};
  bool is_be_{false};
  unsigned int samplerate_{0};
  unsigned int format_size_{0};
  unsigned int format_size_in_bytes_{0};
  unsigned int channels_{0};
  std::string error_{};
};

}  // namespace switcher
#endif
