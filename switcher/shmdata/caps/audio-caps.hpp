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
#include "../../utils/safe-bool-idiom.hpp"

namespace switcher {
namespace shmdata {
namespace caps {

/**
 * A class for parsing audio caps in Shmdata. If parsed with success, several informations can be
 * obtained like the sample format, the number of channels and the samplerate specified in the
 * original caps.
 *
 * Once constructed, the caps can be tested like a boolean value, thanks to the SafeBoolIdiom. Any
 * error message can be retrieved with the error_msg method.
 */
class AudioCaps : public SafeBoolIdiom {
 public:
  AudioCaps() = delete;
  /**
   * Construct an AudioCaps object.
   * \param caps String formated caps.
   **/
  AudioCaps(const std::string& caps);

  /**
   * Get the caps.
   * \return Caps.
   */
  std::string get() const;
  /**
   * Get the number of channels for the audio Shmdata.
   * \return Number of channels.
   **/
  unsigned int channels() const;
  /**
   * Test if audio format is unsigned.
   * \return True if unsigned, false otherwise.
   **/
  bool is_unsigned() const;
  /**
   * Test if audio format is signed.
   * \return True if signed, false otherwise.
   **/
  bool is_signed() const;
  /**
   * Test if audio format is float.
   * \return True if float, false otherwise.
   **/
  bool is_float() const;
  /**
   * Get the sample rate.
   * \return Samplerate.
   */
  unsigned int samplerate() const;
  /**
   * Get format size in number of bits.
   * \return Size.
   */
  unsigned int format_size() const;
  /**
   * Get format size in number of bytes.
   * \return Size.
   */
  unsigned int format_size_in_bytes() const;
  /**
   * Get the parsing error message.
   * \return Message.
   */
  std::string error_msg() const;
  /**
   * Set a new sample rate.
   **/
  void set_samplerate(unsigned int rate);
  /**
   * Set the format as float.
   **/
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

}  // namespace caps
}  // namespace shmdata
}  // namespace switcher
#endif
