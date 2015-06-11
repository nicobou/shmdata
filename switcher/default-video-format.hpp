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

#ifndef __SWITCHER_DEFAULT_VIDEO_FORMAT_H__
#define __SWITCHER_DEFAULT_VIDEO_FORMAT_H__

#include <memory>
#include "./quiddity.hpp"
#include "./custom-property-helper.hpp"

namespace switcher {
class DefaultVideoFormat {
 public:
  using uptr = std::unique_ptr<DefaultVideoFormat>;

  DefaultVideoFormat(Quiddity *quid, CustomPropertyHelper *prop_helper);
  DefaultVideoFormat() = delete;
  ~DefaultVideoFormat() = default;
  DefaultVideoFormat(const DefaultVideoFormat &) = delete;
  DefaultVideoFormat &operator=(const DefaultVideoFormat &) = delete;

  void make_format_property(const char *name, const char *display_text);
  bool disable_property();
  bool enable_property();
  std::string get_caps_str() const;
  
 private:
  Quiddity *quid_;
  std::string prop_name_{};
  // custom properties:
  CustomPropertyHelper *custom_props_;
  // video format
  GParamSpec *video_format_spec_{nullptr};
  GEnumValue video_format_[128]{};
  gint format_{0};
  std::vector<std::string> caps_{};
  std::vector<std::string> formats_{};
  static void set_format(const gint value, void *user_data);
  static gint get_format(void *user_data);
};

}  // namespace switcher
#endif
