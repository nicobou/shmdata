/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#undef NDEBUG  // get assert in release mode

#include <cassert>
#include <iostream>
#include "shmdata/type.hpp"

// the shmdata::Type class provides 

template <typename T>
bool check_value(const shmdata::Type& type, const std::string& key, const T& value) {
  return std::any_cast<T>(type.get(key)) == value;
}

bool check_value(const shmdata::Type& type, const std::string& key, const char* value) {
  return std::any_cast<std::string>(type.get(key)) == value;
}

int main () {
  using namespace shmdata;

  { // checking type parsing robustness with a GStreamer simple caps:
    auto test_type_str = std::string(
        "video/x-raw, format=(string)I420, width=(int)320, height=(int)240, "
        "framerate=(fraction)30/1, multiview-mode=(string)mono, pixel-aspect-ratio=(fraction)1/1, "
        "interlace-mode=(string)progressive  , label=(string)\"\\(unexpected\\) \\= chars\\,\"");
    auto type = Type(test_type_str);
    assert(type.name() == "video/x-raw");
    assert(type.get_properties().size() == 8);
    std::cout << std::any_cast<std::string>(type.get("label")) << '\n';
    assert(check_value<std::string>(type, "label", "(unexpected) = chars,"));
    assert(check_value<std::string>(type, "pixel-aspect-ratio", "1/1"));
    assert(check_value<int>(type, "width", 320));
  }

  {  // check building a Type and consistency when serializing/deserializing it
    auto type = Type("video/x-raw");
    type.set_prop("format", "I420");
    type.set_prop("framerate", "fraction", "30/1");
    type.set_prop("height", 4096);
    type.set_prop("interlace-mode", "progressive");
    type.set_prop("label", "unexpected = comma,");
    type.set_prop("multiview-mode", "mono");
    type.set_prop("pixel-aspect-ratio", "fraction", "1/1");
    type.set_prop("width", 4096);
    // building an other Type from 'type' serialization and check it has the right name and
    auto type2 = Type(type.str());
    assert(type2.name() == "video/x-raw");
    assert(check_value(type2, "format", "I420"));
    assert(check_value(type2, "framerate", "30/1"));
    assert(check_value(type2, "height", 4096));
    assert(check_value(type2, "interlace-mode", "progressive"));
    assert(check_value(type2, "label", "unexpected = comma,"));
    assert(check_value(type2, "multiview-mode", "mono"));
    assert(check_value(type2, "pixel-aspect-ratio", "1/1"));
    assert(check_value(type2, "width", 4096));
    // check serializations are the same
    assert(type.str() == type2.str());
  }
  return 0;
}

