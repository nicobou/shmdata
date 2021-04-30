
/*
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#undef NDEBUG  // get assert in release mode

#include <cassert>
#include <string>

#include "switcher/quiddity/claw/connection-spec.hpp"

using namespace switcher::quiddity::claw;
using namespace std::string_literals;  // enables s-suffix for std::string literals

int main() {
  {  // testing a valid spec
    auto spec = R"(
    {
    "follower":
      [
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"]
        },
        {
          "name": "mic%d",
          "description": "Audio inputs to be analysed",
          "caps": ["audio/x-raw", "audio/mpeg"]
        },
        {
          "name": "custom",
          "description": "Custom shmdata for my statistical analysis",
          "caps": []
        }
      ],
    "writer":
      [
        {
          "name": "texture",
          "description": "Produced rendering",
          "caps": ["video/x-raw"]
        }
      ]
    }
    )"s;
    assert(ConnectionSpec(spec));
  }  // end testing a valid spec

  {  // testing spec with json error (missing " after 'follower')
    auto spec = R"(
    {
    "follower:
      [
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "JSON error: " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing spec with typo no follower nor writer (typo in follower)
    auto spec = R"(
    {
    "foolower":
      [
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "Follower typo: " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing spec with no array under follower
    auto spec = R"(
    {
    "follower":
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"]
        }
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "No array in follower: " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing unexpected number of keys
    auto spec = R"(
    {
    "follower":
      [
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"],
          "comment": "My unexpected comment"
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "unexpected number of keys : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing name is a string
    auto spec = R"(
    {
    "follower":
      [
        {
          "name": 3,
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "name is not a string : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing description is a string
    auto spec = R"(
    {
    "follower":
      [
        {
          "name": "texture",
          "description": 3.14,
          "caps": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "description is not a string : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing caps entry must be an array
    auto spec = R"(
    {
    "follower":
      [
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": "video/x-raw"
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "caps entry must be an array : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing caps entry contain strings
    auto spec = R"(
    {
    "follower":
      [
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw", 3]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "caps entry must contain strings : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing caps entry can be parsed as shmdata type (here missing value for width)
    auto spec = R"(
    {
    "follower":
      [
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw, width"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "caps entry must be shmdata type : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing spec with duplicates in follower names
    auto spec = R"(
    {
    "follower":
      [
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"]
        },
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "Duplicates in follower names: " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing spec with duplicates in writer names
    auto spec = R"(
    {
    "writer":
      [
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"]
        },
        {
          "name": "texture",
          "description": "Texture to apply during the processing",
          "caps": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "Duplicates in writer names: " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  return 0;
}
