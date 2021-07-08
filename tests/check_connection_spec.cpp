
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
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"]
        },
        {
          "label": "mic%d",
          "description": "Audio inputs to be analysed",
          "can_do": ["audio/x-raw", "audio/mpeg"]
        },
        {
          "label": "custom",
          "description": "Custom shmdata for my statistical analysis",
          "can_do": []
        }
      ],
    "writer":
      [
        {
          "label": "texture",
          "description": "Produced rendering",
          "can_do": ["video/x-raw"]
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
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"]
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
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"]
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
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"]
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
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"],
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

  {  // testing label is a string
    auto spec = R"(
    {
    "follower":
      [
        {
          "label": 3,
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "label is not a string : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing description is a string
    auto spec = R"(
    {
    "follower":
      [
        {
          "label": "texture",
          "description": 3.14,
          "can_do": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "description is not a string : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing can_do entry must be an array
    auto spec = R"(
    {
    "follower":
      [
        {
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": "video/x-raw"
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "can_do entry must be an array : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing can_do entry contain strings
    auto spec = R"(
    {
    "follower":
      [
        {
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw", 3]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "can_do entry must contain strings : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing can_do entry can be parsed as shmdata type (here missing value for width)
    auto spec = R"(
    {
    "follower":
      [
        {
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw, width"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "can_do entry must be shmdata type : " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing spec with duplicates in follower labels
    auto spec = R"(
    {
    "follower":
      [
        {
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"]
        },
        {
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "Duplicates in follower labels: " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  {  // testing spec with duplicates in writer labels
    auto spec = R"(
    {
    "writer":
      [
        {
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"]
        },
        {
          "label": "texture",
          "description": "Texture to apply during the processing",
          "can_do": ["video/x-raw"]
        }
      ]
    }
    )";
    auto cspec = ConnectionSpec(spec);
    std::cout << "Duplicates in writer labels: " << cspec.msg() << '\n';
    assert(!cspec.msg().empty());
    assert(!cspec);
  }

  return 0;
}
