/*
 * This file is part of switcher.
 *
 * switcher-gst is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_DUMMY_PLUGIN_H__
#define __SWITCHER_DUMMY_PLUGIN_H__

#include <memory>
#include <string>

#include "switcher/quiddity/quiddity.hpp"
#include "switcher/gst/unique-gst-element.hpp"
#include "switcher/gst/pipeliner.hpp"

// This quiddity implements a simple gst pipeline

namespace switcher {
namespace quiddities {
using namespace quiddity;
class GstQuid : public Quiddity {
 public:
  GstQuid(quiddity::Config&&);
  ~GstQuid() = default;
  GstQuid(const GstQuid&) = delete;
  GstQuid& operator=(const GstQuid&) = delete;
  
  static std::string make_username();

 private:
  bool play(bool);
  bool start_pipeline();
  bool stop_pipeline();
  bool remake_elements();

  std::unique_ptr<gst::Pipeliner> pipeline_;
  gst::UGstElem src_{"videotestsrc"};
  gst::UGstElem sink_{"xvimagesink"};

  bool started_{false};
  property::prop_id_t started_id_;
};

SWITCHER_DECLARE_PLUGIN(GstQuid);

}  // namespace quiddities
}  // namespace switcher
#endif
