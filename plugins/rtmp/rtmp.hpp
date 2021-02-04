/*
 * This file is part of switcher-rtmp.
 *
 * switcher-rtmp is free software; you can redistribute it and/or
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

#include "switcher/gst/pipeliner.hpp"
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/quiddity/startable.hpp"
#include "switcher/shmdata/connector.hpp"
#include "switcher/shmdata/follower.hpp"

namespace switcher {
namespace quiddities {
/**
 * RTMP class,
 * RTMP quiddity compatible with multiple streaming applications (Youtube, Twitch,...)
 */
using namespace quiddity;
class RTMP : public Quiddity, public Startable {
 public:
  RTMP(quiddity::Config&&);
  ~RTMP();
  RTMP(const RTMP&) = delete;
  RTMP& operator=(const RTMP&) = delete;

 private:
  bool start() final;
  bool stop() final;
  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect(const std::string& shmpath);
  bool on_shmdata_disconnect_all();
  bool can_sink_caps(const std::string& str_caps);

  bool create_gst_pipeline();

  std::string audio_shmpath_{};
  std::string video_shmpath_{};
  shmdata::Connector shmcntr_;
  std::unique_ptr<shmdata::Follower> follower_video_{nullptr};
  std::unique_ptr<shmdata::Follower> follower_audio_{nullptr};

  std::string stream_app_url_{};
  property::prop_id_t stream_app_url_id_;
  std::string stream_key_{};
  property::prop_id_t stream_key_id_;

  std::unique_ptr<gst::Pipeliner> gst_pipeline_{nullptr};
};
SWITCHER_DECLARE_PLUGIN(RTMP);
}  // namespace quiddities
}  // namespace switcher
