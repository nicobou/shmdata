/*
 * This file is part of switcher-timecode.
 *
 * switcher-timecode is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_LTC_TO_JACK_HPP__
#define __SWITCHER_LTC_TO_JACK_HPP__

#include <jack/jack.h>
#include <ltc.h>
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"

namespace switcher {
/**
 * LTCToJack class,
 * Decodes an incoming audio stream with encoded LTC and controls Jack Transport from it.
 *
 */
class LTCToJack : public Quiddity {
 public:
  LTCToJack(QuiddityConfiguration&&);
  ~LTCToJack();
  LTCToJack(const LTCToJack&) = delete;
  LTCToJack& operator=(const LTCToJack&) = delete;

 private:
  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect();
  void on_data(void* data, size_t data_size);
  bool can_sink_caps(std::string str_caps);

  // Shmdata
  ShmdataConnector shmcntr_;  //!< Shmdata connector to connect into the quiddity.
  std::unique_ptr<ShmdataFollower> shm_follower_{nullptr};  //!< Incoming LTC stream

  jack_client_t* jack_client_{nullptr};  //!< Jack client connected to transport
  LTCDecoder* decoder_{nullptr};         //!< LTC frame decoder

  // Time difference detection between jack transport and incoming LTC clock
  double jack_time_ref_{0.};     //!< Jack transport time at the beginning of the LTC decoding
  double diff_clocks_{0.};       //!< Difference between the absolute clocks of jack and LTC
  double drift_threshold_{.05};  //!< Minimum time difference to trigger a repositioning of jack
  PContainer::prop_id_t drift_threshold_id_{0};  //!< Threshold property handle
  std::mutex threshold_mutex_{};                 //!< Protection of drift threshold member

  // Needed for LTC fps detection
  int ltc_fps_{30};                     //!< Detected FPS of the current LTC decoding session
  int tmp_ltc_fps_{0};                  //!< Temporary variable used during FPS detection
  bool ltc_fps_detected_{false};        //!< Flag used to stop FPS detection process
  bool ltc_first_zero_detected{false};  //!< Used during FPS detection to detect start of cycle

  // Play/pause/
  bool paused_{false};        //!< Jack transport ad LTC are currently paused
  bool force_locate_{false};  //!< Force repositioning of jack transport (clocks drift/pause/seek)
};

SWITCHER_DECLARE_PLUGIN(LTCToJack);

}  // namespace switcher

#endif
