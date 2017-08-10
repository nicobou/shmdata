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

#ifndef SWITCHER_LTC_DIFF_HPP
#define SWITCHER_LTC_DIFF_HPP

#include <ltc.h>
#include "switcher/periodic-task.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"

namespace switcher {
/**
 * LTCDiff class,
 */
class LTCDiff : public Quiddity {
 public:
  struct LTCReader {
    LTCReader(LTCDiff* quid, const std::string& shmpath, size_t index);
    ~LTCReader();
    void on_data(void* data, size_t data_size);

    double timecode_{0};                                      //!< Last detected timecode
    std::unique_ptr<ShmdataFollower> shm_follower_{nullptr};  //!< LTC shmdata follower
    std::mutex timecode_m_{};
    LTCDecoder* decoder_{nullptr};  //!< LTC decoder used to detect real time value

    LTCDiff* parent_{nullptr};
    size_t index_{0};
    int ltc_fps_{30};                     //!< Detected FPS of the current LTC decoding session
    int tmp_ltc_fps_{0};                  //!< Temporary variable used during FPS detection
    bool ltc_fps_detected_{false};        //!< Flag used to stop FPS detection process
    bool ltc_first_zero_detected{false};  //!< Used during FPS detection to detect start of cycle
  };

  LTCDiff(QuiddityConfiguration&& conf);
  ~LTCDiff() = default;

 private:

  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect(const std::string& shmpath);
  bool can_sink_caps(const std::string& caps);

  bool is_valid_{false};
  bool do_compute_{false};
  ShmdataConnector shmcntr_{nullptr};
  std::unique_ptr<ShmdataFollower> shm_follower_{
      nullptr};  //!< Unique, used to cadence the time difference computation
  std::unique_ptr<ShmdataWriter> shmw_{nullptr};

  std::map<std::string, std::unique_ptr<LTCReader>>
      ltc_readers_{};  //!< Incoming sound stream optionally used for cadencing.

  std::mutex timecode_m_{};

  double time_difference_{0};  //!< Timecode difference in milliseconds

  // Display of timecodes in read-only properties
  size_t next_index_{0};  //!< Index of the next LTCReader
  std::vector<std::string> display_timecodes_{
      2};  //!< Timecode in string form for display in the inspector
  PContainer::prop_id_t
      display_timecode1_id_{};  //!< Timecode display property of one of the ltc sources
  PContainer::prop_id_t
      display_timecode2_id_{};  //!< Timecode display property of the other ltc source
  std::unique_ptr<PeriodicTask<>> notify_task_{
      nullptr};  //!< Task used to notify the display timecode properties
};
SWITCHER_DECLARE_PLUGIN(LTCDiff)
}  // namespace switcher

#endif
