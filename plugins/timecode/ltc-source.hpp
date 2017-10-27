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

#ifndef SWITCHER_LTC_SOURCE_HPP
#define SWITCHER_LTC_SOURCE_HPP

#include <jack/jack.h>
#include <ltc.h>
#include <deque>
#include <fstream>
#include <switcher/startable-quiddity.hpp>
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"
#include "switcher/threaded-wrapper.hpp"
#include "switcher/unique-gst-element.hpp"

namespace switcher {
/**
 * LTCSource class,
 * Generated an LTC timecode in an audioshmdata or reads it from a prerecorded audio timecode file.
 * To be used for audio and video synchronization.
 */
class LTCSource : public Quiddity, public StartableQuiddity {
 public:
  LTCSource(QuiddityConfiguration&&);
  ~LTCSource();

 private:
  bool start() final;
  bool stop() final;

  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect();
  bool can_sink_caps(const std::string& caps);

  void tick_callback(jack_nframes_t nframes);
  void write_samples_to_shmdata(const unsigned int& nb_samples);
  void generate_ltc_frames(int nb_frames);

  std::deque<ltcsnd_sample_t> samples_{};       //!< Pool of samples to send to shmdata
  std::atomic<bool> generating_frames_{false};  //!< Flag to prevent infinite generation of samples.
  std::unique_ptr<ShmdataWriter> shmw_{};       //!< Shmdata writer.
  ShmdataConnector shmcntr_{nullptr};
  bool external_sync_source_{false};  //!< Is a shmdata connected to use a cadencing source.
  size_t format_size_{0};
  std::unique_ptr<ShmdataFollower> shm_follower_{
      nullptr};                          //!< Incoming sound stream optionally used for cadencing.
  jack_client_t* jack_client_{nullptr};  //!< Jack client for sample generation cadencing

  LTCEncoder* encoder_{nullptr};  //!< LTC encoder used for ltc generation.
  SMPTETimecode st_{};            //!< Starting timecode when generating ltc frames.
  std::unique_ptr<ThreadedWrapper<>> generation_th_{
      std::make_unique<ThreadedWrapper<>>()};  //!< Thread used to generate samples asynchronously
                                               //! to avoid blocking the shmdata writing.

  // Properties
  PContainer::prop_id_t time_reference_id_{0};
  Selection<> time_reference_{{"Absolute timecode", "Relative timecode from start time"}, 1};
  PContainer::prop_id_t fps_id_{0};
  Selection<double> fps_{{"30 FPS", "25 FPS", "24 FPS"}, {30, 25, 24}, 0};
  jack_nframes_t sample_rate_{0};
  PContainer::prop_id_t timeshift_fw_id_{0};
  unsigned int timeshift_fw_{0};
};
SWITCHER_DECLARE_PLUGIN(LTCSource)
}  // namespace switcher
#endif
