/*
 * This file is part of switcher-samplerate.
 *
 * switcher-sample is free software: you can redistribute it and/or modify
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

#ifndef __SWITCHER_RESAMPLE_H__
#define __SWITCHER_RESAMPLE_H__

#include <samplerate.h>
#include "switcher/quiddity/quiddity.hpp"
#include "switcher/shmdata/caps/audio-caps.hpp"
#include "switcher/shmdata/shmdata-connector.hpp"
#include "switcher/shmdata/shmdata-follower.hpp"
#include "switcher/shmdata/shmdata-writer.hpp"

namespace switcher {
class Resample : public Quiddity {
 public:
  Resample(quid::Config&&);

 private:
  bool connect(const std::string& path);
  bool disconnect();
  bool can_sink_caps(const std::string&);

  // resampling
  size_t resampled_size_{0};
  std::vector<float> resampled_{};  // buffer of resampled data
  size_t in_converted_size_{0};
  std::vector<float> in_converted_{};   // depth converted input buffer
  SRC_STATE* resampler_config_{nullptr};
  std::unique_ptr<SRC_DATA> resampler_data_{};
  // shmdata
  std::unique_ptr<AudioCaps> incaps_{};
  std::unique_ptr<ShmdataWriter> shmw_{};
  std::unique_ptr<ShmdataFollower> shmr_{nullptr};
  ShmdataConnector cntr_;
  // properties
  unsigned int samplerate_{44100};
  PContainer::prop_id_t samplerate_id_;
  Selection<int> algo_{{"Best Quality (Bandlimited)",
                        "Medium Quality (Bandlimited)",
                        "Fastest (Bandlimited)",
                        "Zero Order Hold (Blindlingly Fast)",
                        "Linear (Blindlingly Fast)"},
                       {SRC_SINC_BEST_QUALITY,
                        SRC_SINC_MEDIUM_QUALITY,
                        SRC_SINC_FASTEST,
                        SRC_ZERO_ORDER_HOLD,
                        SRC_LINEAR},
                       2};
  PContainer::prop_id_t algo_id_;
};

SWITCHER_DECLARE_PLUGIN(Resample);

}  // namespace switcher
#endif
