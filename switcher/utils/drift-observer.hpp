/*
 * This file is part of switcher-jack.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_DRIFT_OBSERVER_H__
#define __SWITCHER_DRIFT_OBSERVER_H__

namespace switcher {
namespace utils {

/**
 * A drift observer class. Drift is computed from user-poevided calls with timing. The class
 * internaly tracks these timing and keeps a ratio that can be requested. A smoothing factor is
 * available in order to avoid the ration to follow possible hight variation in the user provided
 * timings, possibly due to buffer size.
 *
 * This class has been designed to be used with AudioRingBuffer and AudioResampler.
 *
 * The is a data race among calls to set_current_time_info and set_smoothing_factor. Accordingly, it
 * is suggested to call these methods from the same thread.
 *
 * \tparam TimeType Type of the clock used. It can be used with sample numbers.
 */
template <typename TimeType>
class DriftObserver {
 public:
  DriftObserver() = default;
  ~DriftObserver() = default;
  /**
   * Set the smoothing_factor.
   * \param sf Smoothing factor.
   */
  void set_smoothing_factor(const double& sf);
  /**
   * Set new timing info. This method must be called periodically in order to efficiently observe
   * the drift. \param date Clock value for this instant. \param duration Expected duration until
   * the next call to set_current_time_info \return Corrected duration until the next call to
   * set_current_time_info
   */
  TimeType set_current_time_info(const TimeType date, const TimeType duration);
  /**
   * Get the current ratio.
   * \return Ratio
   **/
  double get_ratio() const { return ratio_; }
  /**
   * Get the current smoothing factor.
   * \param Smoothing factor.
   **/
  double get_smoothing_factor() const { return smoothing_factor_; }

 private:
  TimeType current_buffer_date_{0};
  TimeType current_buffer_duration_{0};
  double remainder_{0};
  double ratio_{1};
  double smoothing_factor_{0.0001};
};

}  // namespace utils
}  // namespace switcher
#include "./drift-observer_spec.hpp"
#endif
