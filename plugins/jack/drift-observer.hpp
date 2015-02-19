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
template<typename TimeType>
class DriftObserver {
 public:
  DriftObserver() = default;
  ~DriftObserver() = default;
  void set_smoothing_factor(const double &sf);  // data race with set_current_time_info 
  // this is returning the duration this duration should have
  TimeType set_current_time_info(const TimeType date,
                                 const TimeType duration);
  double get_ratio() const {return ratio_;}
  double get_smoothing_factor() const {return smoothing_factor_;}
 private:
  TimeType current_buffer_date_{0};
  TimeType current_buffer_duration_{0};
  double remainder_{0};
  double ratio_{1};
  double smoothing_factor_{0.0001};
};

}  // namespace switcher
#include "./drift-observer_spec.hpp"
#endif
