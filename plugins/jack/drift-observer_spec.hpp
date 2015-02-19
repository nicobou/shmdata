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

#include <iostream>

namespace switcher {

template<typename TimeType>
TimeType DriftObserver<TimeType>::set_current_time_info(
    const TimeType date,
    const TimeType duration){
  // collect data only at first call
  if (0 == current_buffer_duration_) {
    current_buffer_date_ = date;
    current_buffer_duration_ = duration;
    return duration;
  }
  // udating statistics for the previous duration
  const double measured_ratio = (double)(date - current_buffer_date_) / current_buffer_duration_;
  if (0.9 < measured_ratio && measured_ratio < 1.1)
    ratio_ = (1 - smoothing_factor_) * ratio_ + smoothing_factor_ * measured_ratio;
  else
    ratio_ = 1;
  current_buffer_date_ = date;
  current_buffer_duration_ = duration;
  // computing expected duration for this current data,
  // according to statistics previouslyt computed
  const double res = duration*ratio_ + remainder_;
  const double decimals = res - std::floor(res);
  // minimising the remainder
  if (decimals < 0.5){
    remainder_ = decimals;
    return static_cast<TimeType>(res);
  } else {
    remainder_ = decimals - 1;
    return static_cast<TimeType>(res + 1);
  }
}

template<typename TimeType>
void DriftObserver<TimeType>::set_smoothing_factor(const double &sf){
  smoothing_factor_ = sf;
}

}  // namespace switcher
