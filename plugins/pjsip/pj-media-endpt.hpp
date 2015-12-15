/*
 * This file is part of switcher-pjsip.
 *
 * switcher-pjsip is free software: you can redistribute it and/or modify
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

#ifndef PLUGINS_PJSIP_MEDIA_ENDPT_H_
#define PLUGINS_PJSIP_MEDIA_ENDPT_H_

#include <pjsua-lib/pjsua.h>

// pjmedia_endpt RAII class

namespace switcher {
class PJCodec;

class PJMediaEndpt {
  friend PJCodec;
 public:
  PJMediaEndpt();
  ~PJMediaEndpt();
  PJMediaEndpt(const PJMediaEndpt &) = delete;
  PJMediaEndpt &operator=(const PJMediaEndpt &) = delete;

  pjmedia_endpt *get() {return med_endpt_;};
  
 private:
  static pjmedia_endpt *med_endpt_;
};

}  // namespace switcher
#endif
