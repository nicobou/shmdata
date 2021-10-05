/*
 * This file is part of switcher-plugin-example.
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

#ifndef __SWITCHER_SHMDATA_DYNAMIC_READER_H__
#define __SWITCHER_SHMDATA_DYNAMIC_READER_H__

#include "switcher/quiddity/quiddity.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;

/**
 * ShmdataDynamicReader class.
 *
 * Example class that illustrates how to implement shmdata
 * reader when the number and\or type of Shmdata is not know
 * at initialisation.
 *
 */

class ShmdataDynReader : public Quiddity {
 public:
  /**
   * ShmdataDynReader constructor. It implements the expected
   * behaviour for being a Quiddity.
   *
   * \param conf Configuration given by Switcher
   */
  ShmdataDynReader(quiddity::Config&& conf);

 private:
  /**
   * Connection specification for the ShmdataDynReader
   * in string format, to be parsed by the Quidditity constructor.
   */
  static const std::string kConnectionSpec;

  /**
   * Implementation of connection callback required by the Claw Class
   */
  bool on_connect(const std::string& shmpath, claw::sfid_t sfid);

  /**
   * Implementation of disconnection callback required by the Claw Class
   */
  bool on_disconnect(claw::sfid_t /*sfid*/);
};

SWITCHER_DECLARE_PLUGIN(ShmdataDynReader);

}  // namespace quiddities
}  // namespace switcher
#endif
