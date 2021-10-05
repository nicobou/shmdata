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

#ifndef __SWITCHER_SHMDATA_DYNAMIC_WRITER_H__
#define __SWITCHER_SHMDATA_DYNAMIC_WRITER_H__

#include "switcher/quiddity/quiddity.hpp"

namespace switcher {
namespace quiddities {
using namespace quiddity;

/**
 * ShmdataDynamicWriter class.
 *
 * Example class that illustrates how to implement Shmdata
 * writer when the number and\or type of Shmdata is not know
 * at initialisation.
 *
 */

class ShmdataDynWriter : public Quiddity {
 public:
  /**
   * ShmdataDynWriter constructor. It implements the expected
   * behaviour for being a Quiddity.
   *
   * \param conf Configuration given by Switcher
   */
  ShmdataDynWriter(quiddity::Config&& conf);

 private:
  /**
   * Connection specification for the ShmdataDynWriter
   * in string format, to be parsed by the Quidditity constructor.
   */
  static const std::string kConnectionSpec;

  /**
   * Quiddity property used for enabling/disabling video shmdata
   */
  property::prop_id_t video_prop_id_;
  bool video_{false};

  /**
   * id for videoA, a dynamic shmdata writer
   */
  claw::swid_t videoA_swid_{Ids::kInvalid};

  /**
   * id for videoB, a dynamic shmdata writer
   */
  claw::swid_t videoB_swid_{Ids::kInvalid};
};

SWITCHER_DECLARE_PLUGIN(ShmdataDynWriter);

}  // namespace quiddities
}  // namespace switcher
#endif
