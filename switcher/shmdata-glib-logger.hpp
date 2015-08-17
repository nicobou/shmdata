/*
 * This file is part of libswitcher.
 *
 * libswitcher is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_SHMDATA_GLIB_LOGGER_H__
#define __SWITCHER_SHMDATA_GLIB_LOGGER_H__

#include <glib.h>
#include "shmdata/abstract-logger.hpp"

namespace switcher{

class ShmdataGlibLogger: public shmdata::AbstractLogger {
 private:
  void on_error(std::string &&str) final {
    g_error("%s", str.c_str());
  }
  void on_critical(std::string &&str) final {
    g_critical("%s", str.c_str());
  }
  void on_warning(std::string &&str) final {
    g_warning("%s", str.c_str());
  }
  void on_message(std::string &&str) final {
    g_message("%s", str.c_str());
  }
  void on_info(std::string &&str) final {
    g_info("%s", str.c_str());
  }
  void on_debug(std::string &&str) final {
    g_debug("%s", str.c_str());
  }
};

}  // namespace switcher
#endif
