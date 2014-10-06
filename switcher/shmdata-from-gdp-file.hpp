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

#ifndef __SWITCHER_SHMDATA_FROM_GDP_FILE_H__
#define __SWITCHER_SHMDATA_FROM_GDP_FILE_H__

#include <gst/gst.h>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "./custom-property-helper.hpp"
#include "./gst-pipeliner.hpp"
#include "./quiddity-manager.hpp"
#include "./startable-quiddity.hpp"

namespace switcher {
class ShmdataFromGDPFile:public GstPipeliner, public StartableQuiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(ShmdataFromGDPFile);
  ShmdataFromGDPFile();
  ~ShmdataFromGDPFile();
  ShmdataFromGDPFile(const ShmdataFromGDPFile &) = delete;
  ShmdataFromGDPFile &operator=(const ShmdataFromGDPFile &) = delete;

  bool start();
  bool stop();

  static void rewind(gpointer user_data);

 private:
  // custom properties:
  CustomPropertyHelper::ptr custom_prop_;

  std::string input_prefix_ {"shmfile_"};
  GParamSpec *input_prefix_param_ {nullptr};

  std::map<std::string, std::string> shmdata_names_ {};
  unsigned int shm_counter_ {0};

  bool init_gpipe() final;

  bool make_players();
  bool clean_players();

  static const gchar *get_input_prefix(void *user_data);
  static void set_input_prefix(const gchar *prefix, void *user_data);

  static std::map<std::string, std::string> getFilenames(std::string prefix);
};
}  // namespace switcher

#endif                          // ifndef
