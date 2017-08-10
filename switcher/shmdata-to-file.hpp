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

#ifndef __SWITCHER_SHMDATA_TO_FILE_H__
#define __SWITCHER_SHMDATA_TO_FILE_H__

#include <gst/gst.h>
#include <string>
#include <unordered_map>
#include "./custom-property-helper.hpp"
#include "./gst-pipeliner.hpp"
#include "./quiddity.hpp"
#include "./segment.hpp"
#include "./startable-quiddity.hpp"

namespace switcher {
class ShmdataToFile : public GstPipeliner, public StartableQuiddity {
 public:
  ShmdataToFile(QuiddityConfiguration&&);
  ~ShmdataToFile();
  ShmdataToFile(const ShmdataToFile&) = delete;
  ShmdataToFile& operator=(const ShmdataToFile&) = delete;

  bool start();
  bool stop();

  // local streams
  bool remove_shmdata(std::string shmdata_socket_path);

 private:
  // custom properties:
  CustomPropertyHelper::ptr custom_prop_;

  std::string output_prefix_{"shmfile_"};

  GParamSpec* output_prefix_param_{nullptr};

  bool init_gpipe() final;

  bool connect(std::string shmdata_socket_path);
  bool disconnect_all();
  bool can_sink_caps(std::string /*unused*/);

  bool make_recorders();
  bool clean_recorders();
  std::unordered_map<std::string, std::string> file_names_{};

  static const gchar* get_output_prefix(void* user_data);
  static void set_output_prefix(const gchar* prefix, void* user_data);
};

}  // namespace switcher

#endif  // ifndef
