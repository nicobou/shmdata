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

#ifndef __SWITCHER_LOGGER_H__
#define __SWITCHER_LOGGER_H__

#include <unordered_map>
#include <mutex>
#include "./quiddity.hpp"

namespace switcher {
class Logger: public Quiddity {
 public:
  SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(Logger);
  Logger(const std::string &);
  ~Logger();
  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;
  bool init() final;

 private:
  static bool installed_;
  bool i_am_the_one_{false};
  std::string last_line_{""};
  PContainer::prop_id_t last_line_id_;
  bool mute_{false};
  bool debug_{true};
  bool verbose_{true};
  std::unordered_map<std::string, guint> handler_ids_{};
  std::mutex last_line_mutex_{};

  void replace_last_line(std::string next_line);
  gboolean install_log_handler(const gchar *log_domain);
  gboolean remove_log_handler(const gchar *log_domain);

  static gboolean install_log_handler_wrapped(gpointer log_domain,
                                              gpointer user_data);
  static gboolean remove_log_handler_wrapped(gpointer log_domain,
                                             gpointer user_data);
  static void log_handler(const gchar *log_domain,
                          GLogLevelFlags log_level, const gchar *message,
                          gpointer user_data);
  static void quiet_log_handler(const gchar *log_domain,
                                GLogLevelFlags log_level,
                                const gchar *message, gpointer user_data);
};

}  // namespace switcher
#endif
