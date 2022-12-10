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

#include "./logger.hpp"

namespace switcher {
namespace logger {

Logger::Logger(const std::string& logger_name, switcher::Configuration* conf, bool debug)
    : log_(std::make_unique<Log>(logger_name, conf, debug)), logger_(log_->get_logger()) {}

Logger::Logger(const Logger& obj) : logger_(obj.logger_) {}

}  // namespace logger
}  // namespace switcher
