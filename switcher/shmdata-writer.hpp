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

#ifndef __SWITCHER_SHMDATA_WRITER_H__
#define __SWITCHER_SHMDATA_WRITER_H__

#include <mutex>
#include "shmdata/writer.hpp"
#include "switcher/make-consultable.hpp"
#include "switcher/periodic-task.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/safe-bool-idiom.hpp"
#include "switcher/shmdata-stat.hpp"
#include "switcher/shmdata-switcher-logger.hpp"

namespace switcher {
class ShmdataWriter : public SafeBoolIdiom {
 public:
  ShmdataWriter(Quiddity* quid,
                const std::string& path,
                size_t memsize,
                const std::string& data_descr);
  ShmdataWriter() = delete;
  ~ShmdataWriter();
  ShmdataWriter(const ShmdataWriter&) = delete;
  ShmdataWriter& operator=(const ShmdataWriter&) = delete;
  ShmdataWriter& operator=(ShmdataWriter&&) = default;

  Make_delegate(ShmdataWriter, shmdata::Writer, &shm_, writer);
  // FIXME use consultable Global Wrapping
  // this is used in order to monitor traffic in the shmdata,
  // i.e. you need to update this at each write with the size writen,
  // regardless of the shmdata::Writer method you are using
  void bytes_written(size_t size);
  std::string get_path() const { return shmpath_; }

 private:
  Quiddity* quid_;
  std::string shmpath_;
  std::string data_type_;
  ShmdataSwitcherLogger shmlog_;
  shmdata::Writer shm_;
  std::unique_ptr<PeriodicTask<>> task_;
  ShmdataStat shm_stats_{};
  std::mutex bytes_mutex_{};

  bool safe_bool_idiom() const final { return static_cast<bool>(shm_); };
  void update_quid_stats();
};

}  // namespace switcher
#endif
