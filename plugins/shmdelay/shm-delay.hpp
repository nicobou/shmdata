/*
 * This file is part of switcher-shmdelay.
 *
 * switcher-shmdelay is free software; you can redistribute it and/or
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

#ifndef SWITCHER_SHMDELAY_HPP
#define SWITCHER_SHMDELAY_HPP

#include <ltc.h>
#include <deque>
#include "switcher/gst-utils.hpp"
#include "switcher/periodic-task.hpp"
#include "switcher/quiddity.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"

namespace switcher {
/**
 * ShmDelay class
 */
class ShmDelay : public Quiddity {
 public:
  ShmDelay(const std::string&);
  ~ShmDelay();

 private:
  struct ShmContent {
    ShmContent(double timestamp, void* content, size_t data_size);
    /** Get physical size of the shmdata
    * content (overhead not accounted for because not relevant in size compared to the data)
    */
    size_t get_size() const { return content_.size() * sizeof(unsigned char); }
    std::vector<unsigned char> content_{};
    size_t data_size_{0};  //!< Size of the shmdata.
    double timestamp_{0};  //!< Timestamp at the time of shmdata reception.
  };

  /**
   * ShmBuffer , contains a vector of ShmContent and is limited in physical size.
   * Can be searched for closest timestamped object.
   */
  struct ShmBuffer {
    ShmBuffer(size_t buffer_size) : max_size(buffer_size) {}
    void push(const ShmContent& content);     //!< Push a ShmContent object in the buffer
    ShmContent find(double timestamp) const;  //!< Find the closest shmdata to the provided
                                              //! timestamp (has ! to be closer than !< 10ms)

    std::deque<ShmContent> buffer_{};  //!< Buffer of shmdata frames.
    mutable std::mutex buffer_m_{};    //!< Mutex here to protect the buffer accesses.
    size_t total_size{0};  //!< Current total size of the buffer (not accounting for overhead)
    size_t max_size{0};    //!< Maximum size in bytes of the buffer.
  };

  bool init() final;

  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect();
  bool can_sink_caps(const std::string& caps);

  bool is_valid_{false};
  std::unique_ptr<ShmdataWriter> shmw_{};  //!< Shmdata writer.
  ShmdataConnector shmcntr_{nullptr};      //!< Shmdata connector for ltc and shmdata inputs
  std::unique_ptr<ShmdataFollower> shm_follower_{nullptr};  //!< Shmdata to be delayed
  std::unique_ptr<PeriodicTask<std::chrono::microseconds>> writing_task_{
      nullptr};  //!< Shmdata writing task (high frequency to be accurate on timestamp checking)
  ShmBuffer delay_content_{1000000000};  //!< Size limit for the buffer (~1GB)
  double last_timestamp_{0};

  // Properties
  unsigned int time_delay_{0};  //!< Delay in milliseconds.
};
SWITCHER_DECLARE_PLUGIN(ShmDelay)
}  // namespace switcher
#endif
