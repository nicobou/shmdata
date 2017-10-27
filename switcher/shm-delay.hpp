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

#include <deque>
#include "./gst-utils.hpp"
#include "./periodic-task.hpp"
#include "./quiddity.hpp"
#include "./shmdata-connector.hpp"
#include "./shmdata-follower.hpp"
#include "./shmdata-writer.hpp"

namespace switcher {
/**
 * ShmDelay class
 */
class ShmDelay : public Quiddity {
 public:
  ShmDelay(QuiddityConfiguration&& conf);
  ~ShmDelay() = default;

 private:
  struct ShmContent {
    ShmContent(double timestamp, void* content, size_t data_size);
    ShmContent() = default;
    ~ShmContent() = default;
    /** Get physical size of the shmdata
    * content (overhead not accounted for because not relevant in size compared to the data)
    */
    size_t get_size() const { return content_.size(); }
    std::vector<u_int8_t> content_{};  //!< Vector containing received shm data frames
    size_t data_size_{0};              //!< Size of the shmdata.
    double timestamp_{0};              //!< Timestamp at the time of shmdata reception.
  };

  /**
   * ShmBuffer , contains a vector of ShmContent and is limited in physical size.
   * Can be searched for closest timestamped object.
   */
  struct ShmBuffer {
    ShmBuffer(size_t buffer_size) : max_size_(buffer_size * (1 << 20)) {}
    ~ShmBuffer() = default;
    void push(const ShmContent& content);  //!< Push a ShmContent object in the buffer
    void set_buffer_size(const size_t& size);
    ShmContent find_closest(
        double timestamp) const;  //!< Find the closest shmdata to the provided timestamp

   private:
    mutable std::mutex buffer_m_{};    //!< Mutex here to protect the buffer accesses.
    std::deque<ShmContent> buffer_{};  //!< Buffer of shmdata frames.
    size_t total_size_{0};  //!< Current total size of the buffer (not accounting for overhead)
    size_t max_size_{0};    //!< Maximum size in bytes of the buffer.
  };


  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect(const std::string& shmpath);

  ShmBuffer delay_content_{1 << 10};   //!< Size limit for the buffer (~1GB)
  ShmdataConnector shmcntr_{nullptr};  //!< Shmdata connector for ltc and shmdata inputs
  std::unique_ptr<ShmdataFollower> shm_follower_{nullptr};   //!< Shmdata to be delayed
  std::unique_ptr<ShmdataFollower> diff_follower_{nullptr};  //!< Timecode delay
  std::unique_ptr<ShmdataWriter> shmw_{};                    //!< Shmdata writer.
  std::unique_ptr<PeriodicTask<std::chrono::microseconds>> writing_task_{
      nullptr};  //!< Shmdata writing task (high frequency to be accurate on timestamp checking)
  double last_timestamp_{0};  //!< Timestamp of the last written shmdata.

  // Properties
  PContainer::prop_id_t time_delay_id_{0};
  double time_delay_{0};  //!< Delay in milliseconds.
  unsigned int buffer_size_{1 << 10};
};

}  // namespace switcher
#endif
