/*
 * This file is part of libswitcher.
 **
 ** libswitcher is free software; you can redistribute it and/or
 ** modify it under the terms of the GNU Lesser General Public
 ** License as published by the Free Software Foundation; either
 ** version 2 of the License, or (at your option) any later version.
 **
 ** This library is distributed in the hope that it will be useful,
 ** but WITHOUT ANY WARRANTY; without even the implied warranty of
 ** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 ** Lesser General Public License for more details.
 **
 ** You should have received a copy of the GNU Lesser General
 ** Public License along with this library; if not, write to the
 ** Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 ** Boston, MA 02111-1307, USA.
 **/

#ifndef __SWITCHER_STD2_H__
#define __SWITCHER_STD2_H__

#include <mutex>

namespace std2 {

class shared_mutex {
 public:
  shared_mutex() {}
  ~shared_mutex() {}

  void lock() { global_mutex_.lock(); }
  void unlock() { global_mutex_.unlock(); }
  void lock_shared() {
    readers_mutex_.lock();
    ++nb_readers_;
    if (nb_readers_ == 1) global_mutex_.lock();
    readers_mutex_.unlock();
  }
  void unlock_shared() {
    readers_mutex_.lock();
    --nb_readers_;
    if (nb_readers_ == 0) global_mutex_.unlock();
    readers_mutex_.unlock();
  }

 private:
  mutable int nb_readers_{0};
  mutable std::mutex readers_mutex_{};
  mutable std::mutex global_mutex_{};
};

template <typename T>
class shared_lock {
 public:
  shared_lock() = delete;
  shared_lock(T& mutex) : shared_mutex_(mutex) { shared_mutex_.lock_shared(); }
  ~shared_lock() { shared_mutex_.unlock_shared(); }

 private:
  T& shared_mutex_;
};

}  // namespace std2
#endif
