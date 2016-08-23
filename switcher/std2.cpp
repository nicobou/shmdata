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

#include "./std2.hpp"

namespace std2 {

shared_mutex::shared_mutex() {}

shared_mutex::~shared_mutex() {}

void shared_mutex::lock() { stmtx_.lock(); }

void shared_mutex::unlock() { stmtx_.unlock(); }

void shared_mutex::lock_shared() { stmtx_.lock_shared(); }

void shared_mutex::unlock_shared() { stmtx_.unlock_shared(); }

shared_lock::shared_lock(shared_mutex& mutex) : shared_mutex_(mutex) {
  shared_mutex_.lock_shared();
}

shared_lock::~shared_lock() { shared_mutex_.unlock_shared(); }

}  // namespace std2
