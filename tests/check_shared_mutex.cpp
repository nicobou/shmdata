/*
 * This file is part of switcher.
 *
 * switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cassert>
#include <chrono>
#include <future>
#include <iostream>
#include "switcher/std2.hpp"

static int val = 0;
static std2::shared_mutex shared_mtx;
static std::mutex mtx;
static const int number_of_operation_per_thread = 1000000;

void writer() {
  for (int i = 0; i < number_of_operation_per_thread; ++i) {
    std::lock_guard<std2::shared_mutex> lock(shared_mtx);
    val = i;
  }
}

void reader() {
  for (int i = 0; i < number_of_operation_per_thread; ++i) {
    std2::shared_lock lock(shared_mtx);
    assert(val >= 0 && val <= number_of_operation_per_thread);
  }
}

void exclusive_writer() {
  for (int i = 0; i < number_of_operation_per_thread; ++i) {
    std::lock_guard<std::mutex> lock(mtx);
    val = i;
  }
}

void exclusive_reader() {
  for (int i = 0; i < number_of_operation_per_thread; ++i) {
    std::lock_guard<std::mutex> lock(mtx);
    assert(val >= 0 && val <= number_of_operation_per_thread);
  }
}

int main() {
  using namespace std::chrono;
  {  // shared lock mtx
    auto start = steady_clock::now();
    auto reader1_handle = std::async(std::launch::async, reader);
    auto reader2_handle = std::async(std::launch::async, reader);
    auto reader3_handle = std::async(std::launch::async, reader);
    auto writer_handle = std::async(std::launch::async, writer);
    reader1_handle.get();
    reader2_handle.get();
    reader3_handle.get();
    writer_handle.get();
    auto duration = duration_cast<milliseconds>(steady_clock::now() - start).count();
    std::cout << "with shared mutex: " << duration << "ms" << '\n';
  }
  {  // exclusive mutex
    auto start = steady_clock::now();
    auto reader1_handle = std::async(std::launch::async, exclusive_reader);
    auto reader2_handle = std::async(std::launch::async, exclusive_reader);
    auto reader3_handle = std::async(std::launch::async, exclusive_reader);
    auto writer_handle = std::async(std::launch::async, exclusive_writer);
    reader1_handle.get();
    reader2_handle.get();
    reader3_handle.get();
    writer_handle.get();
    auto duration = duration_cast<milliseconds>(steady_clock::now() - start).count();
    std::cout << "with mutex: " << duration << "ms" << '\n';
  }
  return 0;
}
