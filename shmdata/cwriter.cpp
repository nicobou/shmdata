/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#include "./cwriter.h"
#include "./sysv-shm.hpp"
#include "./writer.hpp"

namespace shmdata {
struct CWriter {
  CWriter(const char* path,
          size_t memsize,
          const char* type_descr,
          ShmdataLogger log,
          void (*on_client_connected)(void* user_data, int id),
          void (*on_client_disconnected)(void* user_data, int id),
          void* user_data,
          mode_t unix_permission)
      : on_client_connected_(on_client_connected),
        on_client_disconnected_(on_client_disconnected),
        user_data_(user_data),
        writer_(path,
                memsize,
                type_descr,
                static_cast<AbstractLogger*>(log),
                [&](int id) {
                  if (nullptr != on_client_connected_) on_client_connected_(user_data_, id);
                },
                [&](int id) {
                  if (nullptr != on_client_disconnected_) on_client_disconnected_(user_data_, id);
                },
                unix_permission) {}
  void (*on_client_connected_)(void*, int);
  void (*on_client_disconnected_)(void*, int);
  void* user_data_;
  Writer writer_;
};

}  // namespace shmdata

using namespace shmdata;
ShmdataWriter shmdata_make_writer(const char* path,
                                  size_t memsize,
                                  const char* type_descr,
                                  void (*on_client_connected)(void* user_data, int id),
                                  void (*on_client_disconnected)(void* user_data, int id),
                                  void* user_data,
                                  ShmdataLogger log,
                                  mode_t unix_permission) {
  CWriter* res = new CWriter(path,
                             memsize,
                             type_descr,
                             static_cast<AbstractLogger*>(log),
                             on_client_connected,
                             on_client_disconnected,
                             user_data,
                             unix_permission);
  if (res->writer_) return static_cast<void*>(res);
  delete res;
  return nullptr;
}

void shmdata_delete_writer(ShmdataWriter writer) { delete static_cast<CWriter*>(writer); }

int shmdata_copy_to_shm(ShmdataWriter writer, const void* data, size_t size) {
  return static_cast<CWriter*>(writer)->writer_.copy_to_shm(data, size);
}

ShmdataWriterAccess shmdata_get_one_write_access(ShmdataWriter writer) {
  return static_cast<void*>(static_cast<CWriter*>(writer)->writer_.get_one_write_access_ptr());
}

ShmdataWriterAccess shmdata_get_one_write_access_resize(ShmdataWriter writer, size_t new_size) {
  return static_cast<void*>(
      static_cast<CWriter*>(writer)->writer_.get_one_write_access_ptr_resize(new_size));
}

size_t shmdata_shm_resize(ShmdataWriterAccess access, size_t new_size) {
  return static_cast<OneWriteAccess*>(access)->shm_resize(new_size);
}

void* shmdata_get_mem(ShmdataWriterAccess access) {
  return static_cast<OneWriteAccess*>(access)->get_mem();
}

short shmdata_notify_clients(ShmdataWriterAccess access, size_t size) {
  return static_cast<OneWriteAccess*>(access)->notify_clients(size);
}

void shmdata_release_one_write_access(ShmdataWriterAccess access) {
  delete static_cast<OneWriteAccess*>(access);
}

unsigned long shmdata_get_shmmax(ShmdataLogger log) {
  return sysVShm::get_shmmax(static_cast<AbstractLogger*>(log));
}

unsigned long shmdata_get_shmmni(ShmdataLogger log) {
  return sysVShm::get_shmmni(static_cast<AbstractLogger*>(log));
}
