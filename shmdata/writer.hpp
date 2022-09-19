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

#ifndef _SHMDATA_WRITER_H_
#define _SHMDATA_WRITER_H_

#include <memory>
#include <string>

#include "./abstract-logger.hpp"
#include "./safe-bool-idiom.hpp"
#include "shmdata/sysv-sem.hpp"
#include "shmdata/sysv-shm.hpp"
#include "shmdata/unix-socket-protocol.hpp"
#include "shmdata/unix-socket-server.hpp"

namespace shmdata {

class OneWriteAccess;
class Writer : public SafeBoolIdiom {
  friend OneWriteAccess;

 public:
  /**
   * \brief Construct a Writer object.
   *
   * \param   path                  Shmdata path for listening incoming connections by Followers.
   * \param   memsize               Initial size of the shared memory. Note the shared memory
   *                                can be resized at each frame.
   * \param   data_desr             A string description for the frame to be transmitted. It is
   *                                expected to follow.  
   * \param   log                   Log object where to write internal logs.
   * \param   on_client_connect     Callback to be triggered when a follower connects.
   * \param   on_client_disconnect  Callback to be triggered when a follower disconnects.
   * \param   unix_permission       Permission to apply to the internal Unix socket, shared memory and semaphore.
   * 
   */
  Writer(const std::string& path,
         size_t memsize,
         const std::string& data_descr,
         AbstractLogger* log,
         UnixSocketProtocol::ServerSide::onClientConnect on_client_connect = nullptr,
         UnixSocketProtocol::ServerSide::onClientDisconnect on_client_disconnect = nullptr,
         mode_t unix_permission = 0660);
  /**
   * \brief Destruct the Writer and releases resources.
   *
   */
  ~Writer() override = default;
  Writer() = delete;
  Writer(const Writer&) = delete;
  Writer& operator=(const Writer&) = delete;
  Writer& operator=(Writer&&) = delete;

  /**
   * \brief Get currently allocated size of the shared memory used by the writer.
   *
   */
  size_t alloc_size() const;

  /**
   * \brief Copy a frame of data to the shmdata.
   *
   * \param data  Pointer to the begining of the frame.
   * \param size  Size of the frame to copy.
   *
   * \return Success of the copy to the shared memory
   *
   */
  bool copy_to_shm(const void* data, size_t size);

  /**
   * \brief Provide direct access to the memory with lock. The locked/unlocked state of the shared
   * memory is synchronized with the life of the returned value.
   *
   * \return OneWriteAccess object in a unique pointer. Its destruction release the lock.
   *
   */
  std::unique_ptr<OneWriteAccess> get_one_write_access();
  /**
   * \brief Provide lock and resize simultaneously.
   * Same as the get_one_write_access method, but allows to resize the shared memory before
   * the new access
   *
   * \param new_size New size to be allocated.
   *
   * \return OneWriteAccess object in a unique pointer. Its destruction release the lock.
   *
   */
  std::unique_ptr<OneWriteAccess> get_one_write_access_resize(size_t new_size);

  /**
   * \brief Provide lock to the memory.
   * Pointer version of get_one_write_access.
   *
   * \return Pointer to the create OneWriteAccess.
   *
   */ 
  OneWriteAccess* get_one_write_access_ptr();

  /**
   * \brief Provide lock and resize simultaneously.
   * Pointer version of get_one_write_access_resize
   *
   * \return Pointer to the create OneWriteAccess.
   *
   */ 
  OneWriteAccess* get_one_write_access_ptr_resize(size_t new_size);

 private:
  std::string path_;
  UnixSocketProtocol::onConnectData connect_data_;
  UnixSocketProtocol::ServerSide proto_;
  std::unique_ptr<UnixSocketServer> srv_;
  std::unique_ptr<sysVShm> shm_;
  std::unique_ptr<sysVSem> sem_;
  AbstractLogger* log_;
  size_t alloc_size_;
  bool is_valid_{true};
  bool is_valid() const final { return is_valid_; }
};

// see check-shmdata
class OneWriteAccess {
  friend Writer;

 public:
  /**
   * \brief Get the pointer to the shmdata memory.
   *
   * \return The pointer to the shmdata memory
   */
  void* get_mem() { return mem_; };

  /**
   * \brief Resize the shmdata with reallocation of a new uninitialized shared memory space.
   *
   * \note This reinitialize the memory. You probably want to apply writes after resizing.
   *
   * \param newsize Expected new size of the shmdata memory.
   *
   * \return Allocated size, or 0 if resize failed. 
   */
  size_t shm_resize(size_t newsize);

  /**
   * \brief Notify shmdata clients.
   *
   * \note This method must be called only once.
   *
   * \param size Size of the frame to be available for the clients.
   *
   * \return Number of notified clients. 
   *
   */
  short notify_clients(size_t size);
  ~OneWriteAccess() = default;
  OneWriteAccess() = delete;
  OneWriteAccess(const OneWriteAccess&) = delete;
  OneWriteAccess& operator=(const OneWriteAccess&) = delete;
  OneWriteAccess& operator=(OneWriteAccess&&) = default;

 private:
  OneWriteAccess(
      Writer* writer, sysVSem* sem, void* mem, UnixSocketServer* srv, AbstractLogger* log);
  Writer* writer_;
  WriteLock wlock_;
  void* mem_;
  UnixSocketServer* srv_;
  AbstractLogger* log_;
  bool has_notified_{false};
};

}  // namespace shmdata
#endif
