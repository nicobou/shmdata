/*
 *
 * posture is free software; you can redistribute it and/or
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

#include "./posture_detect.hpp"

#include <iostream>
#include <thread>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureDetect,
                                     "Point Cloud Detect",
                                     "video",
                                     "Detect shapes and objects in point clouds",
                                     "LGPL",
                                     "pcldetectsink", "Emmanuel Durand");

PostureDetect::PostureDetect():
    custom_props_(std::make_shared<CustomPropertyHelper> ()) {
}

PostureDetect::~PostureDetect() {
  stop();
}

bool
PostureDetect::start() {
  detect_ = make_shared<Detect>();

  return true;
}

bool
PostureDetect::stop() {
  lock_guard<mutex> lock(mutex_);

  if (detect_ != nullptr) {
    detect_.reset();
  }

  if (cloud_writer_ != nullptr) {
    unregister_shmdata(cloud_writer_->get_path());
    cloud_writer_.reset();
  }

  return true;
}

bool
PostureDetect::init() {
  init_startable(this);
  init_segment(this);

  install_connect_method(std::bind(&PostureDetect::connect, this, std::placeholders::_1),
                         std::bind(&PostureDetect::disconnect, this, std::placeholders::_1),
                         std::bind(&PostureDetect::disconnect_all, this),
                         std::bind(&PostureDetect::can_sink_caps, this, std::placeholders::_1),
                         1);

  return true;
}

bool
PostureDetect::connect(std::string shmdata_socket_path) {
  ShmdataAnyReader::ptr reader = make_shared<ShmdataAnyReader>();
  reader->set_path(shmdata_socket_path);

  // This is the callback for when new clouds are received
  reader->set_callback([=] (void *data,
                             int size,
                             unsigned long long /*unused*/,
                             const char *type,
                             void * /*unused*/ )
  {
    // If another thread is trying to get the merged cloud, don't bother
    if (!mutex_.try_lock())
      return;

    if (detect_ == nullptr || (string(type) != string(POINTCLOUD_TYPE_BASE) && string(type) != string(POINTCLOUD_TYPE_COMPRESSED)))
    {
      mutex_.unlock();
      return;
    }

    // Setting input clouds is thread safe, so lets do it
    detect_->setInputCloud(vector<char>((char*)data, (char*) data + size),
                           string(type) != string(POINTCLOUD_TYPE_BASE));

    thread computeThread = thread([&]() {
      if (cloud_writer_.get() == nullptr) {
        cloud_writer_.reset(new ShmdataAnyWriter);
        cloud_writer_->set_path(make_file_name("cloud"));
        register_shmdata(cloud_writer_);
        if (compress_cloud_)
          cloud_writer_->set_data_type(string(POINTCLOUD_TYPE_COMPRESSED));
        else
          cloud_writer_->set_data_type(string(POINTCLOUD_TYPE_BASE));
        cloud_writer_->start();
      }

      check_buffers();
      if (detect_->detect()) {
        vector<char> cloud = detect_->getProcessedCloud();
        shmwriter_queue_.push_back(make_shared<vector<unsigned char>>(reinterpret_cast<const unsigned char*>(cloud.data()),
                                                                      reinterpret_cast<const unsigned char*>(cloud.data()) + cloud.size()));
        cloud_writer_->push_data_auto_clock((void *) shmwriter_queue_[shmwriter_queue_.size() - 1]->data(),
                                            cloud.size(),
                                            PostureDetect::free_sent_buffer,
                                            (void*)(shmwriter_queue_[shmwriter_queue_.size() - 1].get()));
      }

      mutex_.unlock();
    });

    computeThread.detach();
  },
  nullptr);

  reader->start();
  register_shmdata(reader);
  return true;
}

bool
PostureDetect::disconnect(std::string shmName) {
  std::lock_guard<mutex> lock(mutex_);
  unregister_shmdata(shmName);
  return true;
}

bool
PostureDetect::disconnect_all() {
  return true;
}

bool
PostureDetect::can_sink_caps(string caps) {
  return (caps == POINTCLOUD_TYPE_BASE)
      || (caps == POINTCLOUD_TYPE_COMPRESSED);
}

void
PostureDetect::free_sent_buffer(void* data)
{
  vector<unsigned char>* buffer = static_cast<vector<unsigned char>*>(data);
  buffer->clear();
}

void
PostureDetect::check_buffers()
{
  for (unsigned int i = 0; i < shmwriter_queue_.size();) {
    if (shmwriter_queue_[i]->size() == 0)
      shmwriter_queue_.erase(shmwriter_queue_.begin() + i);
    else
      i++;
  }
}
}  // namespace switcher
