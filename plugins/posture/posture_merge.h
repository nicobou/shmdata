/*
 * This file is part of posture.
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

#ifndef __SWITCHER_POSTURE_MERGE_H__
#define __SWITCHER_POSTURE_MERGE_H__

#include <posture.h>

#include "switcher/quiddity.h"
#include "switcher/segment.h"
#include "switcher/startable-quiddity.h"
#include "switcher/custom-property-helper.h"

#include <memory>
#include <mutex>
#include <string>

namespace switcher
{
  class PostureMerge : public Quiddity, public Segment, public StartableQuiddity
  {
  public:
    SWITCHER_DECLARE_QUIDDITY_PUBLIC_MEMBERS(PostureMerge);
    PostureMerge ();
    ~PostureMerge ();
    PostureMerge (const PostureMerge &) = delete;
    PostureMerge &operator= (const PostureMerge &) = delete;

    bool start ();
    bool stop ();

  private:
    CustomPropertyHelper::ptr custom_props_;
    std::string calibration_path_ {"default.kvc"};
    std::string devices_path_ {"devices.kvc"};
    bool compress_cloud_ {false};
    GParamSpec* calibration_path_prop_ {nullptr};
    GParamSpec* devices_path_prop_ {nullptr};
    GParamSpec* compress_cloud_prop_ {nullptr};

    unsigned int source_id_ {0};
    std::shared_ptr<posture::PointCloudMerger> merger_ {nullptr};
    std::mutex mutex_ {};

    ShmdataAnyWriter::ptr cloud_writer_ {nullptr};

    std::shared_ptr<std::vector<char>> cloud_buffers_[3];
    unsigned int cloud_buffer_index_ {0};

    bool init () final;
    
    bool connect (std::string shmdata_socket_path);
    bool disconnect_all ();
    bool can_sink_caps (std::string caps);

    static const gchar* get_calibration_path(void* user_data);
    static void set_calibration_path(const gchar* name, void* user_data);
    static const gchar* get_devices_path(void* user_data);
    static void set_devices_path(const gchar* name, void* user_data);
    static int get_compress_cloud(void* user_data);
    static void set_compress_cloud(const int compress, void* user_data);
  };
  
  SWITCHER_DECLARE_PLUGIN(PostureMerge);

}  // end of namespace

#endif // ifndef
