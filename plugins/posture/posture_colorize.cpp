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

#include "./posture_colorize.hpp"

#include <iostream>
#include <regex>
#include <thread>

using namespace std;
using namespace switcher::data;
using namespace posture;

namespace switcher {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(PostureColorize,
                                     "Project texture onto mesh",
                                     "video",
                                     "Project texture onto mesh, based on calibration",
                                     "LGPL",
                                     "texturetomeshsink", "Emmanuel Durand");

PostureColorize::PostureColorize():
    custom_props_(std::make_shared<CustomPropertyHelper> ()) {
}

PostureColorize::~PostureColorize() {
  stop();
}

bool
PostureColorize::start() {
  if (is_started())
    return false;

  colorize_ = make_shared<Colorize>();

  colorize_->setCalibrationPath(calibration_path_);

  return true;
}

bool
PostureColorize::stop() {
  lock_guard<mutex> lock(mutex_);

  if (colorize_ != nullptr)
    colorize_.reset();

  mesh_writer_.reset();
  clear_shmdatas();

  return true;
}

bool
PostureColorize::init() {
  init_startable(this);
  init_segment(this);

  install_connect_method(std::bind(&PostureColorize::connect, this, std::placeholders::_1),
                         std::bind(&PostureColorize::disconnect, this, std::placeholders::_1),
                         std::bind(&PostureColorize::disconnect_all, this),
                         std::bind(&PostureColorize::can_sink_caps, this, std::placeholders::_1),
                         1);


  calibration_path_prop_ = custom_props_->make_string_property("calibration_path",
                                          "Path to the calibration file",
                                          calibration_path_.c_str(),
                                          (GParamFlags) G_PARAM_READWRITE,
                                          PostureColorize::set_calibration_path,
                                          PostureColorize::get_calibration_path,
                                          this);
  install_property_by_pspec(custom_props_->get_gobject(),
                            calibration_path_prop_, "calibration_path",
                            "Path to the calibration file");

  return true;
}

bool
PostureColorize::connect(std::string shmdata_socket_path) {
  int id = source_id_;
  source_id_++;

  ShmdataAnyReader::ptr reader = make_shared<ShmdataAnyReader>();
  reader->set_path(shmdata_socket_path);

  reader->set_callback([=] (void *data,
                            int size,
                            unsigned long long /*unused*/,
                            const char *type,
                            void * /*unused*/)
  {
    if (colorize_ == nullptr)
      return;

    if (!mutex_.try_lock())
      return;

    unsigned int width, height, channels;
    // Update the input mesh. This calls the update of colorize_
    if (string(type) == string(POLYGONMESH_TYPE_BASE)) {
      mesh_ = vector<unsigned char>((unsigned char*)data, (unsigned char*)data + size);

      if (images_.size() == 0)
      {
        mutex_.unlock();
        return;
      }

      thread computeThread = thread([=] () {
        colorize_->setInput(mesh_, images_, dims_);
        vector<unsigned char> texturedMesh = colorize_->getTexturedMesh();

        if (mesh_writer_ == nullptr)
        {
          mesh_writer_ = make_shared<ShmdataAnyWriter>();
          mesh_writer_->set_path(make_file_name("mesh"));
          mesh_writer_->set_data_type(POLYGONMESH_TYPE_BASE);
          register_shmdata(mesh_writer_);
          mesh_writer_->start();
        }

        check_buffers();
        shmwriter_queue_.push_back(make_shared<vector<unsigned char>>(texturedMesh.data(), texturedMesh.data() + texturedMesh.size()));
        mesh_writer_->push_data_auto_clock((void*) shmwriter_queue_[shmwriter_queue_.size() - 1]->data(),
                                           texturedMesh.size(),
                                           PostureColorize::free_sent_buffer,
                                           (void*)(shmwriter_queue_[shmwriter_queue_.size() - 1].get()));

        mutex_.unlock();
      });

      computeThread.detach();
    }
    // Update the input textures
    else if (check_image_caps(string(type), width, height, channels)) {
      if (shm_index_.find(id) == shm_index_.end()) {
        shm_index_[id] = shm_index_.size() - 1;

        images_.resize(shm_index_.size());
        dims_.resize(shm_index_.size());
      }
      int index = shm_index_[id];

      images_[index] = vector<unsigned char>((unsigned char*)data, (unsigned char*)data + size);
      dims_[index] = vector<unsigned int>({width, height, channels});
    }

    mutex_.unlock();
  },
  nullptr);

  reader->start();
  register_shmdata(reader);

  return true;
}

bool
PostureColorize::disconnect(std::string /*unused*/) {
  return true;
}

bool
PostureColorize::disconnect_all() {
  return true;
}

bool
PostureColorize::can_sink_caps(std::string caps) {
  if (has_input_mesh_ && caps == POLYGONMESH_TYPE_BASE)
    return false;
  else if (caps == POLYGONMESH_TYPE_BASE)
    return true;

  unsigned int w, h, c;
  if (check_image_caps(caps, w, h, c))
    return true;

  return false;
}

bool
PostureColorize::check_image_caps(string caps, unsigned int& width, unsigned int& height, unsigned int& channels)
{
  int bpp, red, green, blue;
  red = green = blue = 0;

  regex regRgb, regBpp, regWidth, regHeight, regRed, regGreen, regBlue;
  try
  {
      regRgb = regex("(.*video/x-raw-rgb)(.*)", regex_constants::extended);
      regBpp = regex("(.*bpp=\\(int\\))(.*)", regex_constants::extended);
      regWidth = regex("(.*width=\\(int\\))(.*)", regex_constants::extended);
      regHeight = regex("(.*height=\\(int\\))(.*)", regex_constants::extended);
      regRed = regex("(.*red_mask=\\(int\\))(.*)", regex_constants::extended);
      regGreen = regex("(.*green_mask=\\(int\\))(.*)", regex_constants::extended);
      regBlue = regex("(.*blue_mask=\\(int\\))(.*)", regex_constants::extended);
  }
  catch (const regex_error& e)
  {
      cout << "PostureColorize::" << __FUNCTION__ << " - Regex error code: " << e.code() << endl;
      return false;
  }
  
  if (regex_match(caps, regRgb))
  {
    smatch match;
    string substr, format;
  
    if (regex_match(caps, match, regBpp))
    {
      ssub_match subMatch = match[2];
      substr = subMatch.str();
      sscanf(substr.c_str(), ")%u", &bpp);
    }
    if (regex_match(caps, match, regWidth))
    {
      ssub_match subMatch = match[2];
      substr = subMatch.str();
      sscanf(substr.c_str(), ")%u", &width);
    }
    if (regex_match(caps, match, regHeight))
    {
      ssub_match subMatch = match[2];
      substr = subMatch.str();
      sscanf(substr.c_str(), ")%u", &height);
    }
    if (regex_match(caps, match, regRed))
    {
      ssub_match subMatch = match[2];
      substr = subMatch.str();
      sscanf(substr.c_str(), ")%i", &red);
    }
    if (regex_match(caps, match, regGreen))
    {
      ssub_match subMatch = match[2];
      substr = subMatch.str();
      sscanf(substr.c_str(), ")%i", &green);
    }
    if (regex_match(caps, match, regBlue))
    {
      ssub_match subMatch = match[2];
      substr = subMatch.str();
      sscanf(substr.c_str(), ")%i", &blue);
    }
  
    if (bpp == 24)
      channels = 3;
    else
      return false;

    if (red == 16711680 && green == 65280 && blue == 255)
      return true;
    else
      return false;
  }
  else
  {
    return false;
  }
}

const gchar *
PostureColorize::get_calibration_path(void *user_data) {
  PostureColorize *ctx = (PostureColorize *) user_data;
  return ctx->calibration_path_.c_str();
}

void
PostureColorize::set_calibration_path(const gchar *name, void *user_data) {
  PostureColorize *ctx = (PostureColorize *) user_data;
  if (name != nullptr)
    ctx->calibration_path_ = name;
}

void
PostureColorize::free_sent_buffer(void* data)
{
  vector<unsigned char>* buffer = static_cast<vector<unsigned char>*>(data);
  buffer->clear();
}

void
PostureColorize::check_buffers()
{
  for (unsigned int i = 0; i < shmwriter_queue_.size();) {
    if (shmwriter_queue_[i]->size() == 0)
      shmwriter_queue_.erase(shmwriter_queue_.begin() + i);
    else
      i++;
  }
}

}  // namespace switcher
