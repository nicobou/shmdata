/*
 * This file is part of switcher-v4l2.
 *
 * switcher-v4l2 is free software: you can redistribute it and/or modify
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

#include "./v4l2src.hpp"
#include <errno.h>
#include <fcntl.h>
#include <gio/gio.h>
#include <gst/video/video.h>
#include <linux/videodev2.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdlib>  // For srand() and rand()
#include <ctime>    // For time()
#include "switcher/gst/utils.hpp"
#include "switcher/quiddity/container.hpp"
#include "switcher/utils/file-utils.hpp"
#include "switcher/utils/scope-exit.hpp"

namespace switcher {
namespace quiddities {
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(V4L2Src,
                                     "v4l2src",
                                     "v4l2 Video Capture",
                                     "video",
                                     "writer/device",
                                     "Discover and use v4l2 supported capture cards and cameras",
                                     "GPL",
                                     "Nicolas Bouillot");

const std::string V4L2Src::kConnectionSpec(R"(
{
"writer":
  [
    {
      "label": "video",
      "description": "Video stream",
      "can_do": ["video/x-raw"]
    },
    {
      "label": "video-encoded",
      "description": "Encoded video stream",
      "can_do": [ "video/x-h264", "video/x-h265", "video/mpeg", "image/jpeg" ]
    }
  ]
}
)");

void V4L2Src::set_shm_suffix() {
  if (is_current_pixel_format_raw_video()) {
    shmpath_ = claw_.get_shmpath_from_writer_label("video");
  } else {
    shmpath_ = claw_.get_shmpath_from_writer_label("video-encoded");
  }
  g_object_set(G_OBJECT(shmsink_.get_raw()), "socket-path", shmpath_.c_str(), nullptr);
  auto extra_caps = get_quiddity_caps();
  g_object_set(G_OBJECT(shmsink_.get_raw()), "extra-caps-properties", extra_caps.c_str(), nullptr);
}

V4L2Src::V4L2Src(quiddity::Config&& conf)
    : Quiddity(std::forward<quiddity::Config>(conf)),
      Startable(this),
      gst_pipeline_(std::make_unique<gst::Pipeliner>(
          nullptr, nullptr, [this](GstObject* gstobj, GError* err) {
            on_gst_error(gstobj, err);
          })) {
  force_framerate_id_ = pmanage<MPtr(&property::PBag::make_bool)>(
      "force_framerate",
      [this](bool val) {
        force_framerate_ = val;
        return true;
      },
      [this]() { return force_framerate_; },
      "Force framerate",
      "Force the output framerate to be the one announced by the v4l device, "
      "dropping/adding frame if necessary.",
      force_framerate_);

  if (!v4l2src_ || !videoconvert_ || !deinterlace_ || !capsfilter_ || !videorate_ || !shmsink_) {
    is_valid_ = false;
    return;
  }
  // device inspector
  check_folder_for_v4l2_devices();

  update_capture_device();

  if (capture_devices_.empty()) {
    error("no video4linux device detected.");
    is_valid_ = false;
    return;
  }
  devices_id_ = pmanage<MPtr(&property::PBag::make_selection<>)>(
      "device",
      [this](const quiddity::property::IndexOrName& val) {
        if (is_loading_) return false;
        devices_enum_.select(val);
        update_device_specific_properties();
        return true;
      },
      [this]() { return devices_enum_.get(); },
      "Capture Device",
      "Enumeration of v4l2 capture devices",
      devices_enum_);
  group_id_ = pmanage<MPtr(&property::PBag::make_group)>(
      "config", "Capture device configuration", "device specific parameters");

  update_device_specific_properties();

  deinterlace_mode_id_ = pmanage<MPtr(&property::PBag::make_parented_selection<>)>(
      "deinterlace_mode",
      "config",
      [this](const quiddity::property::IndexOrName& val) {
        deinterlace_mode_.select(val);
        return true;
      },
      [this]() { return deinterlace_mode_.get(); },
      "Deinterlace mode",
      "Choose whether input should be deinterlaced or not",
      deinterlace_mode_);

  pmanage<MPtr(&property::PBag::make_group)>(
      "advanced", "Advanced configuration", "Advanced configuration");
  save_device_id_ = pmanage<MPtr(&property::PBag::make_parented_selection<>)>(
      "save_mode",
      "advanced",
      [this](const quiddity::property::IndexOrName& val) {
        save_device_enum_.select(val);
        return true;
      },
      [this]() { return save_device_enum_.get(); },
      "Save Mode",
      "Save Capture Device by",
      save_device_enum_);
  set_shm_suffix();
}

std::string V4L2Src::fetch_current_framerate() {
  CaptureDescription& description = capture_devices_[devices_enum_.get_current_index()];
  int fd = open(description.absolute_path_.c_str(), O_RDONLY);
  if (fd < 0) {
    error("could not open device %", description.absolute_path_);
    return "";
  }

  On_scope_exit { close(fd); };

  v4l2_streamparm src_parm;
  memset(&src_parm, 0, sizeof(src_parm));
  src_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == ioctl(fd, VIDIOC_G_PARM, &src_parm)) {
    int err = errno;
    error("could not get video parameters. Error: %", std::string(strerror(err)));
    return "";
  }

  std::ostringstream stream;
  stream << std::setprecision(5)
         << (static_cast<float>(src_parm.parm.capture.timeperframe.denominator) /
             static_cast<float>(src_parm.parm.capture.timeperframe.numerator));
  std::string fps = stream.str();
  info("detected framerate: %", fps);

  return fps;
}

std::string V4L2Src::fetch_current_resolution() {
  CaptureDescription& description = capture_devices_[devices_enum_.get_current_index()];
  int fd = open(description.absolute_path_.c_str(), O_RDONLY);
  if (fd < 0) {
    error("could not open device %", description.absolute_path_);
    return "";
  }

  On_scope_exit { close(fd); };

  struct v4l2_capability vcap;
  if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &vcap)) {
    int err = errno;
    error("could not get video capture device id. Error: %", std::string(strerror(err)));
    return "";
  }

  auto capabilities = vcap.capabilities;
  auto priv_magic = (capabilities & V4L2_CAP_EXT_PIX_FORMAT) ? V4L2_PIX_FMT_PRIV_MAGIC : 0;
  auto is_multiplanar = capabilities & (V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_M2M_MPLANE |
                                        V4L2_CAP_VIDEO_OUTPUT_MPLANE);
  auto vidcap_buftype =
      is_multiplanar ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_CAPTURE;

  v4l2_format src_fmt;
  memset(&src_fmt, 0, sizeof(src_fmt));
  src_fmt.fmt.pix.priv = priv_magic;
  src_fmt.type = vidcap_buftype;

  if (-1 == ioctl(fd, VIDIOC_G_FMT, &src_fmt)) {
    int err = errno;
    error("could not get video format. Error: %", std::string(strerror(err)));
    return "";
  }

  std::ostringstream stream;
  stream << src_fmt.fmt.pix.width << "x" << src_fmt.fmt.pix.height;
  std::string res = stream.str();

  info("detected resolution: %", res);
  return res;
}

bool V4L2Src::fetch_available_resolutions() {
  CaptureDescription& description = capture_devices_[devices_enum_.get_current_index()];
  int fd = open(description.absolute_path_.c_str(), O_RDONLY);
  if (fd < 0) {
    error("could not open device %", description.absolute_path_);
    return false;
  }

  v4l2_frmsizeenum frmsize;
  memset(&frmsize, 0, sizeof(frmsize));
  frmsize.pixel_format =
      std::get<0>(description.pixel_formats_[pixel_format_enum_.get_current_index()]);
  frmsize.index = 0;
  unsigned default_width = 0;
  unsigned default_height = 0;

  description.frame_size_discrete_.clear();
  while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0 &&
         frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
    if (frmsize.index == 0) {
      default_width = frmsize.discrete.width;
      default_height = frmsize.discrete.height;
    }

    description.frame_size_discrete_.push_back(std::make_pair(
        std::to_string(frmsize.discrete.width), std::to_string(frmsize.discrete.height)));
    frmsize.index++;
  }

  if (frmsize.type != V4L2_FRMSIZE_TYPE_DISCRETE) {
    description.frame_size_stepwise_max_width_ = frmsize.stepwise.max_width;
    description.frame_size_stepwise_min_width_ = frmsize.stepwise.min_width;
    description.frame_size_stepwise_step_width_ = frmsize.stepwise.step_width;
    description.frame_size_stepwise_max_height_ = frmsize.stepwise.max_height;
    description.frame_size_stepwise_min_height_ = frmsize.stepwise.min_height;
    description.frame_size_stepwise_step_height_ = frmsize.stepwise.step_height;
    default_width = frmsize.stepwise.max_width;
    default_height = frmsize.stepwise.max_height;
    width_ = default_width;
    height_ = default_height;
  } else {
    description.frame_size_stepwise_max_width_ = -1;
    description.frame_size_stepwise_min_width_ = -1;
    description.frame_size_stepwise_step_width_ = -1;
    description.frame_size_stepwise_max_height_ = -1;
    description.frame_size_stepwise_min_height_ = -1;
    description.frame_size_stepwise_step_height_ = -1;
  }

  close(fd);

  return true;
}

bool V4L2Src::fetch_available_frame_intervals() {
  CaptureDescription& description = capture_devices_[devices_enum_.get_current_index()];
  int fd = open(description.absolute_path_.c_str(), O_RDONLY);
  if (fd < 0) {
    error("could not open device %", description.absolute_path_);
    return false;
  }

  v4l2_frmivalenum frmival;
  memset(&frmival, 0, sizeof(frmival));
  frmival.pixel_format =
      std::get<0>(description.pixel_formats_[pixel_format_enum_.get_current_index()]);

  // Only true with continuous resolution devices.
  if (width_ != -1) {
    frmival.width = width_;
  } else {
    frmival.width =
        atoi(description.frame_size_discrete_[resolutions_enum_.get_current_index()].first.c_str());
  }
  if (height_ != -1) {
    frmival.height = width_;
  } else {
    frmival.height = atoi(
        description.frame_size_discrete_[resolutions_enum_.get_current_index()].second.c_str());
  }

  frmival.index = 0;

  description.frame_interval_discrete_.clear();
  while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) >= 0 &&
         frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
    if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
      description.frame_interval_discrete_.push_back(
          std::make_pair(std::to_string(frmival.discrete.numerator),
                         std::to_string(frmival.discrete.denominator)));
    }
    frmival.index++;
  }

  if (frmival.type != V4L2_FRMIVAL_TYPE_DISCRETE) {
    description.frame_interval_stepwise_max_numerator_ = frmival.stepwise.max.numerator;
    description.frame_interval_stepwise_max_denominator_ = frmival.stepwise.max.denominator;
    description.frame_interval_stepwise_min_numerator_ = frmival.stepwise.max.numerator;
    description.frame_interval_stepwise_min_denominator_ = frmival.stepwise.max.denominator;
    description.frame_interval_stepwise_step_numerator_ = frmival.stepwise.step.numerator;
    description.frame_interval_stepwise_step_denominator_ = frmival.stepwise.step.denominator;
  } else {
    description.frame_interval_stepwise_max_numerator_ = -1;
    description.frame_interval_stepwise_max_denominator_ = -1;
    description.frame_interval_stepwise_min_numerator_ = -1;
    description.frame_interval_stepwise_min_denominator_ = -1;
    description.frame_interval_stepwise_step_numerator_ = -1;
    description.frame_interval_stepwise_step_denominator_ = -1;
  }

  close(fd);

  return true;
}

void V4L2Src::update_capture_device() {
  gint i = 0;
  std::vector<std::string> names;
  std::vector<std::string> nicks;
  for (auto& it : capture_devices_) {
    names.push_back(it.card_);
    nicks.push_back(it.file_device_);
    i++;
  }
  devices_enum_ = property::Selection<>(std::make_pair(names, nicks), 0);
}

void V4L2Src::update_device_specific_properties() {
  if (capture_devices_.empty()) return;
  update_pixel_format();
  update_tv_standard();
}

bool V4L2Src::is_current_pixel_format_raw_video() const {
  std::string video_raw("video/x-raw");
  if (std::string(pixel_format_enum_.get_attached(), 0, video_raw.size()) != video_raw)
    return false;
  return true;
}

void V4L2Src::update_pixel_format() {
  CaptureDescription& cap_descr = capture_devices_[devices_enum_.get_current_index()];
  pmanage<MPtr(&property::PBag::remove)>(pixel_format_id_);
  pixel_format_id_ = 0;
  if (cap_descr.pixel_formats_.empty()) return;
  std::vector<std::string> nicks;
  std::vector<std::string> names;
  for (auto& it : cap_descr.pixel_formats_) {
    nicks.push_back(std::get<1>(it));
    names.push_back(std::get<2>(it));
  }
  pixel_format_enum_ = property::Selection<>(std::make_pair(std::move(names), std::move(nicks)), 0);
  pixel_format_id_ = pmanage<MPtr(&property::PBag::make_parented_selection<>)>(
      "pixel_format",
      "config",
      [this](const quiddity::property::IndexOrName& val) {
        pixel_format_enum_.select(val);
        set_shm_suffix();
        debug("pix selected");
        fetch_available_resolutions();
        update_device_resolution();
        return true;
      },
      [this]() { return pixel_format_enum_.get(); },
      "Pixel format",
      "Pixel format of selected capture devices",
      pixel_format_enum_);

  pmanage<MPtr(&property::PBag::set<quiddity::property::IndexOrName>)>(
      pixel_format_id_, quiddity::property::IndexOrName(0));
}

void V4L2Src::update_device_resolution() {
  CaptureDescription& cap_descr = capture_devices_[devices_enum_.get_current_index()];
  pmanage<MPtr(&property::PBag::remove)>(resolutions_id_);
  resolutions_id_ = 0;
  pmanage<MPtr(&property::PBag::remove)>(width_id_);
  width_id_ = 0;
  pmanage<MPtr(&property::PBag::remove)>(height_id_);
  height_id_ = 0;

  if (!cap_descr.frame_size_discrete_.empty()) {
    width_ = -1;
    height_ = -1;
    std::vector<std::string> names;

    for (const auto& it : cap_descr.frame_size_discrete_)
      names.push_back(std::string(it.first) + "x" + std::string(it.second));

    resolutions_enum_ = property::Selection<>(std::move(names), 0);
    resolutions_id_ = pmanage<MPtr(&property::PBag::make_parented_selection<>)>(
        "resolution",
        "config",
        [this](const quiddity::property::IndexOrName& val) {
          resolutions_enum_.select(val);
          fetch_available_frame_intervals();
          update_device_framerate();
          return true;
        },
        [this]() { return resolutions_enum_.get(); },
        "Resolution",
        "Select resolution",
        resolutions_enum_);
  } else if (cap_descr.frame_size_stepwise_max_width_ >= 1) {
    resolutions_id_ = pmanage<MPtr(&property::PBag::make_parented_selection<property::Fraction>)>(
        "resolution",
        "config",
        [this](const quiddity::property::IndexOrName& val) {
          custom_resolutions_.select(val);
          if (custom_resolutions_.get_current() == "Custom") {
            pmanage<MPtr(&property::PBag::enable)>(width_id_);
            pmanage<MPtr(&property::PBag::enable)>(height_id_);
            return true;
          }
          auto fract = custom_resolutions_.get_attached();
          pmanage<MPtr(&property::PBag::set<int>)>(width_id_, fract.numerator());
          pmanage<MPtr(&property::PBag::set<int>)>(height_id_, fract.denominator());
          static const std::string why_disconnected =
              "this property is available only with custom resolution";
          pmanage<MPtr(&property::PBag::disable)>(width_id_, why_disconnected);
          pmanage<MPtr(&property::PBag::disable)>(height_id_, why_disconnected);
          return true;
        },
        [this]() { return custom_resolutions_.get(); },
        "Resolution",
        "Select resolution",
        custom_resolutions_);

    width_id_ =
        pmanage<MPtr(&property::PBag::make_parented_int)>("width",
                                                          "config",
                                                          [this](const int& val) {
                                                            width_ = val;
                                                            fetch_available_frame_intervals();
                                                            update_device_framerate();
                                                            return true;
                                                          },
                                                          [this]() { return width_; },
                                                          "Width",
                                                          "Capture width",
                                                          width_,
                                                          cap_descr.frame_size_stepwise_min_width_,
                                                          cap_descr.frame_size_stepwise_max_width_);
    height_id_ = pmanage<MPtr(&property::PBag::make_parented_int)>(
        "height",
        "config",
        [this](const int& val) {
          height_ = val;
          fetch_available_frame_intervals();
          update_device_framerate();
          return true;
        },
        [this]() { return height_; },
        "Height",
        "Capture height",
        height_,
        cap_descr.frame_size_stepwise_min_height_,
        cap_descr.frame_size_stepwise_max_height_);
  } else
    return;

  std::string current_res = fetch_current_resolution();
  if (!current_res.empty()) {
    info("Automatically setting resolution for video input %",
         capture_devices_[devices_enum_.get_current_index()].file_device_);
    pmanage<MPtr(&property::PBag::set<quiddity::property::IndexOrName>)>(resolutions_id_,
                                                                         current_res);
  } else {
    pmanage<MPtr(&property::PBag::set_to_current)>(resolutions_id_);
  }
}

void V4L2Src::update_device_framerate() {
  CaptureDescription& cap_descr = capture_devices_[devices_enum_.get_current_index()];
  pmanage<MPtr(&property::PBag::remove)>(standard_framerates_id_);
  standard_framerates_id_ = 0;
  pmanage<MPtr(&property::PBag::remove)>(custom_framerate_id_);
  custom_framerate_id_ = 0;
  pmanage<MPtr(&property::PBag::remove)>(framerates_enum_id_);
  framerates_enum_id_ = 0;

  if (!cap_descr.frame_interval_discrete_.empty()) {
    std::vector<std::string> names;
    for (const auto& it : cap_descr.frame_interval_discrete_) {
      // inversing enumerator and denominator because gst wants
      // framerate while v4l2 gives frame interval
      std::ostringstream stream;
      stream << std::setprecision(5) << (std::stoi(it.second) / std::stoi(it.first));
      names.push_back(stream.str());
    }

    framerates_enum_ = property::Selection<>(std::move(names), 0);
    framerates_enum_id_ = pmanage<MPtr(&property::PBag::make_parented_selection<>)>(
        "framerate",
        "config",
        [this](const quiddity::property::IndexOrName& val) {
          framerates_enum_.select(val);
          return true;
        },
        [this]() { return framerates_enum_.get(); },
        "Framerate",
        "Framerate of selected capture device",
        framerates_enum_);
  } else if (cap_descr.frame_interval_stepwise_max_numerator_ >= 1) {
    custom_framerate_id_ = pmanage<MPtr(&property::PBag::make_parented_fraction)>(
        "custom_framerate",
        "config",
        [this](const property::Fraction& val) {
          custom_framerate_ = val;
          return true;
        },
        [this]() { return custom_framerate_; },
        "Custom Framerate",
        "Custom capture framerate",
        custom_framerate_,
        1,
        1,
        2997,
        125);

    standard_framerates_id_ =
        pmanage<MPtr(&property::PBag::make_parented_selection<property::Fraction>)>(
            "standard_framerates",
            "config",
            [this](const quiddity::property::IndexOrName& val) {
              standard_framerates_.select(val);
              if (standard_framerates_.get_current() == "Custom") {
                pmanage<MPtr(&property::PBag::enable)>(custom_framerate_id_);
                return true;
              }
              auto fract = standard_framerates_.get_attached();
              pmanage<MPtr(&property::PBag::set<property::Fraction>)>(custom_framerate_id_, fract);
              static const std::string why_disconnected =
                  "this property is available only with custom frame rate";
              pmanage<MPtr(&property::PBag::disable)>(custom_framerate_id_, why_disconnected);
              return true;
            },
            [this]() { return standard_framerates_.get(); },
            "Framerate",
            "Framerate of selected capture device",
            standard_framerates_);
  } else
    return;

  std::string current_framerate = fetch_current_framerate();
  if (!current_framerate.empty()) {
    info("Automatically setting framerate for video input %",
         capture_devices_[devices_enum_.get_current_index()].file_device_);
    if (framerates_enum_id_ > 0) {
      pmanage<MPtr(&property::PBag::set<quiddity::property::IndexOrName>)>(framerates_enum_id_,
                                                                           current_framerate);
    } else {
      pmanage<MPtr(&property::PBag::set<quiddity::property::IndexOrName>)>(standard_framerates_id_,
                                                                           current_framerate);
    }
  } else {
    if (framerates_enum_id_ > 0) {
      pmanage<MPtr(&property::PBag::set_to_current)>(framerates_enum_id_);
    } else {
      pmanage<MPtr(&property::PBag::set_to_current)>(standard_framerates_id_);
    }
  }
}

void V4L2Src::update_tv_standard() {
  CaptureDescription& cap_descr = capture_devices_[devices_enum_.get_current_index()];
  pmanage<MPtr(&property::PBag::remove)>(tv_standards_id_);
  tv_standards_id_ = 0;
  if (cap_descr.tv_standards_.empty()) return;
  std::vector<std::string> names;
  for (auto& it : cap_descr.tv_standards_) names.push_back(it);
  tv_standards_enum_ = property::Selection<>(std::move(names), 0);
  tv_standards_id_ = pmanage<MPtr(&property::PBag::make_parented_selection<>)>(
      "tv_standard",
      "config",
      [this](const quiddity::property::IndexOrName& val) {
        tv_standards_enum_.select(val);
        return true;
      },
      [this]() { return tv_standards_enum_.get(); },
      "TV standard",
      "TV standard",
      tv_standards_enum_);
}

bool V4L2Src::remake_elements() {
  if (capture_devices_.empty()) {
    error("no capture device available for starting capture");
    return false;
  }
  if (!gst::UGstElem::renew(v4l2src_, {"device", "norm", "io-mode"}) ||
      !gst::UGstElem::renew(videoconvert_, {}) ||
      !gst::UGstElem::renew(deinterlace_, {"mode", "method"}) ||
      !gst::UGstElem::renew(videorate_) || !gst::UGstElem::renew(capsfilter_, {"caps"}) ||
      !gst::UGstElem::renew(shmsink_, {"socket-path", "extra-caps-properties"})) {
    error("could not renew elements for video capture");
    return false;
  }
  return true;
}

std::string V4L2Src::pixel_format_to_string(unsigned pf_id) {
  std::string pixfmt;
  pixfmt += (char)(pf_id & 0xff);
  pixfmt += (char)((pf_id >> 8) & 0xff);
  pixfmt += (char)((pf_id >> 16) & 0xff);
  pixfmt += (char)((pf_id >> 24) & 0xff);
  return pixfmt;
}

void V4L2Src::set_device_id(const std::string& file_path, const std::string& id) {
  // reading the device file for inspecting capability
  int fd = open(file_path.c_str(), O_RDONLY);
  if (fd < 0) {
    int err = errno;
    error("could not set device id. Error: %", std::string(strerror(err)));
    return;
  }
  On_scope_exit { close(fd); };

  struct v4l2_capability vcap;
  if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &vcap)) {
    int err = errno;
    error("could not get device capabilities. Error: %", std::string(strerror(err)));
    return;
  }
  const auto bus_info = std::string((char*)vcap.bus_info);
  auto it = std::find_if(
      capture_devices_.begin(), capture_devices_.end(), [&](const CaptureDescription& desc) {
        return bus_info == desc.bus_info_;
      });
  if (capture_devices_.end() == it) {
    error("could not set device id. % seems to be a video device, but is not listed internally",
          file_path);
    return;
  }
  it->device_id_ = id;
}

bool V4L2Src::inspect_file_device(const std::string& file_path) {
  int fd = open(file_path.c_str(), O_RDONLY);
  if (fd < 0) {
    error("could not open device %", file_path);
    return false;
  }
  On_scope_exit { close(fd); };

  CaptureDescription description;
  description.absolute_path_ = file_path;
  struct v4l2_capability vcap;
  if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &vcap)) {
    int err = errno;
    error("could not get device capabilities. Error: %", std::string(strerror(err)));
    return false;
  }
  description.file_device_ = file_path;
  description.card_ = (char*)vcap.card;
  description.bus_info_ = (char*)vcap.bus_info;
  description.device_id_ = description.bus_info_;
  description.driver_ = (char*)vcap.driver;

  // pixel format
  v4l2_fmtdesc fmt;
  unsigned default_pixel_format = 0;
  memset(&fmt, 0, sizeof(fmt));
  fmt.index = 0;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while (ioctl(fd, VIDIOC_ENUM_FMT, &fmt) >= 0) {
    if (fmt.pixelformat != 0) {
      if (default_pixel_format == 0) default_pixel_format = fmt.pixelformat;
      GstStructure* structure = gst_v4l2_object_v4l2fourcc_to_structure(fmt.pixelformat);
      if (nullptr != structure) {
        GstCaps* caps = gst_caps_new_full(structure, nullptr);
        On_scope_exit { gst_caps_unref(caps); };
        gchar* tmp = gst_caps_to_string(caps);
        On_scope_exit { g_free(tmp); };
        description.pixel_formats_.push_back(
            std::make_tuple(fmt.pixelformat, tmp, reinterpret_cast<const char*>(fmt.description)));
      } else {
        warning("pixel format % not supported", pixel_format_to_string(fmt.pixelformat));
      }
    }
    fmt.index++;
  }

  if (default_pixel_format == 0) {
    error("no default pixel format found for %", file_path);
    return false;
  }

  v4l2_standard std;
  memset(&std, 0, sizeof(std));
  std.index = 0;
  description.tv_standards_.push_back("none");
  while (ioctl(fd, VIDIOC_ENUMSTD, &std) >= 0) {
    description.tv_standards_.push_back((char*)std.name);
    std.index++;
  }

  capture_devices_.push_back(description);

  return true;
}

std::vector<std::string> V4L2Src::get_file_names_with_prefix(const std::string& dir_path,
                                                             const std::string& prefix) {
  // FIXME move this to a file utils and reimplement without glib
  std::vector<std::string> res;
  GFile* inspected_dir = g_file_new_for_commandline_arg(dir_path.c_str());
  On_scope_exit { g_object_unref(inspected_dir); };
  GError* error;
  GFileEnumerator* enumerator;
  GFileInfo* info;
  error = nullptr;
  enumerator = g_file_enumerate_children(
      inspected_dir, "*", G_FILE_QUERY_INFO_NOFOLLOW_SYMLINKS, nullptr, &error);
  if (!enumerator) return res;
  error = nullptr;
  info = g_file_enumerator_next_file(enumerator, nullptr, &error);
  while ((info) && (!error)) {
    GFile* descend = g_file_get_child(inspected_dir, g_file_info_get_name(info));
    char* filename = g_file_get_basename(descend);
    On_scope_exit {
      if (nullptr != filename) g_free(filename);
    };
    if (prefix.empty()) {
      res.push_back(filename);
    } else {
      if (g_str_has_prefix(filename, prefix.c_str())) {
        res.push_back(filename);
      }
    }
    g_object_unref(descend);
    error = nullptr;
    info = g_file_enumerator_next_file(enumerator, nullptr, &error);
  }
  error = nullptr;
  g_file_enumerator_close(enumerator, nullptr, &error);
  return res;
}

bool V4L2Src::check_folder_for_v4l2_devices() {
  auto files = get_file_names_with_prefix("/dev/", "video");
  if (files.empty()) {
    warning("no v4l2 device found");
    return false;
  }
  for (auto& it : files) inspect_file_device("/dev/" + it);

  // push USB entries at the end of capture_devices_
  std::sort(capture_devices_.begin(),
            capture_devices_.end(),
            [](const CaptureDescription&, const CaptureDescription& descr2) {
              auto tmp = descr2.bus_info_;
              std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
              if (std::string::npos != tmp.rfind("usb")) return true;
              return false;
            });

  auto files_by_id = get_file_names_with_prefix("/dev/v4l/by-id/", {});
  for (auto& it : files_by_id) set_device_id("/dev/v4l/by-id/" + it, it);
  return true;
}

bool V4L2Src::start() {
  configure_capture();
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);

  gst_bin_add_many(
      GST_BIN(gst_pipeline_->get_pipeline()), v4l2src_.get_raw(), capsfilter_.get_raw(), nullptr);
  gst_element_link_many(v4l2src_.get_raw(), capsfilter_.get_raw(), nullptr);

  if (is_current_pixel_format_raw_video()) {
    gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                     videoconvert_.get_raw(),
                     deinterlace_.get_raw(),
                     nullptr);
    gst_element_link_many(
        capsfilter_.get_raw(), videoconvert_.get_raw(), deinterlace_.get_raw(), nullptr);
  }

  if (force_framerate_) {
    gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), videorate_.get_raw());
    if (is_current_pixel_format_raw_video()) {
      gst_element_link(deinterlace_.get_raw(), videorate_.get_raw());
    } else {
      gst_element_link(capsfilter_.get_raw(), videorate_.get_raw());
    }
  }

  gst_bin_add(GST_BIN(gst_pipeline_->get_pipeline()), shmsink_.get_raw());
  if (force_framerate_) {
    gst_element_link(videorate_.get_raw(), shmsink_.get_raw());
  } else if (is_current_pixel_format_raw_video()) {
    gst_element_link(deinterlace_.get_raw(), shmsink_.get_raw());
  } else {
    gst_element_link(capsfilter_.get_raw(), shmsink_.get_raw());
  }

  shm_sub_ = std::make_unique<shmdata::GstTreeUpdater>(
      this, shmsink_.get_raw(), shmpath_, shmdata::GstTreeUpdater::Direction::writer);

  gst_pipeline_->play(true);
  pmanage<MPtr(&property::PBag::disable)>(devices_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(force_framerate_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(group_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(resolutions_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(width_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(height_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(tv_standards_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(standard_framerates_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(framerates_enum_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(custom_framerate_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(pixel_format_id_, disabledWhenStartedMsg);
  pmanage<MPtr(&property::PBag::disable)>(deinterlace_mode_id_, disabledWhenStartedMsg);
  return true;
}

bool V4L2Src::stop() {
  shm_sub_.reset();
  remake_elements();
  gst_pipeline_ = std::make_unique<gst::Pipeliner>(
      nullptr, nullptr, [this](GstObject* gstobj, GError* err) { on_gst_error(gstobj, err); });
  pmanage<MPtr(&property::PBag::enable)>(devices_id_);
  pmanage<MPtr(&property::PBag::enable)>(force_framerate_id_);
  pmanage<MPtr(&property::PBag::enable)>(group_id_);
  pmanage<MPtr(&property::PBag::enable)>(resolutions_id_);
  pmanage<MPtr(&property::PBag::enable)>(width_id_);
  pmanage<MPtr(&property::PBag::enable)>(height_id_);
  pmanage<MPtr(&property::PBag::enable)>(tv_standards_id_);
  pmanage<MPtr(&property::PBag::enable)>(standard_framerates_id_);
  pmanage<MPtr(&property::PBag::enable)>(framerates_enum_id_);
  pmanage<MPtr(&property::PBag::enable)>(custom_framerate_id_);
  pmanage<MPtr(&property::PBag::enable)>(pixel_format_id_);
  pmanage<MPtr(&property::PBag::enable)>(deinterlace_mode_id_);
  return true;
}

bool V4L2Src::configure_capture() {
  if (capture_devices_.empty()) {
    error("no device available for capture");
    return false;
  }
  g_object_set(
      G_OBJECT(v4l2src_.get_raw()), "device", devices_enum_.get_attached().c_str(), nullptr);
  if (tv_standards_id_ != 0 && tv_standards_enum_.get_current_index() > 0)  // 0 is none
    g_object_set(
        G_OBJECT(v4l2src_.get_raw()), "norm", tv_standards_enum_.get_current().c_str(), nullptr);

  // When using a videorate element we need to copy the buffers between elements otherwise we get
  // stuck.
  if (force_framerate_) g_object_set(G_OBJECT(v4l2src_.get_raw()), "io_mode", 1, nullptr);

  g_object_set(G_OBJECT(deinterlace_.get_raw()),
               "mode",
               deinterlace_mode_.get_current_index(),
               "method",
               4,
               nullptr);

  std::string caps = pixel_format_enum_.get_attached();
  if (0 != width_id_)
    caps = caps + ", width=(int)" + std::to_string(width_) + ", height=(int)" +
           std::to_string(height_);
  else if (0 != resolutions_id_)
    caps = caps + ", width=(int)" +
           capture_devices_[devices_enum_.get_current_index()]
               .frame_size_discrete_[resolutions_enum_.get_current_index()]
               .first.c_str() +
           ", height=(int)" +
           capture_devices_[devices_enum_.get_current_index()]
               .frame_size_discrete_[resolutions_enum_.get_current_index()]
               .second.c_str();
  if (0 != framerates_enum_id_)
    caps = caps + ", framerate=(fraction)" +
           capture_devices_[devices_enum_.get_current_index()]
               .frame_interval_discrete_[framerates_enum_.get_current_index()]
               .second.c_str() +
           "/" +
           capture_devices_[devices_enum_.get_current_index()]
               .frame_interval_discrete_[framerates_enum_.get_current_index()]
               .first.c_str();
  else if (0 != custom_framerate_id_) {
    caps = caps + ", framerate=(fraction)" + std::to_string(custom_framerate_.numerator()) + "/" +
           std::to_string(custom_framerate_.denominator());
  }
  debug("caps for v4l2src: %", caps);
  GstCaps* usercaps = gst_caps_from_string(caps.c_str());
  g_object_set(G_OBJECT(capsfilter_.get_raw()), "caps", usercaps, nullptr);
  gst_caps_unref(usercaps);
  return true;
}

GstStructure* V4L2Src::gst_v4l2_object_v4l2fourcc_to_structure(guint32 fourcc) {
  GstStructure* structure = NULL;

  switch (fourcc) {
    case V4L2_PIX_FMT_MJPEG: /* Motion-JPEG */
#ifdef V4L2_PIX_FMT_PJPG
    case V4L2_PIX_FMT_PJPG: /* Progressive-JPEG */
#endif
    case V4L2_PIX_FMT_JPEG: /* JFIF JPEG */
      structure = gst_structure_new_empty("image/jpeg");
      break;
    case V4L2_PIX_FMT_YYUV:  /* 16  YUV 4:2:2     */
    case V4L2_PIX_FMT_HI240: /*  8  8-bit color   */
      /* FIXME: get correct fourccs here */
      break;
#ifdef V4L2_PIX_FMT_MPEG4
    case V4L2_PIX_FMT_MPEG4:
      structure = gst_structure_new(
          "video/mpeg", "mpegversion", G_TYPE_INT, 4, "systemstream", G_TYPE_BOOLEAN, FALSE, NULL);
      break;
#endif
#ifdef V4L2_PIX_FMT_H263
    case V4L2_PIX_FMT_H263:
      structure = gst_structure_new("video/x-h263", "variant", G_TYPE_STRING, "itu", NULL);
      break;
#endif
#ifdef V4L2_PIX_FMT_H264
    case V4L2_PIX_FMT_H264: /* H.264 */
      structure = gst_structure_new_empty("video/x-h264");
      break;
#endif
    case V4L2_PIX_FMT_RGB332:
    case V4L2_PIX_FMT_RGB555X:
    case V4L2_PIX_FMT_RGB565X:
      /* FIXME: get correct fourccs here */
      break;
    case V4L2_PIX_FMT_GREY: /*  8  Greyscale     */
    case V4L2_PIX_FMT_RGB555:
    case V4L2_PIX_FMT_RGB565:
    case V4L2_PIX_FMT_RGB24:
    case V4L2_PIX_FMT_BGR24:
    case V4L2_PIX_FMT_RGB32:
    case V4L2_PIX_FMT_BGR32:
    case V4L2_PIX_FMT_NV12: /* 12  Y/CbCr 4:2:0  */
    case V4L2_PIX_FMT_NV21: /* 12  Y/CrCb 4:2:0  */
    case V4L2_PIX_FMT_YVU410:
    case V4L2_PIX_FMT_YUV410:
    case V4L2_PIX_FMT_YUV420: /* I420/IYUV */
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_YVU420:
    case V4L2_PIX_FMT_UYVY:
#if 0
    case V4L2_PIX_FMT_Y41P:
#endif
    case V4L2_PIX_FMT_YUV422P:
#ifdef V4L2_PIX_FMT_YVYU
    case V4L2_PIX_FMT_YVYU:
#endif
    case V4L2_PIX_FMT_YUV411P: {
      GstVideoFormat format;

      switch (fourcc) {
        case V4L2_PIX_FMT_GREY: /*  8  Greyscale     */
          format = GST_VIDEO_FORMAT_GRAY8;
          break;
        case V4L2_PIX_FMT_RGB555:
          format = GST_VIDEO_FORMAT_RGB15;
          break;
        case V4L2_PIX_FMT_RGB565:
          format = GST_VIDEO_FORMAT_RGB16;
          break;
        case V4L2_PIX_FMT_RGB24:
          format = GST_VIDEO_FORMAT_RGB;
          break;
        case V4L2_PIX_FMT_BGR24:
          format = GST_VIDEO_FORMAT_BGR;
          break;
        case V4L2_PIX_FMT_RGB32:
          format = GST_VIDEO_FORMAT_RGBx;
          break;
        case V4L2_PIX_FMT_BGR32:
          format = GST_VIDEO_FORMAT_BGRx;
          break;
        case V4L2_PIX_FMT_NV12:
          format = GST_VIDEO_FORMAT_NV12;
          break;
        case V4L2_PIX_FMT_NV21:
          format = GST_VIDEO_FORMAT_NV21;
          break;
        case V4L2_PIX_FMT_YVU410:
          format = GST_VIDEO_FORMAT_YVU9;
          break;
        case V4L2_PIX_FMT_YUV410:
          format = GST_VIDEO_FORMAT_YUV9;
          break;
        case V4L2_PIX_FMT_YUV420:
          format = GST_VIDEO_FORMAT_I420;
          break;
        case V4L2_PIX_FMT_YUYV:
          format = GST_VIDEO_FORMAT_YUY2;
          break;
        case V4L2_PIX_FMT_YVU420:
          format = GST_VIDEO_FORMAT_YV12;
          break;
        case V4L2_PIX_FMT_UYVY:
          format = GST_VIDEO_FORMAT_UYVY;
          break;
#if 0
        case V4L2_PIX_FMT_Y41P:
          format = GST_VIDEO_FORMAT_Y41P;
          break;
#endif
        case V4L2_PIX_FMT_YUV411P:
          format = GST_VIDEO_FORMAT_Y41B;
          break;
        case V4L2_PIX_FMT_YUV422P:
          format = GST_VIDEO_FORMAT_Y42B;
          break;
#ifdef V4L2_PIX_FMT_YVYU
        case V4L2_PIX_FMT_YVYU:
          format = GST_VIDEO_FORMAT_YVYU;
          break;
#endif
        default:
          format = GST_VIDEO_FORMAT_UNKNOWN;
          g_assert_not_reached();
          break;
      }
      structure = gst_structure_new(
          "video/x-raw", "format", G_TYPE_STRING, gst_video_format_to_string(format), NULL);
      break;
    }
    case V4L2_PIX_FMT_DV:
      structure = gst_structure_new("video/x-dv", "systemstream", G_TYPE_BOOLEAN, TRUE, NULL);
      break;
    case V4L2_PIX_FMT_MPEG: /* MPEG          */
      structure = gst_structure_new("video/mpegts", "systemstream", G_TYPE_BOOLEAN, TRUE, NULL);
      break;
    case V4L2_PIX_FMT_WNVA: /* Winnov hw compres */
      break;
#ifdef V4L2_PIX_FMT_SBGGR8
    case V4L2_PIX_FMT_SBGGR8:
      structure = gst_structure_new_empty("video/x-bayer");
      break;
#endif
#ifdef V4L2_PIX_FMT_SN9C10X
    case V4L2_PIX_FMT_SN9C10X:
      structure = gst_structure_new_empty("video/x-sonix");
      break;
#endif
#ifdef V4L2_PIX_FMT_PWC1
    case V4L2_PIX_FMT_PWC1:
      structure = gst_structure_new_empty("video/x-pwc1");
      break;
#endif
#ifdef V4L2_PIX_FMT_PWC2
    case V4L2_PIX_FMT_PWC2:
      structure = gst_structure_new_empty("video/x-pwc2");
      break;
#endif
    default:
      GST_DEBUG("Unknown fourcc 0x%08x %" GST_FOURCC_FORMAT, fourcc, GST_FOURCC_ARGS(fourcc));
      break;
  }
  return structure;
}

void V4L2Src::on_gst_error(GstObject*, GError* err) {
  error("v4l2 camera error: %", std::string(err->message));
  self_destruct();
}

InfoTree::ptr V4L2Src::on_saving() {
  auto res = InfoTree::make();
  CaptureDescription& desc = capture_devices_[devices_enum_.get_current_index()];
  res->graft(".device_id", InfoTree::make(desc.device_id_));
  res->graft(".bus_id", InfoTree::make(desc.bus_info_));
  std::string save_mode = save_device_enum_.get_current_index() == 0 ? "port" : "device";
  res->graft(".save_by", InfoTree::make(save_mode));
  return res;
}

void V4L2Src::on_loading(InfoTree::ptr&& tree) {
  if (!tree || tree->empty()) {
    warning("loading deprecated v4l2src device save: devices may swap");
    return;
  }
  std::string device_id = tree->branch_read_data<std::string>(".device_id");
  std::string bus_id = tree->branch_read_data<std::string>(".bus_id");
  std::string save_mode = tree->branch_read_data<std::string>(".save_by");

  if (save_mode == "port") {
    auto it = std::find_if(
        capture_devices_.begin(), capture_devices_.end(), [&](const CaptureDescription& capt) {
          return capt.bus_info_ == bus_id;
        });
    if (capture_devices_.end() == it)
      warning("device not found at port %", bus_id);
    else
      pmanage<MPtr(&property::PBag::set<quiddity::property::IndexOrName>)>(
          devices_id_, quiddity::property::IndexOrName(it - capture_devices_.begin()));
  } else {  // save by device
    auto it = std::find_if(
        capture_devices_.begin(), capture_devices_.end(), [&](const CaptureDescription& capt) {
          return capt.device_id_ == device_id;
        });
    if (capture_devices_.end() == it)
      warning("device % not found", device_id);
    else
      pmanage<MPtr(&property::PBag::set<quiddity::property::IndexOrName>)>(
          devices_id_, quiddity::property::IndexOrName(it - capture_devices_.begin()));
  }
  // this is locking device change from property:
  is_loading_ = true;
}

void V4L2Src::on_loaded() { is_loading_ = false; }

}  // namespace quiddities
}  // namespace switcher
