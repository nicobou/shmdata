/*
 * This file is part of switcher-glfwin.
 *
 * switcher-glfwin is free software; you can redistribute it and/or
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

#include "./glfwvideo.hpp"
#include <gst/gst.h>
#include <cstdlib>
#include "./glfw-renderer.hpp"
#include "switcher/gprop-to-prop.hpp"
#include "switcher/scope-exit.hpp"
#include "switcher/shmdata-utils.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "./stb_image.h"
#ifdef HAVE_CONFIG_H
#include "../../config.h"
#endif

namespace switcher {
SWITCHER_DECLARE_PLUGIN(GLFWVideo);
SWITCHER_MAKE_QUIDDITY_DOCUMENTATION(GLFWVideo,
                                     "glfwin",
                                     "OpenGL Video Display (configurable)",
                                     "video",
                                     "reader/device/occasional-writer",
                                     "Video window with fullscreen",
                                     "LGPL",
                                     "Jérémie Soria");

const char* GLFWVideo::kVertexSource = R"(
  #version 330

  smooth out vec2 UV;

  uniform int geometry = 0;
  uniform int rotation;
  uniform int flip;

  const vec2 vertices[4] = vec2[]
  (
    vec2(-1.0, 1.0),
    vec2(1.0, 1.0),
    vec2(-1.0, -1.0),
    vec2(1.0, -1.0)
  );

  const vec2 uvs[4] = vec2[]
  (
    vec2(0.0, 0.0),
    vec2(1.0, 0.0),
    vec2(0.0, 1.0),
    vec2(1.0, 1.0)
  );

  void main()
  {
    UV = uvs[gl_VertexID];

    if (geometry == 1) {
      if (flip == 1)
        UV = vec2(1.f - UV.s, UV.t);
      else if (flip == 2)
        UV = vec2(UV.s, 1.f - UV.t);
      else if (flip == 3)
        UV = vec2(1.f - UV.s, 1.f - UV.t);

      if (rotation == 1)
        UV = vec2(UV.t, 1.f - UV.s);
      else if (rotation == 2)
        UV = vec2(1.f - UV.t, UV.s);
      else if (rotation == 3)
        UV = vec2(1.f - UV.s, 1.f - UV.t);
    }

    gl_Position = vec4(vertices[gl_VertexID], 0.f, 1.f);
  }
 )";

const char* GLFWVideo::kFragmentSource = R"(
  #version 330

  in vec2 UV;

  out vec4 out_color;

  uniform sampler2D tex;

  void main()
  {
    out_color = texture(tex, UV);
  }
 )";

const std::string GLFWVideo::kBackgroundColorDisabled =
    "Cannot modify color when background image mode is selected.";
const std::string GLFWVideo::kBackgroundImageDisabled =
    "Cannot modify background image when color mode is selected.";
const std::string GLFWVideo::kFullscreenDisabled =
    "This property is disabled while in fullscreen mode.";
const std::string GLFWVideo::kBackgroundTypeImage = "Image";
const std::string GLFWVideo::kBackgroundTypeColor = "Color";

std::atomic<int> GLFWVideo::instance_counter_(0);

GLFWVideo::GLFWVideo(const std::string& name)
    : shmcntr_(static_cast<Quiddity*>(this)),
      gst_pipeline_(std::make_unique<GstPipeliner>(nullptr, nullptr)),
      title_(name),
      title_id_(pmanage<MPtr(&PContainer::make_string)>("title",
                                                        [this](const std::string& val) {
                                                          title_ = val;
                                                          glfwSetWindowTitle(window_,
                                                                             title_.c_str());
                                                          return true;
                                                        },
                                                        [this]() { return title_; },
                                                        "Window Title",
                                                        "Window Title",
                                                        title_)),
      xevents_to_shmdata_id_(pmanage<MPtr(&PContainer::make_bool)>(
          "xevents",
          [this](bool val) {
            xevents_to_shmdata_ = val;
            if (xevents_to_shmdata_) {
              keyb_shm_ = std::make_unique<ShmdataWriter>(
                  this, make_file_name("keyb"), sizeof(KeybEvent), "application/x-keyboard-events");
              if (!keyb_shm_.get()) {
                g_warning("GLFW keyboard event shmdata writer failed");
                keyb_shm_.reset(nullptr);
              }

              mouse_shm_ = std::make_unique<ShmdataWriter>(
                  this, make_file_name("mouse"), sizeof(MouseEvent), "application/x-mouse-events");
              if (!mouse_shm_.get()) {
                g_warning("GLFW mouse event shmdata writer failed");
                mouse_shm_.reset(nullptr);
              }
            } else {
              mouse_shm_.reset(nullptr);
              keyb_shm_.reset(nullptr);
            }
            return true;
          },
          [this]() { return xevents_to_shmdata_; },
          "Keyboard/Mouse Events",
          "Capture Keyboard/Mouse Events",
          xevents_to_shmdata_)),
      fullscreen_id_(pmanage<MPtr(&PContainer::make_bool)>(
          "fullscreen",
          [this](bool val) {
            fullscreen_ = val;
            if (val) {
              minimized_width_ = width_;
              minimized_height_ = height_;
              minimized_position_x_ = position_x_;
              minimized_position_y_ = position_y_;
              discover_monitor_properties();
              auto monitor_config = get_monitor_config();
              if (!monitor_config.monitor) {
                g_warning(
                    "Could not get the monitor config for this window, not switching to "
                    "fullscreen.");
                return false;
              }
              pmanage<MPtr(&PContainer::disable)>(width_id_, kFullscreenDisabled);
              pmanage<MPtr(&PContainer::disable)>(height_id_, kFullscreenDisabled);
              pmanage<MPtr(&PContainer::disable)>(position_x_id_, kFullscreenDisabled);
              pmanage<MPtr(&PContainer::disable)>(position_y_id_, kFullscreenDisabled);
              width_ = monitor_config.width;
              height_ = monitor_config.height;
              position_x_ = monitor_config.position_x;
              position_y_ = monitor_config.position_y;
            } else {
              width_ = minimized_width_;
              height_ = minimized_height_;
              position_x_ = minimized_position_x_;
              position_y_ = minimized_position_y_;
              pmanage<MPtr(&PContainer::enable)>(width_id_);
              pmanage<MPtr(&PContainer::enable)>(height_id_);
              pmanage<MPtr(&PContainer::enable)>(position_x_id_);
              pmanage<MPtr(&PContainer::enable)>(position_y_id_);
            }
            set_size();
            set_position();
            return true;
          },
          [this]() { return fullscreen_; },
          "Window Fullscreen",
          "Toggle fullscreen on the window in its current monitor",
          false)),
      decorated_id_(pmanage<MPtr(&PContainer::make_bool)>(
          "decorated",
          [this](bool val) {
            decorated_ = val;
            add_rendering_task([this]() {
              swap_window();
              return false;  // Always start over the rendering tasks unstacking.
            });
            return true;
          },
          [this]() { return decorated_; },
          "Window Decoration",
          "Show/Hide Window Decoration",
          true)),
      always_on_top_id_(pmanage<MPtr(&PContainer::make_bool)>(
          "always_on_top",
          [this](bool val) {
            always_on_top_ = val;
            add_rendering_task([this]() {
              swap_window();
              return false;  // Always start over the rendering tasks unstacking.
            });
            return true;
          },
          [this]() { return always_on_top_; },
          "Always On Top",
          "Toggle Window Always On Top",
          true)),
      background_config_id_(pmanage<MPtr(&PContainer::make_group)>(
          "background_config",
          "Background configuration",
          "Select if you want a color or an image background when no video is playing.")),
      background_type_id_(pmanage<MPtr(&PContainer::make_parented_selection<>)>(
          "background_type",
          "background_config",
          [this](size_t val) {
            background_type_.select(val);
            if (background_type_.get_current() == kBackgroundTypeImage) {
              pmanage<MPtr(&PContainer::disable)>(color_id_, kBackgroundColorDisabled);
              pmanage<MPtr(&PContainer::enable)>(background_image_id_);
              draw_image_ = true;
              add_rendering_task([this]() {
                draw_data_ = image_data_;
                return true;
              });
            } else {
              pmanage<MPtr(&PContainer::disable)>(background_image_id_, kBackgroundImageDisabled);
              pmanage<MPtr(&PContainer::enable)>(color_id_);
              draw_image_ = false;
            }
            return true;
          },
          [this]() { return background_type_.get(); },
          "Background type",
          "Use a color or image background. Default: color.",
          background_type_)),
      color_(0, 0, 0, 0xFF),
      color_id_(pmanage<MPtr(&PContainer::make_parented_color)>(
          "color",
          "background_config",
          [this](const Color& val) {
            color_ = val;
            add_rendering_task([this]() {
              set_color();
              return true;
            });
            return true;
          },
          [this]() { return color_; },
          "Background color",
          "Color of the background when no video is displayed.",
          color_)),
      background_image_id_(pmanage<MPtr(&PContainer::make_parented_string)>(
          "background_image",
          "background_config",
          [this](const std::string& val) {
            background_image_ = val;
            if (background_image_.empty()) return true;
            add_rendering_task([this]() {
              int w = 0, h = 0, n = 0;
              auto data = stbi_load(background_image_.c_str(), &w, &h, &n, 4);
              On_scope_exit { stbi_image_free(data); };

              if (!data) {
                g_warning("Failed to load image at path %s.", background_image_.c_str());
                return true;
              }

              if (n < 3) {
                g_warning("Source background image is neither RGB nor RGBA (glfw).");
                return true;
              }

              size_t data_size = w * h * n;
              image_width_ = w;
              image_height_ = h;
              image_components_ = n;
              image_data_.resize(data_size);
              std::copy(data, data + data_size, image_data_.data());
              if (!draw_video_) setup_background_texture();
              return true;
            });
            return true;
          },
          [this]() { return background_image_; },
          "Background image file",
          "Path to the image to use as background when no video is played.",
          std::string())),
      rotation_id_(
          pmanage<MPtr(&PContainer::make_selection<>)>("rotation",
                                                       [this](size_t val) {
                                                         rotation_.select(val);
                                                         add_rendering_task([this, val]() {
                                                           set_viewport();
                                                           set_rotation_shader();
                                                           return true;
                                                         });
                                                         return true;
                                                       },
                                                       [this]() { return rotation_.get(); },
                                                       "Rotation modes",
                                                       "Possible rotation modes of the video.",
                                                       rotation_)),
      flip_id_(pmanage<MPtr(&PContainer::make_selection<>)>("flip",
                                                            [this](size_t val) {
                                                              flip_.select(val);
                                                              add_rendering_task([this, val]() {
                                                                set_flip_shader();
                                                                return true;
                                                              });
                                                              return true;
                                                            },
                                                            [this]() { return flip_.get(); },
                                                            "Flip modes",
                                                            "Possible flip modes of the video.",
                                                            flip_)),
      vsync_id_(pmanage<MPtr(&PContainer::make_selection<int>)>(
          "vsync",
          [this](size_t val) {
            vsync_.select(val);
            add_rendering_task([this]() {
              glfwSwapInterval(vsync_.get_attached());
              return true;
            });
            return true;
          },
          [this]() { return vsync_.get(); },
          "Vertical synchronization type",
          "Select the vertical synchronization mode (Hard/Soft/None).",
          vsync_)),
      geometry_task_(std::make_unique<PeriodicTask>(
          [this]() {
            if (!window_moved_) return;
            pmanage<MPtr(&PContainer::notify)>(position_x_id_);
            pmanage<MPtr(&PContainer::notify)>(position_y_id_);
            pmanage<MPtr(&PContainer::notify)>(width_id_);
            pmanage<MPtr(&PContainer::notify)>(height_id_);
            add_rendering_task([this]() {
              set_viewport();
              return true;
            });

            window_moved_ = false;
          },
          std::chrono::milliseconds(500))) {
  if (getenv("DISPLAY") == nullptr) {
    if (-1 == setenv("DISPLAY", ":0", false)) {
      g_warning("BUG: Failed to set a display!");
      return;
    }
  }

  if (!instance_counter_ && !glfwInit()) {
    g_warning("BUG: Failed to initialize glfw library!");
    return;
  }

  discover_monitor_properties();

  if (!monitors_config_.size()) {
    g_warning("BUG: Failed to discover monitors.");
    return;
  }

  width_id_ = pmanage<MPtr(&PContainer::make_int)>("width",
                                                   [this](const int& val) {
                                                     width_ = val;
                                                     set_size();
                                                     return true;
                                                   },
                                                   [this]() { return width_; },
                                                   "Window Width",
                                                   "Set Window Width",
                                                   800,
                                                   1,
                                                   max_width_);
  height_id_ = pmanage<MPtr(&PContainer::make_int)>("height",
                                                    [this](const int& val) {
                                                      height_ = val;
                                                      set_size();
                                                      return true;
                                                    },
                                                    [this]() { return height_; },
                                                    "Window Height",
                                                    "Set Window Height",
                                                    600,
                                                    1,
                                                    max_height_);
  position_x_id_ = pmanage<MPtr(&PContainer::make_int)>("position_x",
                                                        [this](const int& val) {
                                                          position_x_ = val;
                                                          set_position();
                                                          return true;
                                                        },
                                                        [this]() { return position_x_; },
                                                        "Window Position X",
                                                        "Set Window Horizontal Position",
                                                        0,
                                                        0,
                                                        max_width_);
  position_y_id_ = pmanage<MPtr(&PContainer::make_int)>("position_y",
                                                        [this](const int& val) {
                                                          position_y_ = val;
                                                          set_position();
                                                          return true;
                                                        },
                                                        [this]() { return position_y_; },
                                                        "Window Position Y",
                                                        "Set Window Vertical Position",
                                                        0,
                                                        0,
                                                        max_height_);

  shmcntr_.install_connect_method(
      [this](const std::string& shmpath) { return on_shmdata_connect(shmpath); },
      [this](const std::string&) { return on_shmdata_disconnect(); },
      [this]() { return on_shmdata_disconnect(); },
      [this](const std::string& caps) { return can_sink_caps(caps); },
      1);

  pmanage<MPtr(&PContainer::make_bool)>("keyb_interaction",
                                        [this](const bool& val) {
                                          keyb_interaction_ = val;
                                          return true;
                                        },
                                        [this]() { return keyb_interaction_; },
                                        "Keyboard Shortcuts",
                                        "Enable/Disable keybord shortcuts",
                                        keyb_interaction_);

  if (!remake_elements()) return;

  glfwWindowHint(GLFW_VISIBLE, false);
  glfwWindowHint(GLFW_RESIZABLE, true);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#if HAVE_OSX
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = create_window();
  if (!window_) return;

  glfwMakeContextCurrent(window_);
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    g_warning("BUG: glad could not load openGL functionalities.");
    return;
  }

  g_debug("OpenGL Version %d.%d loaded", GLVersion.major, GLVersion.minor);

  if (!setup_shaders()) return;
  glGenTextures(1, &drawing_texture_);
  setup_vertex_array();

  glfwMakeContextCurrent(nullptr);

  pmanage<MPtr(&PContainer::set_to_current)>(color_id_);
  pmanage<MPtr(&PContainer::set_to_current)>(background_type_id_);

  RendererSingleton::get()->subscribe_to_render_loop(this);
  ++instance_counter_;

  is_valid_ = true;
}

bool GLFWVideo::init() {
  // We need the quiddity config to be loaded so we cannot do it in the constructor.
  if (is_valid_) {
    load_icon();
    setup_icon();
  }

  return is_valid_;
}

GLFWVideo::~GLFWVideo() {
  // Blocking call until the window is unsubscribed from the render loop.
  ongoing_destruction_ = true;
  RendererSingleton::get()->unsubscribe_from_render_loop(this);

  destroy_gl_elements();
  if (window_) glfwDestroyWindow(window_);

  --instance_counter_;
  if (!instance_counter_) {
    glfwTerminate();
  }
}

void GLFWVideo::destroy_gl_elements() {
  glfwMakeContextCurrent(window_);
  glDeleteTextures(1, &drawing_texture_);
  glDeleteProgram(shader_program_);
  glDeleteShader(fragment_shader_);
  glDeleteShader(vertex_shader_);
  glDeleteVertexArrays(1, &vao_);
  glfwMakeContextCurrent(nullptr);
}

GLFWwindow* GLFWVideo::create_window(GLFWwindow* old) {
  glfwWindowHint(GLFW_DECORATED, decorated_);
  glfwWindowHint(GLFW_FLOATING, always_on_top_);

  auto window = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, old);
  if (!window) {
    g_warning("Could not create glfw window, probably an OpenGL version mismatch.");
    return nullptr;
  }

  set_events_cb(window);

  //  Always create in (0, 0).
  glfwSetWindowPos(window, 0, 0);
  glfwShowWindow(window);

  glfwMakeContextCurrent(window);

  glfwSwapInterval(1);

  glfwMakeContextCurrent(nullptr);

  return window;
}

void GLFWVideo::swap_window() {
  glfwMakeContextCurrent(nullptr);

  auto new_window = create_window(window_);
  if (!new_window) return;

  glfwDestroyWindow(window_);
  window_ = new_window;

  setup_icon();

  glfwMakeContextCurrent(window_);
  setup_vertex_array();
  glUseProgram(shader_program_);

  // Needed after the re-creation of the context.
  set_geometry();
}

inline void GLFWVideo::add_rendering_task(std::function<bool()> task) {
  if (!ongoing_destruction_) RendererSingleton::get()->add_rendering_task(this, task);
}

bool GLFWVideo::setup_shaders() {
  GLint status;
  shader_program_ = glCreateProgram();

  vertex_shader_ = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex_shader_, 1, &kVertexSource, NULL);
  glCompileShader(vertex_shader_);
  glGetShaderiv(vertex_shader_, GL_COMPILE_STATUS, &status);
  if (status != GL_TRUE) {
    char buffer[512];
    glGetShaderInfoLog(vertex_shader_, 512, NULL, buffer);
    g_warning("Failed to compile vertex shader (glfwin): %s.", buffer);
    return false;
  }

  fragment_shader_ = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment_shader_, 1, &kFragmentSource, NULL);
  glCompileShader(fragment_shader_);
  glGetShaderiv(fragment_shader_, GL_COMPILE_STATUS, &status);
  if (status != GL_TRUE) {
    char buffer[512];
    glGetShaderInfoLog(fragment_shader_, 512, NULL, buffer);
    g_warning("Failed to compile fragment shader (glfwin): %s.", buffer);
    return false;
  }

  glAttachShader(shader_program_, vertex_shader_);
  glAttachShader(shader_program_, fragment_shader_);
  glBindFragDataLocation(shader_program_, 0, "out_color");
  glLinkProgram(shader_program_);
  glUseProgram(shader_program_);

  return true;
}

bool GLFWVideo::setup_vertex_array() {
  if (vao_) glDeleteVertexArrays(1, &vao_);
  glGenVertexArrays(1, &vao_);
  glBindVertexArray(vao_);
  return true;
}

void GLFWVideo::setup_video_texture() {
  glBindTexture(GL_TEXTURE_2D, drawing_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D,
               0,
               GL_RGBA,
               vid_width_,
               vid_height_,
               0,
               GL_RGBA,
               GL_UNSIGNED_INT_8_8_8_8_REV,
               0);
  glBindTexture(GL_TEXTURE_2D, 0);
  set_viewport();
}

void GLFWVideo::setup_background_texture() {
  glBindTexture(GL_TEXTURE_2D, drawing_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexImage2D(GL_TEXTURE_2D,
               0,
               GL_RGBA,
               image_width_,
               image_height_,
               0,
               image_components_ == 4 ? GL_RGBA : GL_RGB,
               image_components_ == 4 ? GL_UNSIGNED_INT_8_8_8_8_REV : GL_UNSIGNED_BYTE,
               image_data_.data());
  glGenerateMipmap(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, 0);
  set_viewport();
}

void GLFWVideo::load_icon() {
  icon_.pixels = nullptr;
  if (!config<MPtr(&InfoTree::branch_has_data)>("icon")) {
    g_warning("Icons configuration is missing (glfw).");
    return;
  }
  auto icon_config = config<MPtr(&InfoTree::branch_get_value)>("icon").copy_as<std::string>();

  int w = 0, h = 0, n = 0;
  auto icon_content = stbi_load(icon_config.c_str(), &w, &h, &n, 4);
  if (n != 4) return;  // Only accept 32-bit formats for the icon.
  On_scope_exit { stbi_image_free(icon_content); };
  if (!icon_content) return;
  int data_size = w * h * n;
  icon_data_.resize(data_size);
  std::copy(icon_content, icon_content + data_size, icon_data_.data());
  icon_.width = w;
  icon_.height = h;
  icon_.pixels = icon_data_.data();
}

void GLFWVideo::setup_icon() {
  if (!icon_.pixels) return;
  glfwSetWindowIcon(window_, 1, &icon_);
}

void GLFWVideo::set_events_cb(GLFWwindow* window) {
  glfwSetWindowUserPointer(window, this);  // Will be used by events callbacks.

  glfwSetWindowCloseCallback(window, close_cb);
  glfwSetWindowPosCallback(window, move_cb);
  glfwSetWindowSizeCallback(window, resize_cb);
  glfwSetWindowFocusCallback(window, focus_cb);
  glfwSetKeyCallback(window, key_cb);
  glfwSetCursorPosCallback(window, mouse_cb);
  glfwSetCursorEnterCallback(window, enter_cb);
}

void GLFWVideo::close_cb(GLFWwindow* window) {
  auto quiddity = static_cast<GLFWVideo*>(glfwGetWindowUserPointer(window));
  quiddity->self_destruct();
}

void GLFWVideo::move_cb(GLFWwindow* window, int pos_x, int pos_y) {
  auto quiddity = static_cast<GLFWVideo*>(glfwGetWindowUserPointer(window));
  quiddity->position_x_ = pos_x;
  quiddity->position_y_ = pos_y;
  quiddity->window_moved_ = true;
}

void GLFWVideo::resize_cb(GLFWwindow* window, int width, int height) {
  auto quiddity = static_cast<GLFWVideo*>(glfwGetWindowUserPointer(window));
  quiddity->width_ = width;
  quiddity->height_ = height;
  quiddity->window_moved_ = true;
}

void GLFWVideo::focus_cb(GLFWwindow* window, int focused) {
  auto quiddity = static_cast<GLFWVideo*>(glfwGetWindowUserPointer(window));
  if (focused)
    quiddity->graft_tree(".focused.", InfoTree::make("true"));
  else
    quiddity->graft_tree(".focused.", InfoTree::make("false"));
}

void GLFWVideo::key_cb(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
  auto quiddity = static_cast<GLFWVideo*>(glfwGetWindowUserPointer(window));

  if (quiddity->keyb_shm_.get()) {
    guint32 val = key;
    auto keybevent = KeybEvent(val, action);
    quiddity->keyb_shm_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(&keybevent, sizeof(KeybEvent));
    quiddity->keyb_shm_->bytes_written(sizeof(KeybEvent));
  }

  if (quiddity->keyb_interaction_) {
    if (action == GLFW_RELEASE) return;  // Only process the key presses.
    switch (key) {
      case GLFW_KEY_F:
      case GLFW_KEY_ESCAPE:
        quiddity->pmanage<MPtr(&PContainer::set<bool>)>(
            quiddity->fullscreen_id_,
            !quiddity->fullscreen_);  // toggle fullscreen
        break;
      case GLFW_KEY_D:
        quiddity->pmanage<MPtr(&PContainer::set<bool>)>(
            quiddity->decorated_id_,
            !quiddity->decorated_);  // toggle decoration
        break;
      case GLFW_KEY_T:
        quiddity->pmanage<MPtr(&PContainer::set<bool>)>(
            quiddity->always_on_top_id_,
            !quiddity->always_on_top_);  // toggle always on top status
        break;
      default:
        break;
    }
  }
}

void GLFWVideo::mouse_cb(GLFWwindow* window, double xpos, double ypos) {
  auto quiddity = static_cast<GLFWVideo*>(glfwGetWindowUserPointer(window));

  if (!quiddity->xevents_to_shmdata_) return;
  if (!quiddity->mouse_shm_.get()) return;
  if (!quiddity->cursor_inside_) return;

  auto mouse_event = MouseEvent(xpos, ypos, 1);
  quiddity->mouse_shm_->writer<MPtr(&shmdata::Writer::copy_to_shm)>(&mouse_event,
                                                                    sizeof(MouseEvent));
  quiddity->mouse_shm_->bytes_written(sizeof(MouseEvent));
}

void GLFWVideo::enter_cb(GLFWwindow* window, int entered) {
  auto quiddity = static_cast<GLFWVideo*>(glfwGetWindowUserPointer(window));
  quiddity->cursor_inside_ = entered;
}

void GLFWVideo::set_viewport() {
  On_scope_exit { glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); };

  // Resets the viewport if no video is playing.
  if (!draw_video_) {
    glViewport(0, 0, width_, height_);
    return;
  }

  float window_ratio = static_cast<float>(width_) / static_cast<float>(height_);
  float image_ratio = static_cast<float>(vid_width_) / static_cast<float>(vid_height_);

  // Inverts the ratio if rotated 90-degrees.
  if (rotation_.get() == 1 || rotation_.get() == 2) image_ratio = 1.f / image_ratio;

  float padding_x = 0.f;
  float padding_y = 0.f;

  if (image_ratio > window_ratio) {
    padding_y = (1.f - window_ratio / image_ratio) * height_ / 2.f;
    glViewport(padding_x, padding_y, width_, height_ * (window_ratio / image_ratio));
  } else {
    padding_x = (1.f - image_ratio / window_ratio) * width_ / 2.f;
    glViewport(padding_x, padding_y, width_ * image_ratio / window_ratio, height_);
  }
}

inline void GLFWVideo::set_color() {
  glClearColor(static_cast<GLfloat>(color_.red()) / 255.0f,
               static_cast<GLfloat>(color_.green()) / 255.0f,
               static_cast<GLfloat>(color_.blue()) / 255.0f,
               static_cast<GLfloat>(color_.alpha()) / 255.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

inline void GLFWVideo::set_rotation_shader() {
  glUniform1i(glGetUniformLocation(shader_program_, "rotation"), rotation_.get());
}

inline void GLFWVideo::set_flip_shader() {
  glUniform1i(glGetUniformLocation(shader_program_, "flip"), flip_.get());
}

inline void GLFWVideo::enable_geometry() {
  glUniform1i(glGetUniformLocation(shader_program_, "geometry"), 1);
}

inline void GLFWVideo::disable_geometry() {
  glUniform1i(glGetUniformLocation(shader_program_, "geometry"), 0);
}

inline void GLFWVideo::set_position() { glfwSetWindowPos(window_, position_x_, position_y_); }

inline void GLFWVideo::set_size() { glfwSetWindowSize(window_, width_, height_); }

inline void GLFWVideo::set_geometry() {
  set_position();
  set_color();
  set_viewport();
  set_rotation_shader();
  set_flip_shader();
}

void GLFWVideo::discover_monitor_properties() {
  int count;
  monitors_config_.clear();
  auto all_monitors = glfwGetMonitors(&count);
  if (!all_monitors) return;

  max_width_ = 0;
  max_height_ = 0;
  for (int i = 0; i < count; ++i) {
    auto mode = glfwGetVideoMode(all_monitors[i]);
    if (!mode) continue;
    MonitorConfig config;
    config.monitor = all_monitors[i];
    config.width = mode->width;
    config.height = mode->height;
    glfwGetMonitorPos(all_monitors[i], &config.position_x, &config.position_y);
    max_width_ = std::max(config.width + config.position_x, max_width_);
    max_height_ = std::max(config.height + config.position_y, max_height_);
    monitors_config_.push_back(config);
  }
}

const GLFWVideo::MonitorConfig GLFWVideo::get_monitor_config() const {
  auto config = std::find_if(
      monitors_config_.cbegin(), monitors_config_.cend(), [this](const MonitorConfig& conf) {
        // Check if the current window is within the bounds of a monitor.
        if (conf.position_x <= position_x_ && (conf.position_x + conf.width) > position_x_ &&
            conf.position_y <= position_y_ && (conf.position_y + conf.height) > position_y_) {
          return true;
        }
        return false;
      });
  if (config == monitors_config_.cend()) return MonitorConfig();
  return *config;
}

bool GLFWVideo::on_shmdata_connect(const std::string& shmpath) {
  shmpath_ = shmpath;
  g_object_set(G_OBJECT(shmsrc_.get_raw()), "socket-path", shmpath_.c_str(), nullptr);
  shm_sub_ = std::make_unique<GstShmdataSubscriber>(
      shmsrc_.get_raw(),
      [this](const std::string& caps) {
        graft_tree(".shmdata.reader." + shmpath_,
                   ShmdataUtils::make_tree(caps, ShmdataUtils::get_category(caps), ShmdataStat()));
        GstCaps* gstcaps = gst_caps_from_string(caps.c_str());
        On_scope_exit {
          if (gstcaps) gst_caps_unref(gstcaps);
        };
        GstStructure* caps_struct = gst_caps_get_structure(gstcaps, 0);
        const GValue* width_val = gst_structure_get_value(caps_struct, "width");
        const GValue* height_val = gst_structure_get_value(caps_struct, "height");
        vid_width_ = g_value_get_int(width_val);
        vid_height_ = g_value_get_int(height_val);

        auto frame_size = vid_width_ * vid_height_ * 4;  // Only RGBA is supported.
        video_frames_.resize(frame_size);

        add_rendering_task([this]() {
          draw_video_ = true;
          setup_video_texture();
          enable_geometry();
          return true;
        });
      },
      [this](const ShmdataStat&) {},
      [this]() {
        add_rendering_task([this]() {
          draw_video_ = false;
          setup_background_texture();
          disable_geometry();
          return true;
        });
      });

  shm_follower_ = std::make_unique<ShmdataFollower>(this,
                                                    shmpath_,
                                                    nullptr,
                                                    [this](const std::string& /*shmtype*/) {
                                                      add_rendering_task([this]() {
                                                        draw_video_ = true;
                                                        setup_video_texture();
                                                        enable_geometry();
                                                        return true;
                                                      });
                                                    },
                                                    [this]() {
                                                      add_rendering_task([this]() {
                                                        draw_video_ = false;
                                                        setup_background_texture();
                                                        disable_geometry();
                                                        return true;
                                                      });
                                                    });

  // Fakesink setup
  g_object_set(G_OBJECT(fakesink_.get_raw()),
               "silent",
               TRUE,
               "signal-handoffs",
               TRUE,
               "sync",
               FALSE,
               nullptr);
  g_signal_connect(G_OBJECT(fakesink_.get_raw()), "handoff", G_CALLBACK(on_handoff_cb), this);

  // Pipeline setup
  g_object_set(G_OBJECT(gst_pipeline_->get_pipeline()), "async-handling", TRUE, nullptr);

  // Capsfilter setup
  GstCaps* usercaps = gst_caps_from_string("video/x-raw,format=RGBA");
  g_object_set(G_OBJECT(capsfilter_.get_raw()), "caps", usercaps, nullptr);
  gst_caps_unref(usercaps);

  // Pipeline assembly
  gst_bin_add_many(GST_BIN(gst_pipeline_->get_pipeline()),
                   shmsrc_.get_raw(),
                   queue_.get_raw(),
                   videoconvert_.get_raw(),
                   capsfilter_.get_raw(),
                   gamma_.get_raw(),
                   videobalance_.get_raw(),
                   fakesink_.get_raw(),
                   nullptr);
  gst_element_link_many(shmsrc_.get_raw(),
                        queue_.get_raw(),
                        videoconvert_.get_raw(),
                        capsfilter_.get_raw(),
                        gamma_.get_raw(),
                        videobalance_.get_raw(),
                        fakesink_.get_raw(),
                        nullptr);

  install_gst_properties();
  gst_pipeline_->play(true);

  return true;
}

bool GLFWVideo::on_shmdata_disconnect() {
  prune_tree(".shmdata.reader." + shmpath_);
  shm_sub_.reset();
  shm_follower_.reset();
  On_scope_exit { gst_pipeline_ = std::make_unique<GstPipeliner>(nullptr, nullptr); };

  return remake_elements();
}

bool GLFWVideo::remake_elements() {
  remove_gst_properties();
  if (!UGstElem::renew(shmsrc_) || !UGstElem::renew(queue_) || !UGstElem::renew(videoconvert_) ||
      !UGstElem::renew(capsfilter_) || !UGstElem::renew(gamma_, {"gamma"}) ||
      !UGstElem::renew(videobalance_, {"contrast", "brightness", "hue", "saturation"}) ||
      !UGstElem::renew(fakesink_)) {
    g_error("glfwin could not renew GStreamer elements");
    return false;
  }
  install_gst_properties();
  return true;
}

void GLFWVideo::install_gst_properties() {
  pmanage<MPtr(&PContainer::push)>("gamma",
                                   GPropToProp::to_prop(G_OBJECT(gamma_.get_raw()), "gamma"));
  pmanage<MPtr(&PContainer::push)>(
      "contrast", GPropToProp::to_prop(G_OBJECT(videobalance_.get_raw()), "contrast"));
  pmanage<MPtr(&PContainer::push)>(
      "brightness", GPropToProp::to_prop(G_OBJECT(videobalance_.get_raw()), "brightness"));
  pmanage<MPtr(&PContainer::push)>("hue",
                                   GPropToProp::to_prop(G_OBJECT(videobalance_.get_raw()), "hue"));
  pmanage<MPtr(&PContainer::push)>(
      "saturation", GPropToProp::to_prop(G_OBJECT(videobalance_.get_raw()), "saturation"));
}

void GLFWVideo::remove_gst_properties() {
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("gamma"));
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("contrast"));
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("brightness"));
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("hue"));
  pmanage<MPtr(&PContainer::remove)>(pmanage<MPtr(&PContainer::get_id)>("saturation"));
}

inline void GLFWVideo::on_handoff_cb(GstElement* /*object*/,
                                     GstBuffer* buf,
                                     GstPad* /*pad*/,
                                     gpointer user_data) {
  GLFWVideo* context = static_cast<GLFWVideo*>(user_data);

  GstMapInfo map;

  if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
    g_warning("gst_buffer_map failed: canceling video buffer access");
    return;
  }
  On_scope_exit { gst_buffer_unmap(buf, &map); };

  context->video_frames_.write(map.data, map.data + map.size);
}

bool GLFWVideo::can_sink_caps(std::string caps) {
  return GstUtils::can_sink_caps("videoconvert", caps);
};

}  // namespace switcher
