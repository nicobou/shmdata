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

#ifndef __SWITCHER_GLFW_VIDEO_H__
#define __SWITCHER_GLFW_VIDEO_H__

// clang-format off
#include "./glad.h"
#include <GLFW/glfw3.h>
#include <imgui.h>
// clang-format on
#include "switcher/gst-pipeliner.hpp"
#include "switcher/gst-shmdata-subscriber.hpp"
#include "switcher/multiple-buffer.hpp"
#include "switcher/periodic-task.hpp"
#include "switcher/shmdata-connector.hpp"
#include "switcher/shmdata-follower.hpp"
#include "switcher/shmdata-writer.hpp"

namespace switcher {

//! GLFWVideo class, to display video in a resizable window.
class GLFWVideo : public Quiddity {
  friend class GLFWRenderer;

 public:

  /**
   * \brief Constructor
   * \param Title of the window
   */
  GLFWVideo(QuiddityConfiguration&&);
  /**
   * \brief Destructor
   */
  ~GLFWVideo();

  /**
   * \brief No copy, no move
   */
  GLFWVideo(const GLFWVideo&) = delete;
  GLFWVideo& operator=(const GLFWVideo&) = delete;

 private:
  /**
   * \brief Object used for keyboard events forwarding to shmdata
   */
  struct KeybEvent {
    KeybEvent(int keyval, int down) : keyval_(keyval), down_(down) {}
    int keyval_{0};
    int down_{0};
  };

  /**
   * \brief Object used for mouse events forwarding to shmdata
   */
  struct MouseEvent {
    MouseEvent(double x, double y, uint32_t button_mask)
        : x_(x), y_(y), button_mask_(button_mask) {}
    double x_{0};
    double y_{0};
    uint32_t button_mask_{0};
  };

  /**
   * \brief Object storing the global configuration for a given monitor
   */
  struct MonitorConfig {
    int width{0};
    int height{0};
    int position_x{0};
    int position_y{0};
    GLFWmonitor* monitor{nullptr};
  };

  struct GUIConfiguration {
    GUIConfiguration(GLFWVideo* window);
    ~GUIConfiguration();

    struct GUIContext {
      GUIContext() { ctx = ImGui::CreateContext(); };
      ~GUIContext() { ImGui::DestroyContext(ctx); };
      ImGuiContext* ctx{nullptr};
    };

    void init_imgui();
    void destroy_imgui();
    void init_properties();
    std::vector<std::string> get_fonts();
    bool generate_font_texture(std::string font);
    void show();
    static void render_overlay(ImDrawData* draw_data);

    enum Alignment : unsigned int {
      TOP_ALIGNED = 1,
      BOTTOM_ALIGNED = 1 << 1,
      VERTICAL_CENTER = 1 << 2,
      LEFT_ALIGNED = 1 << 3,
      RIGHT_ALIGNED = 1 << 4,
      HORIZONTAL_CENTER = 1 << 5
    };

    GLuint gui_vao_{0};
    GLuint gui_vbo_{0};
    size_t gui_vbo_max_size_{20000};
    GLuint gui_elements_{0};
    GLuint gui_shader_program_{0};
    GLuint gui_vertex_shader_{0};
    GLuint gui_fragment_shader_{0};
    GLuint gui_font_texture_id_{0};
    GLint gui_texture_location_{0};
    GLint gui_proj_matrix_location_{0};
    GLint gui_position_location_{0};
    GLint gui_uv_location_{0};
    GLint gui_color_location_{0};
    bool initialized_{false};

    GLFWVideo* parent_window_{nullptr};

    ImVec2 text_size_{};
    std::string text_{};
    PContainer::prop_id_t text_id_{0};
    Selection<unsigned int> alignment_{{"Top left corner",
                                        "Bottom left corner",
                                        "Top right corner",
                                        "Bottom right corner",
                                        "Center",
                                        "Top center",
                                        "Bottom center"},
                                       {Alignment::TOP_ALIGNED | Alignment::LEFT_ALIGNED,
                                        Alignment::BOTTOM_ALIGNED | Alignment::LEFT_ALIGNED,
                                        Alignment::TOP_ALIGNED | Alignment::RIGHT_ALIGNED,
                                        Alignment::BOTTOM_ALIGNED | Alignment::RIGHT_ALIGNED,
                                        Alignment::VERTICAL_CENTER | Alignment::HORIZONTAL_CENTER,
                                        Alignment::TOP_ALIGNED | Alignment::HORIZONTAL_CENTER,
                                        Alignment::BOTTOM_ALIGNED | Alignment::HORIZONTAL_CENTER},
                                       6};
    PContainer::prop_id_t alignment_id_{0};
    bool use_custom_font_{};
    PContainer::prop_id_t use_custom_font_id_{0};
    std::string custom_font_{};
    PContainer::prop_id_t custom_font_id_{0};
    std::vector<std::string> fonts_list_;
    Selection<> fonts_{{"none"}, 0};
    PContainer::prop_id_t font_id_{0};
    Color color_;
    PContainer::prop_id_t color_id_{0};
    unsigned int font_size_{20};
    PContainer::prop_id_t font_size_id_{0};
    std::unique_ptr<GUIContext> context_{nullptr};
    ImFontAtlas font_atlas_{};
  };

  /**
   * \brief Class constants
   */
  static const std::string kFullscreenDisabled;
  static const std::string kBackgroundColorDisabled;
  static const std::string kBackgroundImageDisabled;
  static const std::string kBackgroundTypeImage;
  static const std::string kBackgroundTypeColor;
  static const std::string kOverlayDisabledMessage;

  /**
   * \brief Quiddity lifecycle methods
   */
  void destroy_gl_elements();
  GLFWwindow* create_window(GLFWwindow* old = nullptr);
  void swap_window();

  /**
   * \brief OpenGL-related methods
   */
  void add_rendering_task(std::function<bool()> task);
  bool setup_shaders();
  bool setup_vertex_array();
  void setup_video_texture();
  void setup_background_texture();
  void load_icon();
  void setup_icon();

  /**
   * \brief GLFW events management
   */
  void set_events_cb(GLFWwindow* window);
  static void close_cb(GLFWwindow* window);
  static void move_cb(GLFWwindow* window, int pos_x, int pos_y);
  static void resize_cb(GLFWwindow* window, int width, int height);
  static void focus_cb(GLFWwindow* window, int focused);
  static void key_cb(GLFWwindow* window, int key, int scancode, int action, int mods);
  static void mouse_cb(GLFWwindow* window, double xpos, double ypos);
  static void enter_cb(GLFWwindow* window, int entered);

  /**
   * \brief Geometry methods, to be launched in appropriate OpenGL context
   */
  void set_viewport(bool clear = true);
  void set_color();
  void set_rotation_shader();
  void set_flip_shader();
  void enable_geometry();
  void disable_geometry();
  void set_position();
  void set_size();
  void set_geometry();

  /**
   * \brief Monitor management methods
   */
  void discover_monitor_properties();
  const MonitorConfig get_monitor_config() const;

  /**
   * \brief Data management methods
   */
  bool on_shmdata_connect(const std::string& shmpath);
  bool on_shmdata_disconnect();
  bool remake_elements();
  void install_gst_properties();
  void remove_gst_properties();
  static void on_handoff_cb(GstElement* object, GstBuffer* buf, GstPad* pad, gpointer user_data);
  bool can_sink_caps(std::string caps);

  /**
   * \brief Used for OpenGL rendering
   */
  std::atomic<bool> ongoing_destruction_{false};
  static std::atomic<int> instance_counter_;
  MultipleBuffer<3, uint8_t> video_frames_{};
  std::vector<uint8_t> image_data_{};
  GLuint drawing_texture_{0};
  GLuint shader_program_{0};
  GLuint vertex_shader_{0};
  GLuint fragment_shader_{0};
  GLuint vao_{0};
  std::atomic<bool> draw_video_{false};
  std::atomic<bool> draw_image_{false};

  /**
   * \brief Shmdata connection
   */
  ShmdataConnector shmcntr_;
  std::string shmpath_{};
  std::unique_ptr<ShmdataFollower> shm_follower_{nullptr};
  std::unique_ptr<GstShmdataSubscriber> shm_sub_{nullptr};

  /**
   * \brief Gstreamer elements and tools
   */
  std::unique_ptr<GstPipeliner> gst_pipeline_;
  UGstElem shmsrc_{"shmdatasrc"};
  UGstElem queue_{"queue"};
  UGstElem videoconvert_{"videoconvert"};
  UGstElem capsfilter_{"capsfilter"};
  UGstElem gamma_{"gamma"};
  UGstElem videobalance_{"videobalance"};
  UGstElem fakesink_{"fakesink"};

  /**
   * \brief Window content properties
   */
  GLFWwindow* window_{nullptr};
  bool cursor_inside_{false};
  std::vector<uint8_t> icon_data_;
  GLFWimage icon_;
  /**
   * \brief Background properties
   */
  PContainer::prop_id_t background_config_id_{0};
  Selection<> background_type_{{kBackgroundTypeColor, kBackgroundTypeImage}, 0};
  PContainer::prop_id_t background_type_id_{0};
  Color color_;
  PContainer::prop_id_t color_id_{0};
  std::string background_image_{};
  PContainer::prop_id_t background_image_id_{0};
  /**
   * \brief Overlay properties
   */
  std::mutex configuration_mutex_{};
  PContainer::prop_id_t overlay_id_{0};
  bool show_overlay_{false};
  PContainer::prop_id_t show_overlay_id_{0};
  std::unique_ptr<GUIConfiguration> gui_configuration_{nullptr};
  std::unique_ptr<PeriodicTask<>> geometry_task_;
  /**
   * \brief top level properties
   */
  std::string title_;
  PContainer::prop_id_t title_id_{0};
  gboolean keyb_interaction_{TRUE};
  bool xevents_to_shmdata_{false};
  PContainer::prop_id_t xevents_to_shmdata_id_{0};
  std::unique_ptr<ShmdataWriter> keyb_shm_{nullptr};
  std::unique_ptr<ShmdataWriter> mouse_shm_{nullptr};
  int vid_width_{0};
  int vid_height_{0};
  int image_width_{0};
  int image_height_{0};
  int image_components_{0};
  std::atomic<bool> window_moved_{false};

  /**
   * \brief Properties related to window geometry
   */
  std::vector<MonitorConfig> monitors_config_{};
  bool fullscreen_{false};
  PContainer::prop_id_t fullscreen_id_{0};
  bool decorated_{false};
  PContainer::prop_id_t decorated_id_{0};
  bool always_on_top_{false};
  PContainer::prop_id_t always_on_top_id_{0};
  bool win_aspect_ratio_toggle_{false};
  PContainer::prop_id_t win_aspect_ratio_toggle_id_{0};
  int width_{800};
  PContainer::prop_id_t width_id_{0};
  int height_{600};
  PContainer::prop_id_t height_id_{0};
  int position_x_{0};
  PContainer::prop_id_t position_x_id_{0};
  int position_y_{0};
  PContainer::prop_id_t position_y_id_{0};
  Selection<> rotation_{
      {"None", "90 degrees clockwise", "90 degrees anti-clockwise", "180 degrees"}, 0};
  PContainer::prop_id_t rotation_id_{0};
  Selection<> flip_{{"None", "Horizontal flip", "Vertical flip", "Bidirectional flip"}, 0};
  PContainer::prop_id_t flip_id_{0};
  Selection<int> vsync_{{"Hard", "Soft", "None"}, {1, -1, 0}, 0};
  PContainer::prop_id_t vsync_id_{0};
  int max_width_{0};
  int max_height_{0};
  int minimized_width_{800};
  int minimized_height_{600};
  int minimized_position_x_{0};
  int minimized_position_y_{0};
};

}  // namespace switcher
#endif
