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

#ifndef __SWITCHER_GLFW_RENDERER_H__
#define __SWITCHER_GLFW_RENDERER_H__

#include <future>
#include "./glfwvideo.hpp"

namespace switcher {

class GLFWRenderer {
 public:
  GLFWRenderer();
  ~GLFWRenderer();
  GLFWRenderer(const GLFWRenderer&) = delete;
  GLFWRenderer& operator=(const GLFWRenderer&) = delete;

  void subscribe_to_render_loop(GLFWVideo* window);
  void unsubscribe_from_render_loop(GLFWVideo* window);
  void add_rendering_task(GLFWVideo* window, std::function<bool()> task);

 private:
  void render_loop();
  void check_unsubscribe_from_render_loop();
  bool do_rendering_tasks(GLFWVideo* window);

  std::map<GLFWVideo*, std::vector<std::function<bool()>>> rendering_tasks_;
  std::vector<GLFWVideo*> unsubscribers_;
  std::mutex subscription_mutex_{};
  std::mutex rendering_task_mutex_{};
  std::condition_variable cond_subscription_{};
  std::atomic<bool> running_{true};
  std::future<void> gl_loop_;
};

class RendererSingleton {
 public:
  static GLFWRenderer* get();
  GLFWRenderer& operator=(const GLFWRenderer&) = delete;

 private:
  static std::unique_ptr<GLFWRenderer> s_instance_;
  static std::mutex creation_mutex_;
};
};

#endif
