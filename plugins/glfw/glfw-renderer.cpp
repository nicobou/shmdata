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

#include "./glfw-renderer.hpp"
#include "switcher/utils/scope-exit.hpp"

namespace switcher {

std::unique_ptr<GLFWRenderer> RendererSingleton::s_instance_;
std::mutex RendererSingleton::creation_mutex_;
std::mutex RendererSingleton::creation_window_mutex_;

GLFWRenderer::GLFWRenderer()
    : gl_loop_(std::async(std::launch::async, [this]() { render_loop(); })) {}

GLFWRenderer::~GLFWRenderer() { running_ = false; }

void GLFWRenderer::render_loop() {
  while (running_) {
    // Wait if there are no window.
    if (!GLFWVideo::instance_counter_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    int current_window = 0;

    {
      std::lock_guard<std::mutex> lock(RendererSingleton::creation_window_mutex_);

      for (auto& instance : pop_rendering_tasks()) {
        ++current_window;
        auto current = instance.first;
        glfwMakeContextCurrent(current->window_);
        On_scope_exit { glfwMakeContextCurrent(nullptr); };
        bool ret = true;
        for (auto& task : instance.second) {
          if (task && !task()) ret = false;
        }
        if (!ret) continue;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (current->draw_video_ || current->draw_image_) {
          glBindTexture(GL_TEXTURE_2D, current->drawing_texture_);

          if (current->draw_video_) {
            bool has_changed = true;
            auto data = current->video_frames_.read(has_changed);
            if (has_changed) {
              glTexSubImage2D(GL_TEXTURE_2D,
                              0,
                              0,
                              0,
                              current->vid_width_,
                              current->vid_height_,
                              GL_RGBA,
                              GL_UNSIGNED_INT_8_8_8_8_REV,
                              (const GLvoid*)data);
              glGenerateMipmap(GL_TEXTURE_2D);
            }
          }

          glBindVertexArray(current->vao_);
          glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        }

        // Draw the overlay
        if (current->show_overlay_) {
          // We MUST lock before checking the status of the ImGui configuration, otherwise undefined
          // behaviour is to be expected.
          std::lock_guard<std::mutex> lock(current->configuration_mutex_);
          if (current->gui_configuration_ && current->gui_configuration_->initialized_) {
            current_window_ = current;
            ImGui::SetCurrentContext(current->gui_configuration_->context_->ctx);
            ImGui::NewFrame();
            current->gui_configuration_->show();
            ImGui::Render();
          }
        }

        // Only swap the buffers of the first window, just for vertical synchronization.
        if (current_window == 1 || GLFWVideo::instance_counter_ == 1)
          glfwSwapBuffers(current->window_);
        else {
          glReadBuffer(GL_BACK);
          glDrawBuffer(GL_FRONT);
          glBlitFramebuffer(0,
                            0,
                            current->width_,
                            current->height_,
                            0,
                            0,
                            current->width_,
                            current->height_,
                            GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT,
                            GL_NEAREST);
          glDrawBuffer(GL_BACK);
        }
      }
    }
    glfwPollEvents();
  }
}

void GLFWRenderer::subscribe_to_render_loop(GLFWVideo* window) {
  add_rendering_task(window, []() {
    glDrawBuffer(GL_FRONT_AND_BACK);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawBuffer(GL_BACK);
    return true;
  });
}

void GLFWRenderer::unsubscribe_from_render_loop(GLFWVideo* window) {
  // Wait for the render loop to be done with the window before destroying it.
    std::lock_guard<std::mutex> lock(rendering_task_mutex_);
    auto renderer = rendering_tasks_.find(window);
    if (renderer != rendering_tasks_.end()) rendering_tasks_.erase(renderer);
}

GLFWRenderer::rendering_tasks_t GLFWRenderer::pop_rendering_tasks() {
  // construct copy for caller, and remove lambdas
  rendering_tasks_t ret;
  {
    std::lock_guard<std::mutex> lock(rendering_task_mutex_);
    ret = rendering_tasks_;
    for (auto& task : rendering_tasks_) {
      task.second.clear();
    }
  }
  return ret;
}

void GLFWRenderer::add_rendering_task(GLFWVideo* window, std::function<bool()> task) {
  std::lock_guard<std::mutex> lock(rendering_task_mutex_);
  rendering_tasks_[window].push_back(task);
}

GLFWRenderer* RendererSingleton::get() {
  if (s_instance_.get() == nullptr) {
    std::lock_guard<std::mutex> lock(creation_mutex_);
    if (s_instance_.get() == nullptr) {
      s_instance_ = std::make_unique<GLFWRenderer>();
    }
  }
  return s_instance_.get();
}

}  // namespace switcher
