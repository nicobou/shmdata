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
#include "switcher/scope-exit.hpp"

namespace switcher {

std::unique_ptr<GLFWRenderer> RendererSingleton::s_instance_;
std::mutex RendererSingleton::creation_mutex_;

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

    // Check if any window asked to be unsubscribed from the render loop.
    check_unsubscribe_from_render_loop();

    int current_window = 0;
    for (auto& instance : rendering_tasks_) {
      ++current_window;
      auto current = instance.first;
      glfwMakeContextCurrent(current->window_);
      On_scope_exit { glfwMakeContextCurrent(nullptr); };
      if (!do_rendering_tasks(current)) {
        continue;
      }

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
        glBindTexture(GL_TEXTURE_2D, 0);
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

    glfwPollEvents();
  }
}

void GLFWRenderer::subscribe_to_render_loop(GLFWVideo* window) {
  add_rendering_task(window, [window]() {
    glDrawBuffer(GL_FRONT_AND_BACK);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDrawBuffer(GL_BACK);
    return true;
  });
}

void GLFWRenderer::unsubscribe_from_render_loop(GLFWVideo* window) {
  // Wait for the render loop to be done with the window before destroying it.
  std::unique_lock<std::mutex> lock(subscription_mutex_);
  unsubscribers_.push_back(window);
  cond_subscription_.wait(lock);
}

void GLFWRenderer::check_unsubscribe_from_render_loop() {
  std::lock_guard<std::mutex> lock(subscription_mutex_);

  for (auto& unsubscriber : unsubscribers_) {
    auto renderer = rendering_tasks_.find(unsubscriber);
    if (renderer != rendering_tasks_.end()) rendering_tasks_.erase(renderer);
  }

  cond_subscription_.notify_all();

  unsubscribers_.clear();
}

void GLFWRenderer::add_rendering_task(GLFWVideo* window, std::function<bool()> task) {
  std::lock_guard<std::mutex> lock(rendering_task_mutex_);
  rendering_tasks_[window].push_back(task);
}

bool GLFWRenderer::do_rendering_tasks(GLFWVideo* window) {
  auto ret = true;

  std::lock_guard<std::mutex> lock(rendering_task_mutex_);
  auto& tasks = rendering_tasks_[window];
  for (auto& task : tasks) {
    if (task && !task()) ret = false;
  }

  tasks.clear();  // We still clear even if a task failed.

  return ret;
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
};
