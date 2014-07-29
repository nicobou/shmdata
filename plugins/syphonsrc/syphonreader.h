/*
 * This file is part of syphonsrc.
 *
 * switcher-top is free software; you can redistribute it and/or
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

#ifndef __SWITCHER_SYPHONOBJC_H__
#define __SWITCHER_SYPHONOBJC_H__

#include <string>
#include <vector>

namespace switcher
{
  typedef void (*syphonCallback)(void* context, const char* buffer, int& width, int& height);

  class SyphonReader
  {
  public:
    SyphonReader();
    SyphonReader(syphonCallback, void*);
    ~SyphonReader();
    void connect(const char* serveName = nullptr, const char* appName = nullptr);
    void disconnect();
    void getFrame(int& width, int& height, char* pixels);

  protected:
    syphonCallback callback_;
    void* context_;
    void* client_; // Holds the real Syphon client
    void* glContext_; // CGL context

    void* texture_;
    int width_, height_;
    std::vector<char> frame_;

    unsigned int drawTex_, fbo_;

    void createFBO();
    void init();
    void handleNewFrame();
  };

} // end of namespace

#endif // ifndef
