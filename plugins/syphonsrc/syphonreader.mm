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

#include "syphonreader.h"
#import <Syphon/Syphon.h>

#include <iostream>

using namespace std;

namespace switcher
{

  SyphonReader::SyphonReader() :
    callback_(nullptr),
    context_(nullptr),
    client_(nullptr),
    glContext_(nullptr),
    width_(0),
    height_(0),
    drawTex_(0),
    fbo_(0)
  {
    init();
  }

  SyphonReader::SyphonReader(syphonCallback callback, void* context) :
    callback_(nullptr),
    context_(nullptr),
    client_(nullptr),
    glContext_(nullptr),
    width_(0),
    height_(0),
    drawTex_(0),
    fbo_(0)
  {
    init();
    callback_ = callback;
    context_ = context;
  }

  SyphonReader::~SyphonReader()
  {
    disconnect();
  }

  void
  SyphonReader::connect(const char* serverName, const char* appName)
  {
    SyphonClient* newClient = nullptr;
    SyphonClient* client = (SyphonClient*)client_;

    NSArray* serverMatches;
    if (serverName != nullptr && appName == nullptr)
      serverMatches = [[SyphonServerDirectory sharedDirectory] serversMatchingName:[NSString stringWithCString:serverName] appName:nil]; 
    else if (serverName == nullptr && appName != nullptr)
      serverMatches = [[SyphonServerDirectory sharedDirectory] serversMatchingName:nil appName:[NSString stringWithCString:appName]]; 
    else if (serverName != nullptr && appName != nullptr)
      serverMatches = [[SyphonServerDirectory sharedDirectory] serversMatchingName:[NSString stringWithCString:serverName] appName:[NSString stringWithCString:appName]]; 
    else
      serverMatches = [[SyphonServerDirectory sharedDirectory] serversMatchingName:nil appName:nil]; 

    if ([serverMatches count] != 0)
    {
      NSString *currentServer, *foundServer;
      foundServer = [[serverMatches lastObject] objectForKey:SyphonServerDescriptionUUIDKey];

      currentServer = [client.serverDescription objectForKey:SyphonServerDescriptionUUIDKey];

      if (foundServer && [currentServer isEqualToString:foundServer])
        newClient = [client retain];
      else
        newClient = [[SyphonClient alloc] initWithServerDescription:[serverMatches lastObject] options:nil newFrameHandler:^(SyphonClient* client) {
          handleNewFrame();
        }];
    }
    else
      cout << "SyphonClient::connect - Unable to find any server matching the given serverName - " << serverName << " and appName - " << appName << endl;

    [client release];
    client_ = newClient;
  }

  void
  SyphonReader::disconnect()
  {
    if (client_ != nullptr)
      [(SyphonClient*)client_ release];

    if (glContext_ != nullptr)
      CGLDestroyContext(*(CGLContextObj*)glContext_);
  }

  void
  SyphonReader::getFrame(int& width, int& height, char* pixels)
  {
  }

  void
  SyphonReader::createFBO()
  {
    if (drawTex_ != 0)
      glDeleteTextures(1, &drawTex_);
    glEnable(GL_ALL_ATTRIB_BITS);
    glGenTextures(1, &drawTex_);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, drawTex_);
    glTexImage2D(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA8, width_, height_, 0, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, nullptr);

    if (fbo_ != 0)
      glDeleteFramebuffers(1, &fbo_);
    glGenFramebuffers(1, &fbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE_ARB, drawTex_, 0);
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE)
      cout << "SyphonReader::createFBO - Error while creating FBO" << endl;
    else
      cout << "SyphonReader::createFBO - FBO created successfully" << endl;

    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }
  
  void
  SyphonReader::init()
  {
    CGLPixelFormatAttribute attr[4] = {
      kCGLPFAAccelerated,
      kCGLPFAOpenGLProfile,
      (CGLPixelFormatAttribute) kCGLOGLPVersion_Legacy,
      (CGLPixelFormatAttribute) 0
    };

    CGLPixelFormatObj pix;
    CGLError error;
    GLint num;
    error = CGLChoosePixelFormat(attr, &pix, &num);
    if (error != kCGLNoError)
      cout << "SyphonReader::init - Error while creating the pixel format: " << error << endl;
    else
      cout << "SyphonReader::init - Pixel format created successfully" << endl;
    glContext_ = (void*)(new CGLContextObj);
    error = CGLCreateContext(pix, nullptr, (CGLContextObj*)glContext_);
    if (error != kCGLNoError)
      cout << "SyphonReader::init - Error while creating context: " << error << endl;
    else
      cout << "SyphonReader::init - CGL context created successfully" << endl;

    CGLDestroyPixelFormat(pix);
  }

  void
  SyphonReader::handleNewFrame()
  {
    CGLError error;
    error = CGLSetCurrentContext(*(CGLContextObj*)glContext_);
    if (error != kCGLNoError)
      cout << "SyphonReader::handleNewFrame - Error while setting context: " << error << endl;

    SyphonImage* latestImage = [client_ newFrameImageForContext:CGLGetCurrentContext()];

    NSSize texSize = [latestImage textureSize];

    if (width_ != texSize.width || height_ != texSize.height)
    {
      width_ = texSize.width;
      height_ = texSize.height;
      frame_.resize(width_ * height_ * 4); // RGBA = 4 bytes per pixel
      createFBO();
    }

    glGetError();

    glBindFramebuffer(GL_FRAMEBUFFER, fbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE_ARB, drawTex_, 0);
    glViewport(0, 0, width_, height_);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, width_, 0.0, height_, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, [latestImage textureName]);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    glColor4f(1.0, 1.0, 1.0, 1.0);
    GLfloat texCoords[] = {
      0.0, height_,
      width_, height_,
      width_, 0.0,
      0.0, 0.0
    };

    GLfloat verts[] = {
      0.0, 0.0,
      width_, 0.0,
      width_, height_,
      0.0, height_
    };

    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, texCoords);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, verts);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    CGLFlushDrawable(*(CGLContextObj*)glContext_);

    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, drawTex_);
    glGetTexImage(GL_TEXTURE_RECTANGLE_ARB, 0, GL_RGBA, GL_UNSIGNED_BYTE, frame_.data());
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    GLenum glError = glGetError();
    if (glError != GL_NO_ERROR)
      cout << "SyphonReader::handleNewFrame - Error while copying shared texture. GL state: " << glError << endl;

    CGLSetCurrentContext(nullptr);

    if (callback_ != nullptr)
    {
      callback_(context_, frame_.data(), width_, height_);
    }

    [latestImage release];
  }

}
