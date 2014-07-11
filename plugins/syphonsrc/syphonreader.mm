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
    client_(NULL),
    glContext_(NULL),
    width_(0),
    height_(0)
  {
    CGLPixelFormatAttribute attr[4] = {
      kCGLPFAAccelerated,
      kCGLPFAOpenGLProfile,
      (CGLPixelFormatAttribute) kCGLOGLPVersion_3_2_Core,
      (CGLPixelFormatAttribute) 0
    };

    CGLPixelFormatObj pix;
    CGLError error;
    GLint num;
    error = CGLChoosePixelFormat(attr, &pix, &num);
    glContext_ = (void*)(new CGLContextObj);
    error = CGLCreateContext(pix, NULL, (CGLContextObj*)glContext_);

    CGLDestroyPixelFormat(pix);
  }

  SyphonReader::~SyphonReader()
  {
    disconnect();
  }

  void SyphonReader::connect(const char* serverName)
  {
    SyphonClient* newClient = NULL;
    SyphonClient* client = (SyphonClient*)client_;

    NSArray* serverMatches = [[SyphonServerDirectory sharedDirectory] serversMatchingName:nil appName:[NSString stringWithCString:"Simple Server"]]; 

    cout << [serverMatches count] << endl;
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

    [client release];
    client_ = newClient;
  }

  void SyphonReader::disconnect()
  {
    if (client_ != NULL)
      [(SyphonClient*)client_ release];

    if (glContext_ != NULL)
      CGLDestroyContext(*(CGLContextObj*)glContext_);
  }

  void SyphonReader::getFrame(int& width, int& height, char* pixels)
  {
  }
  
  void SyphonReader::handleNewFrame()
  {
    CGLSetCurrentContext(*(CGLContextObj*)glContext_);

    SyphonImage* latestImage = [client_ newFrameImageForContext:CGLGetCurrentContext()];

    NSSize texSize = [latestImage textureSize];

    if (width_ != texSize.width || height_ != texSize.height)
    {
      width_ = texSize.width;
      height_ = texSize.height;
      frame_.resize(width_ * height_ * 4); // RGBA = 4 bytes per pixel
    }

    glGetError();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_RECTANGLE_ARB, [latestImage textureName]);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, frame_.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    CGLSetCurrentContext(NULL);
  }

}
