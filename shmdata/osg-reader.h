/*
 * Copyright (C) 2012 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 */

#ifndef _SHM_DATA_OSG_READER_H_
#define _SHM_DATA_OSG_READER_H_

#include <osg/Texture2D>

namespace shmdata
{
  class OsgReader_impl;		// forward declaration

  class OsgReader
  {
  public:
    OsgReader ();
    void start (const std::string & socketPath);
      osg::Texture2D * getTexture ();
     ~OsgReader ();
    void setDebug (bool debug);
  private:
      OsgReader_impl * impl_;	// PIMPL opaque pointer
  };

}				//end namespace

#endif				//_SHM_DATA_READER_H_

