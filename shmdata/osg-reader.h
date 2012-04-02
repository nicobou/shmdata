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

/** \addtogroup libshmdata-osg
 * provides video reading to an OpenSceneGraph Texture.
 * compile with `pkg-config --cflags --libs shmdata-osg-0.2`
 *  @{
 */

#ifndef _SHM_DATA_OSG_READER_H_
#define _SHM_DATA_OSG_READER_H_

#include <osg/Texture2D>

/**
 * @file   osg-reader.h
 * 
 * @brief Reading video data from a shared memory into an OpenSceneGraph texture 
 * 
 * 
 */

/**
 * \class shmdata::OsgReader
 *
 * \brief Reading video data from a shared memory into an OpenSceneGraph texture.  
 *  Alpha channel is supported.
 */


namespace shmdata
{
  class OsgReader_impl;		// forward declaration

  class OsgReader
  {
  public:
    /// Create a shmdata::OsgReader
    OsgReader ();
    
    /** 
     * Start the reader
     * 
     * @param socketPath is the file name of the shared memory
     */
    void start (const std::string & socketPath);

    /** 
     * Get the texture where video is written.
     * 
     * @return the pointer to the osg::Texture2D instance where video is written
     */
    osg::Texture2D * getTexture ();

     ~OsgReader ();

     /** 
      * Set debug enable printing debug messages
      * 
      * @param debug is the boolean enabling or disabling debug 
      */
    void setDebug (bool debug);
  private:
      OsgReader_impl * impl_;	// PIMPL opaque pointer
  };

}				//end namespace

#endif				//_SHM_DATA_READER_H_

/** @}*/
