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

#include <osg/Node>
#include <osg/Geometry>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/DrawPixels>
#include <osg/PolygonOffset>
#include <osg/Geode>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgText/Text>
#include <osgViewer/Viewer>
#include <iostream>

#include "shmdata/osg-reader.h"

osg::Node * createFilterWall (osg::BoundingBox & bb, osg::Texture2D * texture)
{
  osg::Group * group = new osg::Group;

  // left hand side of bounding box.
  osg::Vec3 top_left (bb.xMin (), bb.yMin (), bb.zMax ());
  osg::Vec3 bottom_left (bb.xMin (), bb.yMin (), bb.zMin ());
  osg::Vec3 bottom_right (bb.xMin (), bb.yMax (), bb.zMin ());
  osg::Vec3 top_right (bb.xMin (), bb.yMax (), bb.zMax ());
  osg::Vec3 center (bb.xMin (), (bb.yMin () + bb.yMax ()) * 0.5f,
		    (bb.zMin () + bb.zMax ()) * 0.5f);
  //float height = bb.zMax()-bb.zMin();

  // create the geometry for the wall.
  osg::Geometry * geom = new osg::Geometry;

  osg::Vec3Array * vertices = new osg::Vec3Array (4);
  (*vertices)[0] = top_left;
  (*vertices)[1] = bottom_left;
  (*vertices)[2] = bottom_right;
  (*vertices)[3] = top_right;
  geom->setVertexArray (vertices);

   osg::Vec2Array * texcoords = new osg::Vec2Array (4);
   (*texcoords)[0].set (0.0f, 1.0f);
   (*texcoords)[1].set (0.0f, 0.0f);
   (*texcoords)[2].set (1.0f, 0.0f);
   (*texcoords)[3].set (1.0f, 1.0f);
   geom->setTexCoordArray (0, texcoords);

  osg::Vec3Array * normals = new osg::Vec3Array (1);
  (*normals)[0].set (1.0f, 0.0f, 0.0f);
  geom->setNormalArray (normals);
  geom->setNormalBinding (osg::Geometry::BIND_OVERALL);

  osg::Vec4Array * colors = new osg::Vec4Array (1);
  (*colors)[0].set (1.0f, 1.0f, 1.0f, 1.0f);
  geom->setColorArray (colors);
  geom->setColorBinding (osg::Geometry::BIND_OVERALL);

  geom->addPrimitiveSet (new osg::DrawArrays (GL_QUADS, 0, 4));

  osg::Geode * geom_geode = new osg::Geode;
  geom_geode->addDrawable (geom);
  group->addChild (geom_geode);

  osg::StateSet * stateset = geom->getOrCreateStateSet ();
  stateset->setTextureAttributeAndModes (0, texture, osg::StateAttribute::ON);
  stateset->setMode(GL_BLEND,osg::StateAttribute::ON);

  return group;

}

osg::Node * createModel (osg::Texture2D * texture)
{

  // create the root node which will hold the model.
  osg::Group * root = new osg::Group ();

  // turn off lighting
  root->getOrCreateStateSet ()->setMode (GL_LIGHTING,
					 osg::StateAttribute::OFF);

  osg::BoundingBox bb (0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);

  root->addChild (createFilterWall (bb, texture));

  return root;
}

int
main (int argc, char *argv[])
{
  // construct the viewer.
  osgViewer::Viewer viewer;

  if (argc != 2)
    {
      std::cerr << "Usage: " << argv[0] << " <socket-path>" << std::endl;
      return -1;
    }

  std::string * socketName = new std::string (argv[1]);

  // int i;
  // for (i = 0; i < 5; i++)
  //   {
  //     //creating the Reader
  //     shmdata::OsgReader * reader = new shmdata::OsgReader ();
  //     reader->setDebug (true);
  //     reader->start (*socketName);
  //     delete reader;
  //   }

  shmdata::OsgReader * reader = new shmdata::OsgReader ();
  reader->setDebug (true);
  reader->start (*socketName);

  //using the texture
  viewer.setSceneData (createModel (reader->getTexture ()));

  return viewer.run ();
}

