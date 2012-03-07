/* OpenSceneGraph example, osgtexture2D.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
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

#include <gst/gst.h>
#include "shmdata.h"
#include <gst/app/gstappsink.h>

// #include <string.h> //memcpy
// gpointer raw_buffer=NULL;
// int raw_buffer_size=0;

GstBuffer *last_buffer_ = NULL;

osg::Node* createFilterWall(osg::BoundingBox& bb,osg::Texture2D* texture)
{
    osg::Group* group = new osg::Group;
    
    // left hand side of bounding box.
    osg::Vec3 top_left(bb.xMin(),bb.yMin(),bb.zMax());
    osg::Vec3 bottom_left(bb.xMin(),bb.yMin(),bb.zMin());
    osg::Vec3 bottom_right(bb.xMin(),bb.yMax(),bb.zMin());
    osg::Vec3 top_right(bb.xMin(),bb.yMax(),bb.zMax());
    osg::Vec3 center(bb.xMin(),(bb.yMin()+bb.yMax())*0.5f,(bb.zMin()+bb.zMax())*0.5f);    
    //float height = bb.zMax()-bb.zMin();
    
    // create the geometry for the wall.
    osg::Geometry* geom = new osg::Geometry;
    
    osg::Vec3Array* vertices = new osg::Vec3Array(4);
    (*vertices)[0] = top_left;
    (*vertices)[1] = bottom_left;
    (*vertices)[2] = bottom_right;
    (*vertices)[3] = top_right;
    geom->setVertexArray(vertices);
    
    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0.0f,1.0f);
    (*texcoords)[1].set(0.0f,0.0f);
    (*texcoords)[2].set(1.0f,0.0f);
    (*texcoords)[3].set(1.0f,1.0f);
    geom->setTexCoordArray(0,texcoords);

    osg::Vec3Array* normals = new osg::Vec3Array(1);
    (*normals)[0].set(1.0f,0.0f,0.0f);
    geom->setNormalArray(normals);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS,0,4));
    
    osg::Geode* geom_geode = new osg::Geode;
    geom_geode->addDrawable(geom);
    group->addChild(geom_geode);
    
    osg::StateSet* stateset = geom->getOrCreateStateSet();
    stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
    
    return group;
    
}



osg::Node* createModel(osg::Texture2D* texture)
{

    // create the root node which will hold the model.
    osg::Group* root = new osg::Group();
    
    // turn off lighting 
    root->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    osg::BoundingBox bb(0.0f,0.0f,0.0f,1.0f,1.0f,1.0f);

    root->addChild(createFilterWall(bb,texture));
    
    return root;
}


/* called when the appsink notifies us that there is a new buffer ready for
 * processing */
static void
on_new_buffer_from_source (GstElement * elt, gpointer user_data)
{
  GstBuffer *buffer;
  osg::Texture2D* texture = (osg::Texture2D*) user_data;

  buffer = gst_app_sink_pull_buffer (GST_APP_SINK (elt));
  GstStructure *imgProp=gst_caps_get_structure (GST_BUFFER_CAPS(buffer),0);
  
  //g_print ("%s\n",gst_caps_to_string (GST_BUFFER_CAPS(buffer)));
//  g_print ("%d %d\n",g_value_get_int (gst_structure_get_value (imgProp,"width")), g_value_get_int (gst_structure_get_value (imgProp,"height")));

  int curWidth = g_value_get_int (gst_structure_get_value (imgProp,"width"));
  int curHeight = g_value_get_int (gst_structure_get_value (imgProp,"height"));

  osg::Image *img = new osg::Image;
  img->setOrigin(osg::Image::TOP_LEFT); 
  img->setImage(curWidth, 
   		curHeight, 
   		0, 
   		GL_RGB, 
   		GL_RGB, 
   		GL_UNSIGNED_SHORT_5_6_5, 
		GST_BUFFER_DATA (buffer),
   		osg::Image::NO_DELETE, 
   		1);

  texture->setImage(img);

   if (last_buffer_ != NULL)  
       gst_buffer_unref (last_buffer_);
  last_buffer_ = buffer;
}



void
on_first_video_data (shmdata::Reader *context, void *user_data)
{

    osg::Texture2D* texture = (osg::Texture2D*) user_data;
    
    GstElement *pipeline = gst_pipeline_new (NULL);
    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    g_print ("creating element to display the shared video \n");
    GstElement *shmDisplay = gst_element_factory_make ("appsink", NULL);
    GstElement *videoConv = gst_element_factory_make ("ffmpegcolorspace", NULL);

    g_object_set (G_OBJECT (shmDisplay), "caps", gst_caps_from_string ("video/x-raw-rgb, bpp=16, depth=16"), NULL);

    g_object_set (G_OBJECT (shmDisplay), "emit-signals", TRUE, "sync", FALSE, NULL);
    g_signal_connect (shmDisplay, "new-buffer",
     		      G_CALLBACK (on_new_buffer_from_source), texture);

    //in order to be dynamic, the shared video is linking to an 
    //element accepting request pad (as funnel of videomixer)
    GstElement *funnel       = gst_element_factory_make ("funnel", NULL);
    g_object_set (G_OBJECT (shmDisplay), "sync", FALSE, NULL);
    
    if (!shmDisplay || !videoConv || !funnel ) {
	g_printerr ("One element could not be created. \n");
    }

    //element must have the same state as the pipeline
    gst_element_set_state (shmDisplay, GST_STATE_PLAYING);
    gst_element_set_state (funnel, GST_STATE_PLAYING);
    gst_element_set_state (videoConv, GST_STATE_PLAYING);
    gst_bin_add_many (GST_BIN (pipeline), funnel, videoConv, shmDisplay, NULL);
    gst_element_link_many (funnel, videoConv, shmDisplay,NULL);
    
    //now tells the shared video reader where to write the data
    context->setSink (pipeline, funnel);
}


void
sharedVideoRead (gpointer user_data)
{
    osg::Texture2D* texture = (osg::Texture2D*) user_data; 
    //texture->setImage(osgDB::readImageFile("truc.tga"));

    GMainLoop *loop;
 
    
    // GstElement *pipeline;
    
    /* Initialisation */
    gst_init (NULL, NULL);
    loop = g_main_loop_new (NULL, FALSE);
    // /* Create gstreamer elements */
    // pipeline   = gst_pipeline_new (NULL);
    // /* we add a message handler */
    // bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
    // gst_bus_add_watch (bus, bus_call, loop);
    // gst_object_unref (bus);
    
    //   GstElement *localVideoSource = gst_element_factory_make ("videotestsrc", NULL);
    //   GstElement *localDisplay = gst_element_factory_make ("xvimagesink", NULL);
    // if (!pipeline || !localVideoSource || !localDisplay) {
    // 	g_printerr ("One element could not be created. Exiting.\n");
    //   }
    //  gst_bin_add_many (GST_BIN (pipeline), localVideoSource, localDisplay, NULL);
    //   gst_element_link (localVideoSource, localDisplay);
    
    /*shmdata::Reader *reader =*/ new shmdata::Reader ("/tmp/rt", &on_first_video_data,texture);
    
//    gst_element_set_state (pipeline, GST_STATE_PLAYING);
    
    /* Iterate */
    g_print ("Running...\n");
    g_main_loop_run (loop);
}

int main(int , char **)
{
    // construct the viewer.
    osgViewer::Viewer viewer;
    
    osg::Texture2D* texture = new osg::Texture2D;
    texture->setDataVariance(osg::Object::DYNAMIC);
    texture->setResizeNonPowerOfTwoHint(false);

    // add model to viewer.
    viewer.setSceneData( createModel(texture) );

    g_thread_init (NULL);
    /*GThread *sharedVideoThread =*/ g_thread_create ((GThreadFunc) sharedVideoRead, texture, FALSE, NULL);
    
    return viewer.run();
}
