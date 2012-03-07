#include <gst/gst.h>
#include <signal.h>
#include <string>
#include "shmdata.h"


GstElement *pipeline;
GstElement *source;    
GstElement *tee;       
GstElement *qlocalxv;  
GstElement *imgsink;   
GstElement *timeoverlay;
GstElement *camsource;

std::string socketName;
shmdata::Writer *writer;

//clean up pipeline when ctrl-c
void
leave(int sig) {
    g_print ("Returned, stopping playback\n");
    gst_element_set_state (pipeline, GST_STATE_NULL);

    g_print ("Deleting pipeline\n");
    gst_object_unref (GST_OBJECT (pipeline));

    exit(sig);
}

//will create the shared video, reading from the tee element  
static gboolean  
add_shared_video_writer()
{
    writer = new shmdata::Writer (pipeline,tee,socketName);
    g_print ("Now writing to the shared memory\n");
    return FALSE;
}

int
main (int   argc,
      char *argv[])
{
    (void) signal(SIGINT,leave);

    gst_init (&argc, &argv);
    GMainLoop *loop = g_main_loop_new (NULL, FALSE);

    /* Check input arguments */
    if (argc != 2) {
	g_printerr ("Usage: %s <socket-path>\n", argv[0]);
	return -1;
    }
    socketName.append (argv[1]);


    /* Create gstreamer elements */
    pipeline    = gst_pipeline_new (NULL);
    source      = gst_element_factory_make ("videotestsrc",  NULL);
    
    camsource   = gst_element_factory_make ("v4l2src",  NULL);

    timeoverlay = gst_element_factory_make ("timeoverlay", NULL);
    tee         = gst_element_factory_make ("tee", NULL);
 
    qlocalxv    = gst_element_factory_make ("queue", NULL);
    imgsink     = gst_element_factory_make ("xvimagesink", NULL);

    
    if (!pipeline || !source || !timeoverlay || !tee || !qlocalxv || !imgsink ) {
	g_printerr ("One element could not be created. Exiting.\n");
	return -1;
    }

    g_object_set (G_OBJECT (imgsink), "sync", FALSE, NULL);

    /*specifying video format*/
    // GstCaps *videocaps;  
    // videocaps = gst_caps_new_simple ("video/x-raw-yuv",  
    //   				     "format", GST_TYPE_FOURCC, GST_MAKE_FOURCC ('I', '4', '2', '0'),  
    //   				     "framerate", GST_TYPE_FRACTION, 30, 1,  
    //   				     "pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1,  
    // 				     /* "width", G_TYPE_INT, 600,  */ 
    // 				     /* "height", G_TYPE_INT, 400,  */ 
    // 				     "width", G_TYPE_INT, 1920,   
    // 				     "height", G_TYPE_INT, 1080,   
    //   				     NULL);  

    gst_bin_add_many (GST_BIN (pipeline),
		      //source, 
		      camsource, 
		      timeoverlay, tee, qlocalxv, imgsink, NULL);


    //shared video can be pluged before or after the pipeline state is set to PLAYING 
    g_timeout_add (1000, (GSourceFunc) add_shared_video_writer, NULL);
    //add_shared_video_writer();

    /* we link the elements together */
    //gst_element_link_filtered (source, timeoverlay,videocaps);
    gst_element_link (camsource,timeoverlay);
    gst_element_link (timeoverlay, tee);
    gst_element_link_many (tee, qlocalxv,imgsink,NULL);

    /* Set the pipeline to "playing" state*/
    gst_element_set_state (pipeline, GST_STATE_PLAYING);

    g_print ("Running...\n");
    g_main_loop_run (loop);

    /* Out of the main loop, clean up nicely */
    g_print ("Returned, stopping playback\n");
    gst_element_set_state (pipeline, GST_STATE_NULL);
    g_print ("Deleting pipeline\n");
    gst_object_unref (GST_OBJECT (pipeline));

    return 0;
}
