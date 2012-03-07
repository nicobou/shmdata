#include <string.h>
#include <math.h>

#include <gst/gst.h>

#include <stdio.h>
#include <stdlib.h>
#include "shmdata.h"


GstElement *pipeline;




//--------------------------RTP---------------------------------------------------
/*
 * A simple RTP server 
 *  sends the output of alsasrc as alaw encoded RTP on port 5002, RTCP is sent on
 *  port 5003. The destination is 127.0.0.1.
 *  the receiver RTCP reports are received on port 5007
 *
 *              .-------.    .--------.      .----------.     .-------.
 *              |appsrc |    | gstpay |      | rtpbin   |     |udpsink|  RTP
 *              |      src->sink     src->send_rtp send_rtp->sink     | port=5002
 *              '-------'    '--------'      |          |     '-------'
 *                                           |          |      
 *                                           |          |     .-------.
 *                                           |          |     |udpsink|  RTCP
 *                                           |    send_rtcp->sink     | port=5003
 *                            .-------.      |          |     '-------' sync=false
 *                 RTCP       |udpsrc |      |          |               async=false
 *               port=5007    |     src->recv_rtcp      |                       
 *                            '-------'      '----------'              
 */

/* change this to send the RTP data and RTCP to another host */
#define DEST_HOST "127.0.0.1"


/* print the stats of a source */
static void
print_source_stats (GObject * source)
{
  GstStructure *stats;
  gchar *str;

  /* get the source stats */
  g_object_get (source, "stats", &stats, NULL);

  /* simply dump the stats structure */
  str = gst_structure_to_string (stats);
  g_print ("source stats: %s\n", str);

  gst_structure_free (stats);
  g_free (str);
}

/* this function is called every second and dumps the RTP manager stats */
static gboolean
print_stats (GstElement * rtpbin)
{
   GObject *session;
   GValueArray *arr;
   GValue *val;
   guint i;

   g_print ("***********************************\n");

   /* get session 0 */
   g_signal_emit_by_name (rtpbin, "get-internal-session", 0, &session);

   /* print all the sources in the session, this includes the internal source */
   g_object_get (session, "sources", &arr, NULL);

   for (i = 0; i < arr->n_values; i++) {
       GObject *source;

       val = g_value_array_get_nth (arr, i);
       source = (GObject *)g_value_get_object (val);
      
       print_source_stats (source);
   }
   g_value_array_free (arr);

   g_object_unref (session);

  return TRUE;
}



// ------------------------------- cleaning --------------------------------------
void
leave(int sig) {
    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (GST_OBJECT (pipeline));
    exit(sig);
}

// ---- data ready ----------------------------
void
on_first_video_data (shmdata::Reader *context, void *user_data)
{
  GstElement *gstpay;
  GstElement *rtpbin, *rtpsink, *rtcpsink, *rtcpsrc;
  GstPad *srcpad, *sinkpad;
    
  GstElement *pipeline = (GstElement *)user_data;
  
  
  gstpay = gst_element_factory_make ("rtpgstpay", NULL);
  g_assert (gstpay);

  GstElement *funnel= gst_element_factory_make ("funnel",NULL);
  g_assert (funnel);

    /* add capture and payloading to the pipeline and link */
  gst_bin_add_many (GST_BIN (pipeline), gstpay, 
		    funnel,
		    NULL);

  gst_element_link (funnel,gstpay);
  
  context->setSink (pipeline,funnel); 
  //gstpay);

  /* the rtpbin element */
  rtpbin = gst_element_factory_make ("gstrtpbin", "rtpbin");
  g_assert (rtpbin);

  gst_bin_add (GST_BIN (pipeline), rtpbin);

  /* the udp sinks and source we will use for RTP and RTCP */
  rtpsink = gst_element_factory_make ("udpsink", "rtpsink");
  g_assert (rtpsink);
  g_object_set (rtpsink, "port", 5002, "host", DEST_HOST, NULL);

  rtcpsink = gst_element_factory_make ("udpsink", "rtcpsink");
  g_assert (rtcpsink);
  g_object_set (rtcpsink, "port", 5003, "host", DEST_HOST, NULL);
  /* no need for synchronisation or preroll on the RTCP sink */
  g_object_set (rtcpsink, "async", FALSE, "sync", FALSE, NULL);

  rtcpsrc = gst_element_factory_make ("udpsrc", "rtcpsrc");
  g_assert (rtcpsrc);
  g_object_set (rtcpsrc, "port", 5007, NULL);

  gst_bin_add_many (GST_BIN (pipeline), rtpsink, rtcpsink, rtcpsrc, NULL);

  /* now link all to the rtpbin, start by getting an RTP sinkpad for session 0 */
  sinkpad = gst_element_get_request_pad (rtpbin, "send_rtp_sink_0");
  srcpad = gst_element_get_static_pad (gstpay, "src");
  if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
    g_error ("Failed to link audio payloader to rtpbin");
  gst_object_unref (srcpad);

  g_print ("srcpadcaps %s\n",gst_caps_to_string (gst_pad_get_caps(srcpad)));

  /* get the RTP srcpad that was created when we requested the sinkpad above and
   * link it to the rtpsink sinkpad*/
  srcpad = gst_element_get_static_pad (rtpbin, "send_rtp_src_0");
  sinkpad = gst_element_get_static_pad (rtpsink, "sink");
  if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
    g_error ("Failed to link rtpbin to rtpsink");
  gst_object_unref (srcpad);
  gst_object_unref (sinkpad);

  /* get an RTCP srcpad for sending RTCP to the receiver */
  srcpad = gst_element_get_request_pad (rtpbin, "send_rtcp_src_0");
  sinkpad = gst_element_get_static_pad (rtcpsink, "sink");
  if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
    g_error ("Failed to link rtpbin to rtcpsink");
  gst_object_unref (sinkpad);

  /* we also want to receive RTCP, request an RTCP sinkpad for session 0 and
   * link it to the srcpad of the udpsrc for RTCP */
  srcpad = gst_element_get_static_pad (rtcpsrc, "src");
  sinkpad = gst_element_get_request_pad (rtpbin, "recv_rtcp_sink_0");
  if (gst_pad_link (srcpad, sinkpad) != GST_PAD_LINK_OK)
    g_error ("Failed to link rtcpsrc to rtpbin");
  gst_object_unref (srcpad);

  /* set the pipeline to playing */
  g_print ("starting sender pipeline\n");
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  /* print stats every second */
  g_timeout_add (1000, (GSourceFunc) print_stats, rtpbin);

}

/* build a pipeline equivalent to:
 *
 * gst-launch -v gstrtpbin name=rtpbin \
 *    my binary frame stream  ! rtpbin.send_rtp_sink_0  \
 *           rtpbin.send_rtp_src_0 ! udpsink port=5002 host=$DEST                      \
 *           rtpbin.send_rtcp_src_0 ! udpsink port=5003 host=$DEST sync=false async=false \
 *        udpsrc port=5007 ! rtpbin.recv_rtcp_sink_0
 */
int
main (int argc, char *argv[])
{
  GMainLoop *loop;
  
  (void) signal(SIGINT,leave);

  /* always init first */
  gst_init (&argc, &argv);

  std::string socketName;
  if (argc != 2) {
      g_printerr ("Usage: %s <socket-path>\n", argv[0]);
      return -1;
  }
  socketName.append (argv[1]);
  
  
  /* the pipeline to hold everything */
  pipeline = gst_pipeline_new (NULL);
  g_assert (pipeline);

  shmdata::Reader *reader;
  reader = new shmdata::Reader (socketName, &on_first_video_data,pipeline);

  /* we need to run a GLib main loop to get the messages */
  loop = g_main_loop_new (NULL, FALSE);
  g_main_loop_run (loop);

  g_print ("stopping sender pipeline\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);

  return 0;
}
