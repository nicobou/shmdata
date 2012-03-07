#ifndef _SHM_DATA_H_
#define _SHM_DATA_H_
#include <string>
#include <gst/gst.h>
#include <gio/gio.h>

namespace shmdata
{
   class Writer {
    public:
       Writer (GstElement *pipeline,GstElement *Element,const std::string socketPath);
       Writer (GstElement *pipeline,GstPad *srcPad,const std::string socketPath);
       Writer ();
	~Writer ();
    private:
	GstElement *qserial_;
	GstElement *serializer_;
	GstElement *shmsink_;
	GstElement *pipeline_;
	gboolean timereset_;
	GstClockTime timeshift_;
	void make_shm_branch(const std::string socketPath);
	void set_branch_state_as_pipeline ();
	void link_branch(GstElement *srcElement);
	void link_branch(GstPad *srcPad);
	static gboolean reset_time (GstPad * pad, GstMiniObject * mini_obj, gpointer user_data);
	static void pad_unblocked (GstPad * pad, gboolean blocked, gpointer user_data);
	static void switch_to_new_serializer (GstPad * pad, gboolean blocked, gpointer user_data );
	static void on_client_connected (GstElement * shmsink, gint num, gpointer user_data); 
    };

   class Reader {
   public:
       Reader (const std::string socketPath, void(*on_first_video_data)( Reader *, void *), void *user_data);
       Reader ();
       ~Reader ();
       //where to push the video data
       void setSink (GstElement *Pipeline, GstElement *sink); 
   private:
       //pipeline elements
       GstElement *pipeline_;
       GstElement *source_;
       GstElement *deserializer_;
       GstElement *sink_;
       GstPad *sinkPad_;
       GstPad *deserialPad_;
       //monitoring the shm file
       GFile *shmfile_; 
       GFileMonitor* dirMonitor_;
       std::string socketName_;
       //user callback
       void (*on_first_video_data_)(Reader *,void *);
       void* userData_;
       //state boolean
       gboolean initialized_; //the shared video has been attached once
       //dynamic linking
       void attach ();
       void detach ();
       //gcallbacks
       static gboolean clean_source (gpointer user_data);
       static GstBusSyncReply message_handler (GstBus *bus, GstMessage *msg, gpointer user_data);
       static void file_system_monitor_change (GFileMonitor *monitor, GFile *file, GFile *other_file, GFileMonitorEvent type, gpointer user_data);
   };

}      //end namespace  shmdata
#endif //_SHM_DATA_H_

