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

//FIXME change license to GPL (libsamplerate) 

#include "m_pd.h"
#include <gst/gst.h>
#include <glib.h>
#include <string.h>
#include <math.h>
#include <samplerate.h>

#include "shmdata/any-data-reader.h"


#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

/* ------------------------ shmsrc~ ----------------------------- */

/* tilde object to take absolute value. */

static t_class *shmsrc_tilde_class;

typedef struct _shmsrc_tilde_buf
{
  void *audio_data;
  void *shm_buf; //for freeing memory after being used
  int remaining_samples;
  int num_channels_to_output;
  int sample_size;
  int num_unused_channels;
  int sample_rate;
} t_shmsrc_tilde_buf;

typedef struct _shmsrc_tilde
{
  t_object x_obj; 
  shmdata_any_reader_t *x_reader;
  double x_init_date;
  int x_num_outlets;
  t_shmsrc_tilde_buf *x_current_audio_buf; 
  long x_pd_samplerate;/* samplerate we're running at */
  char x_shmdata_path[512];
  char x_shmdata_prefix[256];
  char x_shmdata_name[256];
  GAsyncQueue *x_audio_queue;
  t_int **x_myvec;                               /* vector we pass on in the DSP routine */
  t_float x_f;    	/* place to hold inlet's value if it's set by message */
  
} t_shmsrc_tilde;

void shmsrc_tilde_on_data (shmdata_any_reader_t *reader,
	      void *shmbuf,
	      void *data,
	      int data_size,
	      unsigned long long timestamp,
	      const char *type_description, void *user_data)
{
  t_shmsrc_tilde *x = (t_shmsrc_tilde *) user_data;
  
  /* printf ("data %p, data size %d, timestamp %llu, type descr %s\n",   */
  /* 	  data, data_size, timestamp, type_description);   */
  GstStructure *meta_data = gst_structure_from_string (type_description, NULL);
  if (meta_data == NULL) 
    { 
      //post ("metadata is NULL\n"); 
      return; 
    } 
  if (!g_str_has_prefix (gst_structure_get_name (meta_data), "audio/")) 
    { 
      //post ("not an audio stream\n"); 
      return; 
    } //should be "audio/... 
  
  t_shmsrc_tilde_buf *audio_buf = g_malloc0 (sizeof (t_shmsrc_tilde_buf));

  int channels = -1; 
  int samplerate = -1; 
  int width = -1; 
  gst_structure_get (meta_data,  
   		     "rate", G_TYPE_INT, &samplerate,  
   		     "channels", G_TYPE_INT, &channels,  
   		     "width", G_TYPE_INT, &width,  
   		     NULL); 
  if (channels > x->x_num_outlets)
    audio_buf->num_channels_to_output = x->x_num_outlets;
 else if (channels < 0)
    audio_buf->num_channels_to_output = 0;
 else 
    audio_buf->num_channels_to_output = channels;
  
   audio_buf->num_unused_channels = channels - x->x_num_outlets;
  if (audio_buf->num_unused_channels < 0)
    audio_buf->num_unused_channels = 0;

  audio_buf->sample_rate = samplerate;
  audio_buf->sample_size = width;
  audio_buf->remaining_samples = data_size / ((width/8) * channels);
  audio_buf->audio_data = data;
  audio_buf->shm_buf = shmbuf;
  
  //printf ("rate %d, channels %d, width %d num sample=%d\n",samplerate, channels, width, data_size / ((width/8) *channels)); 
  
  g_async_queue_push (x->x_audio_queue, audio_buf);
   
  //free the data, can also be called later
  //shmdata_any_reader_free (shmbuf);
}

static void shmsrc_tilde_reader_restart (t_shmsrc_tilde *x)
{
  shmdata_any_reader_close (x->x_reader);
  x->x_reader = shmdata_any_reader_init ();
  shmdata_any_reader_set_debug (x->x_reader, SHMDATA_ENABLE_DEBUG);
  shmdata_any_reader_set_on_data_handler (x->x_reader, &shmsrc_tilde_on_data, (void *) x);
  //shmdata_any_reader_set_data_type (reader, "application/helloworld_");
  shmdata_any_reader_start (x->x_reader, x->x_shmdata_path);
}


static t_int *shmsrc_tilde_perform(t_int *w)
{
  t_shmsrc_tilde *x = (t_shmsrc_tilde*) (w[1]); 
  int n = (int)(w[2]);  
  t_float *out[x->x_num_outlets];  
  int i;  
  for (i = 0; i < x->x_num_outlets; i++)  
    out[i] = (t_float *)(w[3 + i]);  
  
  if (x->x_current_audio_buf == NULL)
    {
      x->x_current_audio_buf = g_async_queue_try_pop (x->x_audio_queue); //FIXME wrap this in a function and resample
      if (x->x_current_audio_buf != NULL) 
	{ 
	  static float input [2048], output [2048] ;
	  SRC_DATA src_data ;
	  int input_len, output_len, error, terminate ;
	  double src_ratio = 0.789;

	  if (src_ratio >= 1.0)
	    {
	      output_len = 2048;
	      input_len = (int) floor (2048 / src_ratio) ;
	    }
	  else
	  {	
	    input_len = 2048;
	    output_len = (int) floor (2048 * src_ratio) ;
	  } ;
	
	src_data.data_in = input ;
	src_data.input_frames = input_len ;
	
	src_data.src_ratio =  src_ratio;
	
	src_data.data_out = output ;
	src_data.output_frames = output_len;

	if ((error = src_simple (&src_data, SRC_ZERO_ORDER_HOLD, 1))) //SRC_ZERO_ORDER_HOLD SRC_LINEAR SRC_SINC_FASTEST
	  {
	    printf ("\n\nLine %d : %s\n\n", __LINE__, src_strerror (error)) ;
	    exit (1) ;
	  };

	terminate = (int) ceil ((src_ratio >= 1.0) ? src_ratio : 1.0 / src_ratio) ;

	if (fabs (src_data.output_frames_gen - src_ratio * input_len) > 2 * terminate)
	{	
	  printf ("\n\nLine %d : bad output data length %ld should be %d.\n", __LINE__,
		  src_data.output_frames_gen, (int) floor (src_ratio * input_len)) ;
	  printf ("\tsrc_ratio  : %.4f\n", src_ratio) ;
	  printf ("\tinput_len  : %d\n\toutput_len : %d\n\n", input_len, output_len) ;
	} ;

	  g_print ("stream sample rate %d, pd sample rate=%d queue size=%d\n",
		   x->x_current_audio_buf->sample_rate,
		   x->x_pd_samplerate,
		   g_async_queue_length (x->x_audio_queue)); 
	}
      
    }

  if (x->x_current_audio_buf != NULL) 
    { 
      //deinterleaving channels  
      while (n--)  
   	{ 
	  //if case the audio buf ended during the loop
	  if (x->x_current_audio_buf == NULL)
	    for (i=0; i < x->x_num_outlets; i++)  
	      *(out[i]++) = 0.;
	  else
	    {
	      //give audio data 
	      for (i=0; i < x->x_current_audio_buf->num_channels_to_output; i++) 
		{ 
		  if (x->x_current_audio_buf->sample_size == 32)
		    {
		      *(out[i]++) = (t_float) *(t_float *)x->x_current_audio_buf->audio_data;   
		      x->x_current_audio_buf->audio_data += sizeof(t_float); 
		    }
		  else if (x->x_current_audio_buf->sample_size == 16)
		    {
		      *out[i]++ = (t_float)(*(gint16 *)x->x_current_audio_buf->audio_data * 3.051850e-05);
		      x->x_current_audio_buf->audio_data += sizeof(gint16); 
		    }
		}  
	      for (i=0; i < x->x_current_audio_buf->num_unused_channels; i++) 
		x->x_current_audio_buf->audio_data ++; 
	      //give zero when not enough audio for all outlets 
	      for (i=x->x_current_audio_buf->num_channels_to_output; i < x->x_num_outlets; i++)  
		*(out[i]++) = 0.;    
	      //tracking remaining samples in the audio buf
	      x->x_current_audio_buf->remaining_samples--;
	      if (x->x_current_audio_buf->remaining_samples == 0)
		{
		  shmdata_any_reader_free (x->x_current_audio_buf->shm_buf);
		  g_free (x->x_current_audio_buf);
		  x->x_current_audio_buf = g_async_queue_try_pop (x->x_audio_queue); 
		}
	    }
	} 
    } 
   else //no data to play 
    {
      /* set output to zero */  
      while (n--)  
	for (i = 0; i < x->x_num_outlets; i++)  
	  *(out[i]++) = 0.;  
    }

  return (w + 3 + x->x_num_outlets);	
}

/* called to start DSP.  Here we call Pd back to add our perform
   routine to a linear callback list which Pd in turn calls to grind
   out the samples. */
static void shmsrc_tilde_dsp(t_shmsrc_tilde *x, t_signal **sp)
{
  x->x_myvec[0] = (t_int*)x;
  x->x_myvec[1] = (t_int*)sp[0]->s_n;
  x->x_pd_samplerate = (long)sp[0]->s_sr;
  int i;
  for (i = 0; i < x->x_num_outlets; i++)
      x->x_myvec[2 + i] = (t_int*)sp[i+1]->s_vec;

  dsp_addv(shmsrc_tilde_perform, x->x_num_outlets + 2, (t_int *)x->x_myvec);
}

static void shmsrc_tilde_free(t_shmsrc_tilde *x)
{
  shmdata_any_reader_close (x->x_reader);
  t_shmsrc_tilde_buf *buf = g_async_queue_try_pop (x->x_audio_queue); 
  while (buf != NULL)
    {
      shmdata_any_reader_free (buf->shm_buf);
      g_free (buf);
      buf = g_async_queue_try_pop (x->x_audio_queue); 
    }
  g_async_queue_unref (x->x_audio_queue);
  if (x->x_myvec)
    t_freebytes(x->x_myvec, sizeof(t_int *) * (x->x_num_outlets + 3));
  
}



static void *shmsrc_tilde_new(t_symbol *s, t_floatarg num_outlets)
{
  t_shmsrc_tilde *x = (t_shmsrc_tilde *)pd_new(shmsrc_tilde_class);

  x->x_audio_queue = g_async_queue_new();

  //num inlet is also considered as the number of channels
  if (num_outlets < 1)
    x->x_num_outlets = 1;
  else
    x->x_num_outlets = num_outlets;

  x->x_f = 0;
  x->x_init_date = clock_getlogicaltime();
  x->x_reader = NULL;
  x->x_pd_samplerate = -1;
  x->x_current_audio_buf = NULL;

  sprintf (x->x_shmdata_prefix, "/tmp/pd_");
  sprintf (x->x_shmdata_name, "%s",s->s_name);
  sprintf (x->x_shmdata_path, "%s%s",
	   x->x_shmdata_prefix,
	   x->x_shmdata_name);

  int i;
  for (i = 0; i < x->x_num_outlets; i++)
    outlet_new(&x->x_obj, &s_signal);

  x->x_myvec = (t_int **)t_getbytes(sizeof(t_int *) * (x->x_num_outlets + 3));
  if (!x->x_myvec)
    {
      error("shmsrc~: out of memory");
      return NULL;
    }
  x->x_reader = NULL;
  shmsrc_tilde_reader_restart (x);

  return (x);
}

static void shmsrc_tilde_set_name (t_shmsrc_tilde *x, t_symbol *name)
{
  sprintf (x->x_shmdata_name, "%s", name->s_name);
  sprintf (x->x_shmdata_path, "%s%s", x->x_shmdata_prefix, x->x_shmdata_name);
  shmsrc_tilde_reader_restart (x);
}

static void shmsrc_tilde_set_prefix (t_shmsrc_tilde *x, t_symbol *prefix)
{
  sprintf (x->x_shmdata_prefix, "%s", prefix->s_name);
  sprintf (x->x_shmdata_path, "%s%s", x->x_shmdata_prefix, x->x_shmdata_name);
  shmsrc_tilde_reader_restart (x);
}


void shmsrc_tilde_setup(void)
{
  shmsrc_tilde_class = class_new(gensym("shmsrc~"), 
				 (t_newmethod)shmsrc_tilde_new, 
				 (t_method)shmsrc_tilde_free,
				 sizeof(t_shmsrc_tilde), 
				 0, 
				 A_DEFSYM, 
				 A_DEFFLOAT, 
				 A_NULL);
  /* this is magic to declare that the leftmost, "main" inlet
     takes signals; other signal inlets are done differently... */
  CLASS_MAINSIGNALIN(shmsrc_tilde_class, t_shmsrc_tilde, x_f);
  /* here we tell Pd about the "dsp" method, which is called back
     when DSP is turned on. */
  class_addmethod(shmsrc_tilde_class, (t_method)shmsrc_tilde_dsp, gensym("dsp"), 0);
  class_addmethod(shmsrc_tilde_class, (t_method)shmsrc_tilde_set_name, gensym("set_name"), A_DEFSYM, 0);
  class_addmethod(shmsrc_tilde_class, (t_method)shmsrc_tilde_set_prefix, gensym("set_prefix"), A_DEFSYM, 0);

}
