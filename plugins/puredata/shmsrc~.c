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

#include "m_pd.h"
#include <glib.h>
#include <string.h>
#include "shmdata/any-data-reader.h"

#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

/* ------------------------ shmsrc~ ----------------------------- */

/* tilde object to take absolute value. */

static t_class *shmsrc_tilde_class;

typedef struct _shmsrc_tilde
{
  t_object x_obj; 
  shmdata_any_reader_t *x_reader;
  double x_init_date;
  int x_num_outlets;
  unsigned long long x_offset;
  long x_samplerate;               /* samplerate we're running at */
  char x_caps[256];
  char x_shmdata_path[512];
  char x_shmdata_prefix[256];
  char x_shmdata_name[256];
  GAsyncQueue *x_audio_queue;
  GAsyncQueue *x_shmbuf_queue;
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
   
  // void *audio_data = g_malloc0 (data_size);
  //memcpy (audio_data, data, data_size);
    g_async_queue_push (x->x_shmbuf_queue, shmbuf);
    g_async_queue_push (x->x_audio_queue, data);
   
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
  
  //printf ("hehe %d\n",n*x->x_num_outlets*sizeof (t_float));

  t_float *shm_audio_data = (t_float *)g_async_queue_try_pop (x->x_audio_queue);

  if (shm_audio_data != NULL)
    {
      void *shmbuf = g_async_queue_pop (x->x_shmbuf_queue);
      //deinterleaving channels 
      while (n--) 
	for (i=0; i < x->x_num_outlets; i++) //fixme this should be the number of channels in the stream 
	  { 
	    *(out[i]++) = (t_float) *shm_audio_data;  
	    shm_audio_data ++;
	  } 
      //g_free ((void *)shm_audio_data);
      shmdata_any_reader_free (shmbuf);
    }
  else
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
  int i;
  for (i = 0; i < x->x_num_outlets; i++)
      x->x_myvec[2 + i] = (t_int*)sp[i+1]->s_vec;

  dsp_addv(shmsrc_tilde_perform, x->x_num_outlets + 2, (t_int *)x->x_myvec);
}

static void shmsrc_tilde_free(t_shmsrc_tilde *x)
{
  //FIXME clean existing buffers in queue before
  g_async_queue_unref (x->x_audio_queue);
  g_async_queue_unref (x->x_shmbuf_queue);
  if (x->x_myvec)
    t_freebytes(x->x_myvec, sizeof(t_int *) * (x->x_num_outlets + 3));
}



static void *shmsrc_tilde_new(t_symbol *s, t_floatarg num_outlets)
{
  t_shmsrc_tilde *x = (t_shmsrc_tilde *)pd_new(shmsrc_tilde_class);

  x->x_audio_queue = g_async_queue_new();
  x->x_shmbuf_queue = g_async_queue_new();

  //num inlet is also considered as the number of channels
  if (num_outlets < 1)
    x->x_num_outlets = 1;
  else
    x->x_num_outlets = num_outlets;

  x->x_f = 0;
  x->x_offset = 0;
  x->x_init_date = clock_getlogicaltime();
  x->x_reader = NULL;
  x->x_samplerate = 0;
  
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
