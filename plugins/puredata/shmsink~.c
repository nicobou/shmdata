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
#include "shmdata/any-data-writer.h"

#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

/* ------------------------ shmsink~ ----------------------------- */


static t_class *shmsink_class;

#define SHMSINK_NUM_BUF_BEFORE_OVERWRITE 16

typedef struct _shmsink
{
  t_object x_obj; 	/* obligatory header */
  shmdata_any_writer_t *x_writer;
  double x_init_date;
  int x_num_inlets;
  int x_cur_buf; //the buffer to use for writing to the shmdata
  void *x_buf[SHMSINK_NUM_BUF_BEFORE_OVERWRITE];
  unsigned long long x_offset;
  t_float x_f;    	/* place to hold inlet's value if it's set by message */
} t_shmsink;

static t_int *shmsink_perform(t_int *w)
{
  t_shmsink *x = ( t_shmsink *)(w[1]);
  int n = (int)(w[2]);
  t_float *in[256]; //MAX AUDIO INLETS
  int i,j; 
  for (i = 0; i < x->x_num_inlets; i++)
    in[i] = (t_float *)(w[3 + i]); 

  unsigned long long offset = x->x_offset;
  x->x_offset += n; 
  int buf_size = n*sizeof(t_float);

  for (i =0; i < x->x_num_inlets;i++)
    for (j=0; j < n; j++)
      memcpy(x->x_buf[x->x_cur_buf] + i + j * sizeof (t_float), (void *)(in[i] + j * sizeof (t_float)), sizeof(t_float)); 
  //  memcpy(x->x_buf[x->x_cur_buf], (void *)(in), buf_size);

  shmdata_any_writer_push_data (x->x_writer, 
				x->x_buf[x->x_cur_buf],  
				buf_size * x->x_num_inlets,  
				(unsigned long long) (clock_gettimesince(x->x_init_date)* 1000000), //clock  
				NULL, 
				NULL); 

  x->x_cur_buf +=1;
  x->x_cur_buf %=SHMSINK_NUM_BUF_BEFORE_OVERWRITE;
  
  return (w+4);
}

/* called to start DSP.  Here we call Pd back to add our perform
   routine to a linear callback list which Pd in turn calls to grind
   out the samples. */
static void shmsink_dsp(t_shmsink *x, t_signal **sp)
{
  dsp_add(shmsink_perform, 3, x, sp[0]->s_n, sp[0]->s_vec);
}
static void shmsink_free(t_shmsink *x)
{
  int i;
  for (i=0; i<SHMSINK_NUM_BUF_BEFORE_OVERWRITE; i++)
    {
      free (x->x_buf[i]);
    }
  shmdata_any_writer_close (x->x_writer);
}

static void *shmsink_new(t_floatarg num_inlets)
{
  t_shmsink *x = (t_shmsink *)pd_new(shmsink_class);
  if (num_inlets < 1)
    x->x_num_inlets = 1;
  else
    x->x_num_inlets = num_inlets;

  x->x_f = 0;
  x->x_offset = 0;
  x->x_init_date = clock_getlogicaltime();
  x->x_writer = shmdata_any_writer_init ();
  x->x_cur_buf = 0;    
  int i;
  for (i=0; i<SHMSINK_NUM_BUF_BEFORE_OVERWRITE; i++)
    {
      x->x_buf[i] = malloc (x->x_num_inlets * 4096 * sizeof (t_float));
    }

  for (i = 1; i < x->x_num_inlets; i++)
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);

  /* if (verbose) */
  /*    shmdata_any_writer_set_debug (s_app.writer, SHMDATA_ENABLE_DEBUG); */
  /*  else */
  /*    shmdata_any_writer_set_debug (s_app.writer, SHMDATA_DISABLE_DEBUG); */
  shmdata_any_writer_set_data_type (x->x_writer, 
				    "audio/x-raw-float, endianness=1234, width=32, rate=44100, channels=2");
  shmdata_any_writer_set_path (x->x_writer,"/tmp/pd-shmdata-test");
  shmdata_any_writer_start (x->x_writer);
  return (x);
}

/* this routine, which must have exactly this name (with the "~" replaced
   by "_tilde) is called when the code is first loaded, and tells Pd how
   to build the "class". */
void shmsink_tilde_setup(void)
{
  shmsink_class = class_new(gensym("shmsink~"), 
			    (t_newmethod)shmsink_new, 
			    (t_method)shmsink_free,
			    sizeof(t_shmsink), 
			    0, 
			    A_DEFFLOAT, 
			    A_NULL);
  /* this is magic to declare that the leftmost, "main" inlet
     takes signals; other signal inlets are done differently... */
  CLASS_MAINSIGNALIN(shmsink_class, t_shmsink, x_f);
  /* here we tell Pd about the "dsp" method, which is called back
     when DSP is turned on. */
  class_addmethod(shmsink_class, (t_method)shmsink_dsp, gensym("dsp"), 0);
}
