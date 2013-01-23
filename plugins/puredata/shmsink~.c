/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "m_pd.h"
#include "shmdata/any-data-writer.h"

#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

/* ------------------------ shmsink~ ----------------------------- */


static t_class *shmsink_tilde_class;

#define SHMSINK_TILDE_NUM_BUF_BEFORE_OVERWRITE 16
#define SHMSINK_TILDE_MAX_VECTOR_SIZE 4096

typedef struct _shmsink
{
  t_object x_obj;
  shmdata_any_writer_t *x_writer;
  double x_init_date;
  int x_num_inlets;
  int x_cur_buf;                                 /*the buffer to use for writing to the shmdata*/
  void *x_buf[SHMSINK_TILDE_NUM_BUF_BEFORE_OVERWRITE];
  unsigned long long x_offset;
  long x_samplerate;                             /* samplerate we're running at */
  char x_caps[256];
  char x_shmdata_path[512];
  char x_shmdata_prefix[256];
  char x_shmdata_name[256];
  t_int **x_myvec;                               /* vector we pass on in the DSP routine */
  t_float x_f;    	                         /* place to hold inlet's value if it's set by message */
} t_shmsink_tilde;

static t_int *shmsink_tilde_perform(t_int *w)
{
  t_shmsink_tilde *x = ( t_shmsink_tilde *)(w[1]);
  int n = (int)(w[2]);
  t_float *in[x->x_num_inlets];
  int i,j; 
  for (i = 0; i < x->x_num_inlets; i++)
    in[i] = (t_float *)(w[3 + i]); 

  x->x_offset += n; 
  int buf_size = n*sizeof(t_float);
  
  //interleaving channels
  for (i =0; i < x->x_num_inlets;i++) 
    {
      t_float *input = in[i];
      t_float *output = x->x_buf[x->x_cur_buf];
      output += i;

      for (j=0; j < n; j++)  
	{
	  *output = *input;
	  output += x->x_num_inlets;
	  input ++;
	}
    }

  //printf ("shmsink: buf size %d\n", buf_size * x->x_num_inlets);

  shmdata_any_writer_push_data (x->x_writer, 
				x->x_buf[x->x_cur_buf],  
				buf_size * x->x_num_inlets,  
				(unsigned long long) (clock_gettimesince(x->x_init_date)* 1000000), //clock  
				NULL, 
				NULL); 
  x->x_cur_buf +=1;
  x->x_cur_buf %=SHMSINK_TILDE_NUM_BUF_BEFORE_OVERWRITE;
  return (w + 3 + x->x_num_inlets);
}

static void shmsink_tilde_writer_restart (t_shmsink_tilde *x)
{
  shmdata_any_writer_close (x->x_writer);
  x->x_writer = shmdata_any_writer_init ();
  sprintf (x->x_caps, "audio/x-raw-float, endianness=1234, width=32, rate=%d, channels=%d",
	   (int)x->x_samplerate,
	   x->x_num_inlets);
  shmdata_any_writer_set_data_type (x->x_writer, x->x_caps);
  if (shmdata_any_writer_set_path (x->x_writer,x->x_shmdata_path) == 0)
    {
      remove (x->x_shmdata_path);
      shmdata_any_writer_set_path (x->x_writer,x->x_shmdata_path);
    }
  shmdata_any_writer_start (x->x_writer);
}

static void shmsink_tilde_dsp(t_shmsink_tilde *x, t_signal **sp)
{
  x->x_myvec[0] = (t_int*)x;
  x->x_myvec[1] = (t_int*)sp[0]->s_n;
 
  if (x->x_samplerate != sp[0]->s_sr)
    {
      x->x_samplerate = sp[0]->s_sr;
      /* if (verbose) */
      /*    shmdata_any_writer_set_debug (s_app.writer, SHMDATA_ENABLE_DEBUG); */
      /*  else */
      /*    shmdata_any_writer_set_debug (s_app.writer, SHMDATA_DISABLE_DEBUG); */
      shmsink_tilde_writer_restart (x);
   }

  int i;
  for (i = 0; i < x->x_num_inlets; i++)
      x->x_myvec[2 + i] = (t_int*)sp[i]->s_vec;

  if (SHMSINK_TILDE_MAX_VECTOR_SIZE < sp[0]->s_n)
    error("shmsink~: signal vector size too large (max %d)", SHMSINK_TILDE_MAX_VECTOR_SIZE);
  else
    dsp_addv(shmsink_tilde_perform, x->x_num_inlets + 2, (t_int *)x->x_myvec);
}


static void shmsink_tilde_free(t_shmsink_tilde *x)
{
  int i;
  for (i=0; i<SHMSINK_TILDE_NUM_BUF_BEFORE_OVERWRITE; i++)
    {
      free (x->x_buf[i]);
    }
  shmdata_any_writer_close (x->x_writer);

  if (x->x_myvec)
    t_freebytes(x->x_myvec, sizeof(t_int *) * (x->x_num_inlets + 3));

}

static void *shmsink_tilde_new(t_symbol *s, t_floatarg num_inlets)
{
  t_shmsink_tilde *x = (t_shmsink_tilde *)pd_new(shmsink_tilde_class);

  //num inlet is also considered as the number of channels
  if (num_inlets < 1)
    x->x_num_inlets = 1;
  else
    x->x_num_inlets = num_inlets;

  x->x_f = 0;
  x->x_offset = 0;
  x->x_init_date = clock_getlogicaltime();
  x->x_writer = NULL;
  x->x_cur_buf = 0;    
  x->x_samplerate = 0;
  sprintf (x->x_shmdata_prefix, "/tmp/pd_");
  sprintf (x->x_shmdata_name, "%s",s->s_name);
  sprintf (x->x_shmdata_path, "%s%s",
	   x->x_shmdata_prefix,
	   x->x_shmdata_name);
  int i;
  for (i=0; i< SHMSINK_TILDE_NUM_BUF_BEFORE_OVERWRITE; i++)
      x->x_buf[i] = malloc (x->x_num_inlets * SHMSINK_TILDE_MAX_VECTOR_SIZE * sizeof (t_float));

  for (i = 1; i < x->x_num_inlets; i++)
    inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);

  x->x_myvec = (t_int **)t_getbytes(sizeof(t_int *) * (x->x_num_inlets + 3));
  if (!x->x_myvec)
    {
      error("shmsink~: out of memory");
      return NULL;
    }

  return (x);
}


static void shmsink_tilde_set_name (t_shmsink_tilde *x, t_symbol *name)
{
  sprintf (x->x_shmdata_name, "%s", name->s_name);
  sprintf (x->x_shmdata_path, "%s%s", x->x_shmdata_prefix, x->x_shmdata_name);
  shmsink_tilde_writer_restart (x);
}

static void shmsink_tilde_set_prefix (t_shmsink_tilde *x, t_symbol *prefix)
{
  sprintf (x->x_shmdata_prefix, "%s", prefix->s_name);
  sprintf (x->x_shmdata_path, "%s%s", x->x_shmdata_prefix, x->x_shmdata_name);
  shmsink_tilde_writer_restart (x);
}


void shmsink_tilde_setup(void)
{
  shmsink_tilde_class = class_new(gensym("shmsink~"), 
			    (t_newmethod)shmsink_tilde_new, 
			    (t_method)shmsink_tilde_free,
			    sizeof(t_shmsink_tilde), 
			    0, 
			    A_DEFSYM, 
			    A_DEFFLOAT, 
			    A_NULL);
  /* this is magic to declare that the leftmost, "main" inlet
     takes signals; other signal inlets are done differently... */
  CLASS_MAINSIGNALIN(shmsink_tilde_class, t_shmsink_tilde, x_f);
  /* here we tell Pd about the "dsp" method, which is called back
     when DSP is turned on. */
  class_addmethod(shmsink_tilde_class, (t_method)shmsink_tilde_dsp, gensym("dsp"), 0);
  class_addmethod(shmsink_tilde_class, (t_method)shmsink_tilde_set_name, gensym("set_name"), A_DEFSYM, 0);
  class_addmethod(shmsink_tilde_class, (t_method)shmsink_tilde_set_prefix, gensym("set_prefix"), A_DEFSYM, 0);
}
