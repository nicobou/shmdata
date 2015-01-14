/*
 * This file is part of switcher-jack.
 *
 * switcher-myplugin is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include <glib.h>
#include "switcher/std2.hpp"
#include "./jack-client.hpp"

namespace switcher {
JackClient::JackClient(const char *name) :
    client_ (jack_client_open(name,
                              JackNoStartServer,
                              &status_,
                              nullptr /* server name */),
             &jack_client_close)
{
  if ((client_ == NULL) && !(status_ & JackServerFailed)) {
    g_warning("jack_client_open() failed, "
              "status = 0x%2.0x", status_);
    return;
  }
  jack_on_shutdown (client_.get(), &JackClient::on_jack_shutdown, this);
    // if (status_ & JackServerStarted) {
  //   g_warning("JACK server started\n");
  // }
  // if (status_ & JackNameNotUnique) {
    // client_name = jack_get_client_name(client);
    // fprintf (stderr, "unique name `%s' assigned\n", client_name);
  // }
  sample_rate_ = jack_get_sample_rate(client_.get());
  buffer_size_ = jack_get_buffer_size(client_.get());
  // g_print("sample rate %" PRIu32 "\n", sample_rate_);
  // g_print("buffer size %" PRIu32 " \n", jack_get_buffer_size(client_.get()));
  jack_set_process_callback(client_.get(), jack_process, this);
  jack_activate(client_.get());
}

int JackClient::jack_process(jack_nframes_t nframes, void *arg)
{
  JackClient *context = static_cast<JackClient *>(arg);
  if (nullptr != context->user_cb_)
    return context->user_cb_(nframes, context->user_cb_arg_);
  g_print("no jack process callback set by user\n");
  return 0;
}


bool JackClient::safe_bool_idiom() const {
  return static_cast<bool>(client_);
}

jack_nframes_t JackClient::get_sample_rate() const{
  return sample_rate_;
}

jack_nframes_t JackClient::get_buffer_size() const{
  return buffer_size_;
}

void JackClient::on_jack_shutdown (void */*arg*/)
{
  g_warning("jack shut down");
  // TODO invalidate client_
}

jack_client_t *JackClient::get_raw(){
  return client_.get();
}

void JackClient::set_jack_process_callback(JackProcessCallback cb, void *arg){
  user_cb_ = cb;
  user_cb_arg_ = arg;
}

}  // namespace switcher
