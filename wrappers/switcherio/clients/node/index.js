/*
 * This file is part of switcher python wrapper.
 *
 * libswitcher is free software; you can redistribute it and/or
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

const io = require('socket.io-client');
const socket = io.connect('ws://localhost:8000', { reconnect: true });

const ack = (err, res) => { console.log(err, res) }

socket.emit('switcher.name', ack)
socket.emit('switcher.version', ack)
socket.emit('quiddity.create', "dummy", ack)
socket.emit('quiddity.create', "videotestsrc", "vid1", ack)
socket.emit('quiddity.create', "glfwin", ack)
socket.emit('nickname.get', 3, ack)
socket.emit('nickname.set', 3, "win", ack)
const bundle = {
	"bundle": {
		"testBundle": {
			"pipeline": "dummy name=Test",
			"doc": {
		    "long_name": "Test",
		    "category": "test",
		    "tags": "writer",
		    "description": "Test"
			}
		}
	}
}
socket.emit('switcher.bundles', bundle, ack)
socket.emit('switcher.kinds', ack)
socket.emit('switcher.log', "debug", "client: some client-side debugging text", ack)
socket.emit('switcher.quiddities', ack)
socket.emit('property.get', 2, "started", ack)
socket.emit('property.set', 2, "started", true, ack)
socket.emit('quiddity.connect', 3, 2, ack)
socket.emit('quiddity.delete', 1, ack)
socket.emit('quiddity.create', "methodquid", ack)
socket.emit('quiddity.invoke', 4, 'hello', ['Albert Camus'], ack)
