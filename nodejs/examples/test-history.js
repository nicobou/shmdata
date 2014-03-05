/*
 * Copyright (C) 2012-2013 Nicolas Bouillot (http://www.nicolasbouillot.net)
 *
 * This file is part of node-switcher.
 *
 * node-switcher is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * node-switcher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with node-switcher.  If not, see <http://www.gnu.org/licenses/>.
 */

var switcher_addon = require('../build/Release/switcher_addon');

//please quit switcher properly
process.on('exit', function () {
    switcher_addon.close();
    console.log('About to exit.');
});
process.on('SIGINT', function () {
    switcher_addon.close();
    console.log('Got SIGINT.  About to exit.');
    process.exit(0);
});

switcher_addon.register_log_callback(function (msg){
    //console.log('.....log message: ', msg);
});

switcher_addon.register_prop_callback(function (qname, qprop, pvalue){
    //console.log('...PROP...: ', qname, ' ', qprop, ' ', pvalue);
});

switcher_addon.register_signal_callback(function (qname, qprop, pvalue){
    //console.log('...SIGNAL...: ', qname, ' ', qprop, ' ', pvalue);
});


switcher_addon.create("videotestsrc","vid");
switcher_addon.create("videosink","win");
switcher_addon.invoke ("win","connect",["/tmp/switcher_nodeserver_vid_video"])

switcher_addon.save_history ("histo.switcher");

 console.log('\n\n---- Quiddities after initial commands:', 
              switcher_addon.get_quiddities_description());

switcher_addon.load_history_from_current_state ("histo2.switcher");

console.log('\n\n---- Quiddities after load_history_from_current_state:', 
            switcher_addon.get_quiddities_description());

switcher_addon.load_history_from_scratch ("histo.switcher");

switcher_addon.save_history ("histo.switcher");

console.log('\n\n---- Quiddities after load_history_from_scratch:', 
            switcher_addon.get_quiddities_description());
