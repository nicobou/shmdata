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

// switcher_addon.register_log_callback(function (msg){
//     console.log('.....log message: ', msg);
// });

// switcher_addon.register_prop_callback(function (qname, qprop, pvalue){
//     console.log('...PROP...: ', qname, ' ', qprop, ' ', pvalue);
// });

// switcher_addon.register_signal_callback(function (qname, qprop, pvalue){
//     console.log('...SIGNAL...: ', qname, ' ', qprop, ' ', pvalue);
// });

//signal info
console.log ("------------ signals ------------");
console.log (switcher_addon.create("audiotestsrc","sigsubtest"));

console.log (switcher_addon.invoke("sigsubtest","start", []));


setTimeout (function () {console.log ("truyc")}, 1000000);
