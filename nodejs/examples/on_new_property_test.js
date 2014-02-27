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

switcher_addon.register_signal_callback(function (qname, qsignal, pvalue){
    console.log('...SIGNAL...: ', qname, ' ', qsignal, ' ', pvalue);

    if (qsignal == "on-quiddity-created")
    {
	console.log(switcher_addon.subscribe_to_signal (pvalue[0], "on-new-property"));
	console.log(switcher_addon.subscribe_to_signal (pvalue[0], "on-property-removed"));
	console.log(switcher_addon.subscribe_to_signal (pvalue[0], "on-new-method"));
	console.log(switcher_addon.subscribe_to_signal (pvalue[0], "on-method-removed"));
    }

});

//signal info
console.log ("------------ signals ------------");
console.log (switcher_addon.create("videotestsrc","vidtest"));

setTimeout (function () {console.log (switcher_addon.set_property_value ("vidtest","started", "true"))}, 1000);

