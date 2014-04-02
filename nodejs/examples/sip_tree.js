/*
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

var switcher_addon = require('../.libs/switcher');

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

switcher_addon.register_signal_callback(function (qname, qsignal, pvalue){
    //console.log('...SIGNAL...: ', qname, ' ', qprop, ' ', pvalue);
    if (qsignal == "on-tree-grafted" || qsignal == "on-tree-pruned" )
    {
	console.log('info_tree: ', qname, ' ', qsignal, ' ', pvalue[0], 
		    switcher_addon.get_info (qname, pvalue[0]));
    }
});

console.log (switcher_addon.create("sip","sipquid"));

console.log(switcher_addon.subscribe_to_signal ("sipquid", "on-tree-grafted"));
console.log(switcher_addon.subscribe_to_signal ("sipquid", "on-tree-pruned"));

console.log (switcher_addon.invoke("sipquid","register", ["1010", 
							  "10.10.30.115",
							  "1234"]));


setTimeout (function () {console.log ("- the end -")}, 1000000);
