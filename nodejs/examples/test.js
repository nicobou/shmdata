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
    console.log('.....log message: ', msg);
});

switcher_addon.register_prop_callback(function (qname, qprop, pvalue){
    console.log('...PROP...: ', qname, ' ', qprop, ' ', pvalue);
});

switcher_addon.register_signal_callback(function (qname, qprop, pvalue){
    console.log('...SIGNAL...: ', qname, ' ', qprop, ' ', pvalue);
});

//signal info
console.log ("------------ signals ------------");
switcher_addon.create("audiotestsrc","sigsubtest");
console.log (switcher_addon.get_signals_description ("sigsubtest"));
console.log ("--");
console.log (switcher_addon.get_signal_description ("sigsubtest", "on-new-shmdata-writer"));
console.log ("--");
console.log (switcher_addon.get_signals_description_by_class ("audiotestsrc"));
console.log ("--");
console.log (switcher_addon.get_signal_description_by_class ("audiotestsrc", "on-new-shmdata-writer"));
console.log ("------------ fin signals ------------");


switcher_addon.create("audiotestsrc","propsubtest");
switcher_addon.subscribe_to_property ("propsubtest", "freq");
switcher_addon.set_property_value("propsubtest", "freq", "111");
switcher_addon.set_property_value("propsubtest", "freq", "222");
console.log(switcher_addon.list_subscribed_properties ());
switcher_addon.unsubscribe_to_property ("propsubtest", "freq");
switcher_addon.set_property_value("propsubtest", "freq", "333");


 //classes info (json)
 console.log('------------- classes info ---------------');
 console.log(switcher_addon.get_classes_doc () );
 console.log('------------- end classes info------------');

 //audiotestsrc introspection (json)
 console.log('------------- audiotestsrc class ---------------');
 console.log('\n\n** class doc:\n', 
             switcher_addon.get_class_doc ("audiotestsrc") );
 console.log('\n\n** prop doc:\n', 
             switcher_addon.get_properties_description_by_class ("audiotestsrc") );
 console.log('\n\n** prop \"audiotestsrc/freq\" doc:\n', 
             switcher_addon.get_property_description_by_class ("audiotestsrc",
          						      "freq") );
 console.log('\n\n** rtpsession methods description:\n', 
             switcher_addon.get_methods_description_by_class("rtpsession"));
 console.log('\n\n** rtpsession method description:\n', 
             switcher_addon.get_method_description_by_class("rtpsession", "add_destination"));
 console.log('------------- end audiotestsrc class info-------');


 //creation without a name
 console.log('\n\n---- unamed create returned:', 
             switcher_addon.create("audiotestsrc") );
 //creation with a name
 console.log('\n\n---- named create returned:', 
             switcher_addon.create("audiotestsrc","myaudiotest"));


 //properties
 console.log('\n\n---- get myaudiotest property returned:', 
             switcher_addon.get_property_value("myaudiotest", "freq"));

 console.log('\n\n---- set myaudiotest property to 111 returned:', 
             switcher_addon.set_property_value("myaudiotest", "freq", "111"));

 console.log('\n\n---- get myaudiotest property returned:', 
             switcher_addon.get_property_value("myaudiotest", "freq"));

 console.log('\n\n---- get property description returned:', 
             switcher_addon.get_property_description("myaudiotest", "freq"));

 console.log('\n\n---- get properties description returned:', 
             switcher_addon.get_properties_description("myaudiotest"));

 console.log('\n\n---- get quiddity description returned:', 
             switcher_addon.get_quiddity_description("myaudiotest"));



 //remove "myaudiotest
 console.log('\n\n---- remove myaudiotest returned:', 
             switcher_addon.remove("myaudiotest"));

 //creation with a name
 console.log('\n\n---- named create returned:', 
             switcher_addon.create("rtpsession","rtptest"));

 console.log('\n\n---- get method description returned:', 
             switcher_addon.get_method_description("rtptest", "add_destination"));

 console.log('\n\n---- get methods description returned:', 
             switcher_addon.get_methods_description("rtptest"));

 //print existing quiddities
 console.log('\n\n---- get quiddities description:', 
             switcher_addon.get_quiddities_description());

//invoke
console.log('\n\n---- invoke rtptest returned:', 
  	    switcher_addon.invoke("rtptest", "add_destination", ["local","localhost"]));

 //remove 
 console.log('\n\n---- remove rtptest returned:', 
  	    switcher_addon.remove("rtptest"));

