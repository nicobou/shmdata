{
  "targets": 
  [
    {
      "target_name": "switcher_addon",
      "cflags": [ "-std=c++0x", '<!@(pkg-config switcher-0.4 --cflags)'],
      "link_settings": {
          'libraries': ['<!@(pkg-config switcher-0.4 --libs)']
         }	   
      ,	  
      "sources": [ "switcher_addon.cpp" ]
  
  }
  ]	      
}
