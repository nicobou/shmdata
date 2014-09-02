//
//  AppDelegate.h
//  Switcher
//
//  Created by Mike Wozniewski on 13-06-21.
//  Copyright (c) 2013 SAT. All rights reserved.
//

#import <Cocoa/Cocoa.h>
#import <Foundation/NSFileHandle.h>

@ interface AppDelegate:NSObject < NSApplicationDelegate > {
  NSPipe *pipe;
  NSFileHandle *pipeReadHandle;
  IBOutlet NSTextView *textView;
}

@property(assign)
IBOutlet NSWindow *
window;

@end
