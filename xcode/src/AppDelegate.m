//
//  AppDelegate.m
//  Switcher
//
//  Created by Mike Wozniewski on 13-06-21.
//  Copyright (c) 2013 SAT. All rights reserved.
//

#import "AppDelegate.h"


@implementation AppDelegate

- (void)dealloc
{
    [super dealloc];
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication *)theApplication {
    return YES;
}

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification
{
    // Insert code here to initialize your application
    pipe = [NSPipe pipe];
    pipeReadHandle = [pipe fileHandleForReading];
    dup2([[pipe fileHandleForWriting] fileDescriptor], fileno(stdout));
    
    
    [[NSNotificationCenter defaultCenter] addObserver: self selector: @selector(handleLog:) name: NSFileHandleReadCompletionNotification object: pipeReadHandle];
    [pipeReadHandle readInBackgroundAndNotify];
    
}

- (void)logPrint:(NSString*)s
{
    //[textView setString:[[textView string] stringByAppendingString:s]];
}

- (void) handleLog:(NSNotification *)notification
{
    NSString *str = [[NSString alloc] initWithData: [[notification userInfo] objectForKey: NSFileHandleNotificationDataItem] encoding: NSASCIIStringEncoding] ;

    //[[[textView textStorage] mutableString] appendString: str];
    
    [textView setString:[[textView string] stringByAppendingString:str]];
    [str release];
    
    [[notification object] readInBackgroundAndNotify];
}

@end
