## How to use the timecode quiddities package

Each of the quiddities having to do with timecode has little use by themselves but can be used in conjuction to set a 
delay line on generic shmdata. We'll briefly go over each one to understand how they can be used.
We can note that they rely on the [libltc](https://github.com/x42/libltc) library.

#### Generating timecode
The  _ltcsource_ quiddity generates a timecode and sends it in a raw audio shmdata. There are two mutually exclusive 
ways of cadencing 
the timecode generation:
* with the sample rate of the jack server currently running
* with an external audio source (can be from SIP, local or from an external audio card).
The detection is automatic so the rate does not have to be specified. This is the only shmdata input of the quiddity .

After that, there are three options available for the generation:
* time reference: absolute means we take the current GMT time as a start reference when starting the source, relative
 starts at 0 when we start the source.
* FPS: number of ltc frames per second, can be 24, 25 or 30 because of a "limitation" of libltc
* delay: fixed delay in number of frames

This will generate a regular audio shmdata with appropriate caps that can be used as any other audio shmdata. But we 
have other plans for it!

#### Measuring the delay between two LTC timecodes shmdata
And that's where the plan leads us. For scenography or syncing purposes we might want to know the difference between 
two timecodes. This is exactly what the _ltcdiff_ does. It takes two audio/x-raw as input, detects if they contain 
any ltc frames and generates a shmdata containing the absolute difference in milliseconds. We don't pay attention to 
which one is late and which one is early because we consider that, for our usage, the clocks will be delayed enough to
 eliminate the possible ambiguity. 
 
If an audio signal does not contain any ltc frame, the detected timecode will stay at 0 , there is no mechanism to 
know preemptively that we receive ltc from a shmdata. So it is the responsibility of the use to make sure a correct 
audio ltc shmdata is connected to the input of the quiddity. However there are two read-only properties that will 
periodically show the evolution of the detected timecode: "first_timecode" and "second_timecode". 

#### Using timecode difference to delay a shmdata 
This third quiddity is not directly in the timecode plugin but can make use of it. The _shmdelay_ quiddity's goal is 
to delay an input shmdata either by a duration in milliseconds controlled by a "time_delay" property or by the output
 of an _ltcdiff_ quiddity. If an _ltcdiff_ output is connected to the _shmdelay_ input, the delay property will be 
 ignored and will be read-only.