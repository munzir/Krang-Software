This is a collection of stuff for publishing states from joystick-type devices on ach channels.  
For basic library calls to read these devices, see js.c/h, and snbasic. 


Files:
js.c/h - basic wrapper for polling a joystick (logitech gamepad)
js_smm.c/h - shared library containing calls for both jachd and snachd, the joystick and spacenav publishers
jachd.c - the joystick publisher
snachd.c - the spacenav publisher 
jach_listen_and_print.c - example program for how to listen to a joystick channel and unpack it


Usage:
<from a terminal>
$ ./jachd -C (the first time)
 -or-
$ ./jachd (if the channel already exists)

<from another terminal>
$ ./jach_listen_and_print --channel="joystick-data"

** If you want to generalize the functionality of this daemon (because you're neil or one of 
his disciples), the place to look is js_smm.c  These calls could quite reasonably moved to 
somatic/motor*, however since the functionality is specific to joystick messages and devices,
I think it makes the most sense to keep here.