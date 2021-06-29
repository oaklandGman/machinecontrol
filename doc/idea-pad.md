# ideas to maybe implement

1. auto increase speed - bool rampSpeed
    a. starting speed - int motspeed
    b. max speed - int speedMax
    c. strokes per / ramp - calculated automatically (delta speed / reps)

2. auto increase length - bool rampLength
    a. starting length - int motlength
    b. max length - int lengthmax
    c. strokes per / ramp -  calculated automatically (delta length / reps)

3. auto random length - bool rndLength
    a. max length - int rndMax
    b. min length - int motlength
    c. random strokes per - int rndStrokes


UI layout

Tab 1 Debug - fine how it is
Tab 2 Manual Mode - get rid of some of the one time setup things
Tab 3 Config - save/load config, and manual setup stuff
Tab 4 Program - Probably ok?

# Remote control - ESP32 based, uses websocket connection to controller, same as mobile app
Four rotary encoders
  1. Stroke speed
  2. Stroke length / depth
  3. Depth increase / increase delay
  4. Lube amount / frequency
Three buttons
  1. Run / pause
  2. Shoot lube
  3. Enable motors and reset
