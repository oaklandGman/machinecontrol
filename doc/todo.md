# my todo list

little things:

Figure out limit switches
    a. they sort of work, might be interference issue
Finish auto calibrate routine

move limit switches and estop to IO expander, maybe arduino nano?


Done:
save / load config file to flash
get tabs working on client (status, configure)
update newly connected websocket client with running parameters
Program / script list is clickable


Big things:

1. interface for editing json scripts, although /edit works
2. drop-down function selection (manual, auto, run program)
3. implement async wifimanager

Done:
Save / load config file to sd card
Move all hosted files over to sd card
Get micro-sd interface working
Scripting method for scripts - using json
    a. json scripting working, no easy client side editor yet
    b. only very basic feature set implemented
    c. added dual speed stroke mode!
    d. added loop command
