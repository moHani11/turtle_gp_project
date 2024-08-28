### note: to run the code from multiple devices
> change the master ip and the device ip at the begining of the spawning script to the master host url and your  device IP

## ON the master device run:
- roscore (in its own terminal)
- rosrun turtlesim turtlesim_node (in its own terminal)
- rosrun turtlesim_project game_logic.py (in its own terminal)
- rosrun turtlesim_project spawn.py _master:=yes (in its own terminal to control a turtle)

## ON other devices run:
- rosrun turtlesim_project spawn.py (in its own terminal to control a turtle)
