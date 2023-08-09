# Turtlesim_project
Turtlesim_project use 3 nodes:

1. The turtlesim_node from the turtlesim package
2. Turtle_controller node to control the turtle (named “turtle1”) which is already existing in the turtlesim_node
3. Turtle_spawner node to spawn turtles on the window, and to manage which turtle is still “alive” (on the screen)

Interfaces:

1. Turtle.msg and TurtleArray.msg to send the list of turtles (name + coordinates) on the /alive_turtles topic
2. CatchTurtle.srv to send the name of the turtle which was caught

Parameters:

/turtle_controller:
catch_closest_turtle_first
use_sim_time
/turtle_spawner:
spawn_frequency
turtle_name_prefix
use_sim_time
