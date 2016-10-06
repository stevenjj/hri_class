To setup:
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosservice call /spawn 5 5 0 'turtle2'
$ rosservice call /turtle1/teleport_absolute 1.0 1.0 0
$ rosservice call clear
$ rosrun social_tbot social_tbot