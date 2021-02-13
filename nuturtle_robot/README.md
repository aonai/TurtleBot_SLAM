# nuturtle_robot
* A package that runs nodes on the real turtlebot
# Example Usage
On turtlebot, build the worksapce and run
```
roslaunch nuturtle_robot basic_remote.launch robot:=localhost
```
Launchfile `basic_remote.launch` will allow running nodes on the turtlebot locally. Argument `robot` is the hostname of the turtlebot.