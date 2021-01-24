# Turtle Rect
* A package that causes a turtlesim turtle to follow a rectangle.
# Example Usage
```
roslaunch trect trect.launch
rosservice call start "msg:
     fromX: 2.0
     fromY: 3.0
     width: 4.0
     height: 5.0"
```
![Demonstration](<demo/turtle_rect_start.gif>)