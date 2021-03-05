# nurtlesim
* A package that simulates robot kinematics and sensor with Gaussian noise.

# Example Usage
```
roslaunch nurtlesim tube_world.launch use_odom:=true
```
Launchfile `tube_world.launch` will control a robot using `turtlebot3_teleop_key`. It will show a fake_odom turtle moved by commanded twist and a turtle with Gaussian noise and uniform random noise in slipping. This file will also adds some obstacles in the world and publish the "ground truth" locations of obstacles with Gaussian noise. When running the file, a path for fake_odom turtle is shown in white line and a path for the true turtle path is shown in green line.   
Sensor information containing distances and angles to observed noise-obstacle is publsihed at 10 Hz. Obstacles at known locations are shown as green tubes; obstacles with added Gaussian noise are shown as red tubes.  
When robot is colliding with an obstacle, it will move along a tangent line between itself (simulated as as sphere) and the obstacles. 

# Configuration
`wheel_base` - distance between wheels of robot  
`wheel_radius` - radius of wheels  
`odom_frame_id` - name of odometer frame   
`body_frame_id` - name of robot frame  
`left_wheel_joint` - name of robot left wheel joint  
`right_wheel_joint` - name of robot right wheel joint  
`left_wheel_joint_slip` - name of robot left wheel joint with noises  
`right_wheel_joint_slip` - name of robot right wheel joint with noises  
`noise_mean` - mean of Gaussian noise for commanded twist   
`noise_variance` - variance of Gaussian noise for commanded twist  
`slip_min` - minimum uniform radom noise ratio for slipping  
`slip_max` - maximum uniform radom noise ratio for slipping   
`x_coords` - x locations of obstacles in world  
`y_coords` - y locations of obstacles in world  
`covariance_matrix_0` - first row of covariance matrix of obstacle lcoations   
`covariance_matrix_1` - second row of covariance matrix of obstacle lcoations  
`obst_radius` - radius of tube obstacle   
`obst_max_dist` - radius of observation area for robot; obstacles within this range will be used to publish sensor information   
`tube_var` - variance of Gaussian noise for obstacles   

