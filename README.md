The telepresence node
===


**This package takes a ROS image stream, and accepts a service that sends out a
move_base goal at the location of the object that was 'clicked' through the given camera pixel.**

The package includes two nodes:
1. `image_filter`
> This node syncs the Image and the CameraInfo messages.
> This is sometimes needed if the network is slow.
> 
> It also undistorts fisheye lens images, if the `undistort` arg is set in the launch file.
2. `telepresence`
> This node handles the geometric transformations needed to find the goal in the image.
> 
> It needs a `nav_msgs/OccupancyGrid` of the map and `tf` data of the camera.
> It also publishes the goal to tf.
> 
> If a depth cam is available, it can also subscribe to a `sensor_msgs/Pointcloud2` point_map topic
> to find obstacles.
> 
> When the node finds and obstacle, it steps back a configurable distance so the goal
> doesn't make the robot stuck in a costmap.
> 
> A maximum distance is set that is hit if there is no closer obstacle found.
> 
> The search granuality, and the robot radius also have to be set.
> 
> An `rviz` visualization of the ray cast can also be enabled.


The below picture contains additional information about the communication of the nodes.
![publications and subscriptions of the telepresence package](/telepresence_comms.png "publications and subscriptions of the telepresence package")

---

### Browser interface
> There is also a browser interface implemented, that can be toggled in the launch file.
> It streams the image from the telepresence node that can be clicked to send a goal to a robot.
> 
> It can send rotation commands on the `cmd_vel` topic.
> 
> It can also cancel the current move_base goal.
