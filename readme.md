Unzip the gp_plotter workspace into your catkin workspace
catkin_make
Then just call the .py code as:

rosrun gp_plotter gp_plot.py "my_frame"

Where "my_frame" is the name of the world coordinate (usually "world")

After that the "add_point_gp_service" and "set_prior" will be available.
To use, you need to create a request class (of the AddPoint and SetPrior types, respectively) and then just call the services.

The results are visualized through RVIZ, Marker type visualization of the /mesh_publish topic
