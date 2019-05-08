# aurova_reactives
This is a metapackage that contains different packages that perform safety-oriented reactive stop algorithms for mobile robotics. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "packagename_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**reactive_hokuyo**
This package contains a node that, as input, reads the topics /estimated_ackermann_state of type ackermann_msgs::AckermannDriveStamped, and /scan of type sensor_msgs::LaserScan. This node detects obstacles within the range of the laser, and returns the distance to it. The node output is published in the topics /front_obstacle_distance of type std_msgs::Float32, and /pointcloud of type sensor_msgs::PointCloud2.
The following parameters are specified in the cfg file, and can be modified online using rqt_reconfigure.
* ~lateral_safety_margin (default: 0.10): The effective width of the vehicle will be W + 2*margin.
* ~min_obstacle_height (default: 0.30): To filter small obstacles.
* ~euclidean_association_threshold (default: 0.10): To do the clustering.
* ~min_obstacle_radius (default: 0.03): To discard outliers.

**reactive_velodyne**
This package contains a node that, as input, reads the topics /estimated_ackermann_state of type ackermann_msgs::AckermannDriveStamped, and /velodyne_points of type sensor_msgs::PointCloud2. This node detects obstacles within the range of the laser, and returns the distance to it. The node output is published in the topics /front_obstacle_distance of type std_msgs::Float32, and /pointcloud of type sensor_msgs::PointCloud2.

**velocity_recommender**
This package contains a node that, as input, read the outgoing topics from the rest of the packages that make up the metapackage. As a result, it returns a recommended speed according to the obstacles detected, both forward and backward. The node output is published in the topics /forward_recommended_velocity of type std_msgs::Float32, and /backward_recommended_velocity of type std_msgs::Float32.
The following parameters are specified in the cfg file, and can be modified online using rqt_reconfigure.
* ~safety_time (default: 2.0): The recommended velocity depends on the desired time to reach the safety stop point.
* ~safety_distance (default: 0.25): Distance between obstacle and safety stop point.
