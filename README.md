# aurova_reactives
This is a metapackage that contains different packages that perform safety-oriented reactive stop algorithms for mobile robotics. Compiling this metapackage into ROS will compile all the packages at once. This metapackage is grouped as a project for eclipse C++. Each package contains a "packagename_doxygen_config" configuration file for generate doxygen documentation. The packages contained in this metapackage are:

**reactive_hokuyo**
This package contains a node that, as input, reads the topics /estimated_ackermann_state of type ackermann_msgs::AckermannDriveStamped, and /scan of type sensor_msgs::LaserScan. This node detects obstacles within the range of the laser, and returns the distance to it. The node output is published in the topics /front_obstacle_distance of type std_msgs::Float32, and /pointcloud of type sensor_msgs::PointCloud2.

**reactive_velodyne**
This package contains a node that, as input, reads the topics /estimated_ackermann_state of type ackermann_msgs::AckermannDriveStamped, and /velodyne_points of type sensor_msgs::PointCloud2. This node detects obstacles within the range of the laser, and returns the distance to it. The node output is published in the topics /front_obstacle_distance of type std_msgs::Float32, and /pointcloud of type sensor_msgs::PointCloud2.

**velocity_recommender**
This package contains a node that, as input, read the outgoing topics from the rest of the packages that make up the metapackage. As a result, it returns a recommended speed according to the obstacles detected, both forward and backward. The node output is published in the topics /forward_recommended_velocity of type std_msgs::Float32, and /backward_recommended_velocity of type std_msgs::Float32.
