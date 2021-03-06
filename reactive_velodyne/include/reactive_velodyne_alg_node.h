// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _reactive_velodyne_alg_node_h_
#define _reactive_velodyne_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "reactive_velodyne_alg.h"
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Float32.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
//#include "iridrivers/exceptions.h"

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class ReactiveVelodyneAlgNode : public algorithm_base::IriBaseAlgorithm<ReactiveVelodyneAlgorithm>
{
private:
  // Constants
  static const float OUT_OF_RANGE_ = 100.0;
  static const float STOP_VEHICLE_ = 0.0;

  // Constant robot hardware constraints
  static const float WHEELBASE_METERS_ = 1.05;
  static const float DISTANCE_FROM_SENSOR_TO_FRONT_ = 0.750;
  static const float DISTANCE_FROM_SENSOR_TO_BACK_ = 0.800;
  static const float X_DISTANCE_FROM_BASE_LINK_TO_SENSOR_ = 0.550;
  static const float SENSOR_HEIGHT_ = 1.100;
  static const float SAFETY_MARGIN_ABOVE_SENSOR_ = 0.5;

  static const float MAX_VEL_ = 1.3;
  static const float TIME_TO_REACH_OBSTACLE_ = 2.0;

  static const float VEHICLE_WIDTH_ = 0.800;
  static const float ABS_MAX_STEERING_DEG_ANGLE = 30.000;
  static const float MIN_OBSTACLE_HEIGHT_ = 0.300;

  // Input
  bool flag_new_velodyne_data_;
  sensor_msgs::PointCloud2 input_cloud_;
  sensor_msgs::PointCloud2 local_copy_of_input_cloud_;
  sensor_msgs::PointCloud2 obstacle_points_;

  // Configurable safety parameters
  float abs_lateral_safety_margin_;

  float z_threshold_;
  float safety_width_;

  float euclidean_association_threshold_;
  float min_obstacle_radius_;

  // Values to compute output
  float closest_front_obstacle_point_;
  float closest_back_obstacle_point_;
  float steering_angle_;

  // [publisher attributes]
  ros::Publisher front_obstacle_distance_publisher_;
  std_msgs::Float32 front_obstacle_distance_msg_;

  ros::Publisher back_obstacle_distance_publisher_;
  std_msgs::Float32 back_obstacle_distance_msg_;

  ros::Publisher pointcloud_publisher_;
  sensor_msgs::PointCloud2 pointcloud_msg_;

  // [subscriber attributes]
  ros::Subscriber velodyne_subscriber_;
  void cb_velodyne(const sensor_msgs::PointCloud2::ConstPtr& msg);

  ros::Subscriber ackermann_subscriber_;
  void cb_estimatedAckermannState(const ackermann_msgs::AckermannDriveStamped& estimated_ackermann_state_msg);

  pthread_mutex_t velodyne_mutex_;
  void velodyne_mutex_enter(void);
  void velodyne_mutex_exit(void);

  /**
   * \brief config variable
   *
   * This variable has all the driver parameters defined in the cfg config file.
   * Is updated everytime function config_update() is called.
   */
  Config config_;
public:
  /**
   * \brief Constructor
   *
   * This constructor initializes specific class attributes and all ROS
   * communications variables to enable message exchange.
   */
  ReactiveVelodyneAlgNode(void);

  /**
   * \brief Destructor
   *
   * This destructor frees all necessary dynamic memory allocated within this
   * this class.
   */
  ~ReactiveVelodyneAlgNode(void);

protected:
  /**
   * \brief main node thread
   *
   * This is the main thread node function. Code written here will be executed
   * in every node loop while the algorithm is on running state. Loop frequency
   * can be tuned by modifying loop_rate attribute.
   *
   * Here data related to the process loop or to ROS topics (mainly data structs
   * related to the MSG and SRV files) must be updated. ROS publisher objects
   * must publish their data in this process. ROS client servers may also
   * request data to the corresponding server topics.
   */
  void mainNodeThread(void);

  /**
   * \brief dynamic reconfigure server callback
   *
   * This method is called whenever a new configuration is received through
   * the dynamic reconfigure. The derivated generic algorithm class must
   * implement it.
   *
   * \param config an object with new configuration from all algorithm
   *               parameters defined in the config file.
   * \param level  integer referring the level in which the configuration
   *               has been changed.
   */
  void node_config_update(Config &config, uint32_t level);

  /**
   * \brief node add diagnostics
   *
   * In this abstract function additional ROS diagnostics applied to the
   * specific algorithms may be added.
   */
  void addNodeDiagnostics(void);

  // [diagnostic functions]

  // [test functions]
};

#endif
