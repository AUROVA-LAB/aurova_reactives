#include "reactive_velodyne_alg_node.h"

ReactiveVelodyneAlgNode::ReactiveVelodyneAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<ReactiveVelodyneAlgorithm>()
{
  //init class attributes if necessary
  flag_new_velodyne_data_ = false;

  z_threshold_ = MIN_OBSTACLE_HEIGHT_;

  abs_lateral_safety_margin_ = 0.15;

  safety_width_ = VEHICLE_WIDTH_ + (2.0 * abs_lateral_safety_margin_);

  euclidean_association_threshold_ = 0.10;
  min_obstacle_radius_ = 0.03;

  closest_front_obstacle_point_ = OUT_OF_RANGE_;
  closest_back_obstacle_point_ = OUT_OF_RANGE_;

  steering_angle_ = 0.0;

  this->loop_rate_ = 500;

  // [init publishers]
  this->front_obstacle_distance_publisher_ = this->public_node_handle_.advertise < std_msgs::Float32
      > ("/velodyne_front_closest_obstacle_distance", 1);

  this->back_obstacle_distance_publisher_ = this->public_node_handle_.advertise < std_msgs::Float32
      > ("/velodyne_back_closest_obstacle_distance", 1);

  this->pointcloud_publisher_ = this->public_node_handle_.advertise < sensor_msgs::PointCloud2 > ("/velodyne_obstacle_points", 1);

  // [init subscribers]
  this->velodyne_subscriber_ = this->public_node_handle_.subscribe("/velodyne_points", 1,
                                                                   &ReactiveVelodyneAlgNode::cb_velodyne, this);

  this->ackermann_subscriber_ = this->public_node_handle_.subscribe("/estimated_ackermann_state", 1,
                                                                    &ReactiveVelodyneAlgNode::cb_estimatedAckermannState,
                                                                    this);

  pthread_mutex_init(&this->velodyne_mutex_, NULL);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

ReactiveVelodyneAlgNode::~ReactiveVelodyneAlgNode(void)
{
  // [free dynamic memory]
}

void ReactiveVelodyneAlgNode::mainNodeThread(void)
{
  this->velodyne_mutex_enter();
  if (flag_new_velodyne_data_)
  {
    // [fill msg structures]
    flag_new_velodyne_data_ = false;
    local_copy_of_input_cloud_ = input_cloud_;
    this->velodyne_mutex_exit();

    this->alg_.filterPointsOutsideWorkArea(local_copy_of_input_cloud_,
                                           MAX_VEL_, TIME_TO_REACH_OBSTACLE_,
                                           SENSOR_HEIGHT_, MIN_OBSTACLE_HEIGHT_,
                                           SAFETY_MARGIN_ABOVE_SENSOR_,
                                           obstacle_points_);

    if (fabs(steering_angle_) < 1.0) // if the steering is close to zero, the center is at infinity, so we assume straight line
    {
      this->alg_.filterPointsStraightLine(obstacle_points_, safety_width_, obstacle_points_);
    }
    else
    {
      this->alg_.filterPointsByTurningRadius(obstacle_points_, steering_angle_, WHEELBASE_METERS_, safety_width_,
                                             X_DISTANCE_FROM_BASE_LINK_TO_SENSOR_, obstacle_points_);
    }

    this->alg_.findClosestDistance(obstacle_points_, closest_front_obstacle_point_, closest_back_obstacle_point_);


    // [publish messages]
    float front_distance_saturated = closest_front_obstacle_point_ - DISTANCE_FROM_SENSOR_TO_FRONT_;
    if(front_distance_saturated < 0.0)
    {
      front_distance_saturated = 0.0;
    }
    front_obstacle_distance_msg_.data = front_distance_saturated;
    this->front_obstacle_distance_publisher_.publish(this->front_obstacle_distance_msg_);


    float back_distance_saturated = closest_back_obstacle_point_ - DISTANCE_FROM_SENSOR_TO_BACK_;
    if(back_distance_saturated < 0.0)
    {
      back_distance_saturated = 0.0;
    }

    back_obstacle_distance_msg_.data = back_distance_saturated;
    this->back_obstacle_distance_publisher_.publish(this->back_obstacle_distance_msg_);

    obstacle_points_.header.frame_id = local_copy_of_input_cloud_.header.frame_id;
    obstacle_points_.header.stamp = local_copy_of_input_cloud_.header.stamp;
    pointcloud_msg_ = obstacle_points_;
    this->pointcloud_publisher_.publish(this->pointcloud_msg_);

  }
  else
  {
    this->velodyne_mutex_exit();
  }
}

void ReactiveVelodyneAlgNode::cb_velodyne(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  this->velodyne_mutex_enter();

  input_cloud_ = *msg;

  //DEBUG!!
  std::cout << "Velodyne scan received!" << std::endl;
  if (msg == NULL)
    std::cout << std::endl << "Null pointer!!! in function velodyneCB!";

  flag_new_velodyne_data_ = true;

  this->velodyne_mutex_exit();
}

void ReactiveVelodyneAlgNode::cb_estimatedAckermannState(
    const ackermann_msgs::AckermannDriveStamped& estimated_ackermann_state_msg)
{
  this->velodyne_mutex_enter();

  //DEBUG!!
  std::cout << "Ackermann state received!" << std::endl;

  steering_angle_ = estimated_ackermann_state_msg.drive.steering_angle;

  this->velodyne_mutex_exit();
}

void ReactiveVelodyneAlgNode::velodyne_mutex_enter(void)
{
  pthread_mutex_lock(&this->velodyne_mutex_);
}

void ReactiveVelodyneAlgNode::velodyne_mutex_exit(void)
{
  pthread_mutex_unlock(&this->velodyne_mutex_);
}

void ReactiveVelodyneAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void ReactiveVelodyneAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < ReactiveVelodyneAlgNode > (argc, argv, "reactive_velodyne_alg_node");
}
