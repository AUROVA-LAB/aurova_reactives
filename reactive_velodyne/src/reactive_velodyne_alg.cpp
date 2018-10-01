#include "reactive_velodyne_alg.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

ReactiveVelodyneAlgorithm::ReactiveVelodyneAlgorithm(void)
{
  pthread_mutex_init(&this->access_, NULL);
}

ReactiveVelodyneAlgorithm::~ReactiveVelodyneAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void ReactiveVelodyneAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

// ReactiveVelodyneAlgorithm Public API
void ReactiveVelodyneAlgorithm::filterPointsOutsideWorkArea(sensor_msgs::PointCloud2& input, float max_vel,
                                                            float time_to_reach_obstacle,
                                                            sensor_msgs::PointCloud2& output)
{
  //Debug!
  //std::cout<<"filterPointsOutsideWorkArea!!"<<std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PCLPointCloud2 aux_input;
  pcl_conversions::toPCL(input, aux_input);
  pcl::fromPCLPointCloud2(aux_input, *input_cloud_pcl);

  float abs_max_coordinate = max_vel * time_to_reach_obstacle; // We use the worst case, if the vehicle were
                                                               // omnidirectional it can advance this much
                                                               // either in x or y coordinates

  // Create the filtering object
  pcl::PassThrough < pcl::PointXYZI > pass;
  pass.setInputCloud(input_cloud_pcl);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1 * abs_max_coordinate, abs_max_coordinate);
  pass.filter(*cloud_filtered);

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1 * abs_max_coordinate, abs_max_coordinate);
  pass.filter(*cloud_filtered);

  pcl::PCLPointCloud2 aux_output;
  toPCLPointCloud2(*cloud_filtered, aux_output);
  pcl_conversions::fromPCL(aux_output, output);

}

void ReactiveVelodyneAlgorithm::filterPointsStraightLine(sensor_msgs::PointCloud2& input, float vehicle_width,
                                                         sensor_msgs::PointCloud2& output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PCLPointCloud2 aux_input;
  pcl_conversions::toPCL(input, aux_input);
  pcl::fromPCLPointCloud2(aux_input, *input_cloud_pcl);

  // Create the filtering object
  pcl::PassThrough < pcl::PointXYZI > pass;
  pass.setInputCloud(input_cloud_pcl);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1 * vehicle_width / 2.0, vehicle_width / 2.0);
  pass.filter(*cloud_filtered);

  pcl::PCLPointCloud2 aux_output;
  toPCLPointCloud2(*cloud_filtered, aux_output);
  pcl_conversions::fromPCL(aux_output, output);

}

void ReactiveVelodyneAlgorithm::filterPointsByTurningRadius(sensor_msgs::PointCloud2& input, float steering_angle,
                                                            float wheelbase, float vehicle_width,
                                                            float x_axis_distance_from_base_link_to_velodyne,
                                                            sensor_msgs::PointCloud2& output)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud_pcl(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PCLPointCloud2 aux_input;
  pcl_conversions::toPCL(input, aux_input);
  pcl::fromPCLPointCloud2(aux_input, *input_cloud_pcl);

  float turn_center_x_coordinate = -x_axis_distance_from_base_link_to_velodyne;
  float turn_center_y_coordinate = 0.0;
  float steering_angle_radians = steering_angle * M_PI / 180.0;

  turn_center_y_coordinate = wheelbase / tan(steering_angle_radians);

  std::cout << "turning radius = " << turn_center_y_coordinate << std::endl;

  float x = 0.0;
  float y = 0.0;
  float distance;

  for (size_t i = 0; i < input_cloud_pcl->points.size(); ++i)
  {
    x = input_cloud_pcl->points[i].x;
    y = input_cloud_pcl->points[i].y;

    distance = sqrt(
        (x - turn_center_x_coordinate) * (x - turn_center_x_coordinate)
            + (y - turn_center_y_coordinate) * (y - turn_center_y_coordinate));
    if (distance < fabs(turn_center_y_coordinate) + vehicle_width / 2.0
        && distance > fabs(turn_center_y_coordinate) - vehicle_width / 2.0)
    {
      pcl::PointXYZI point;
      point.x = x;
      point.y = y;
      point.z = input_cloud_pcl->points[i].z;
      output_cloud_pcl->points.push_back(point);
    }
  }
  pcl::PCLPointCloud2 aux_output;
  toPCLPointCloud2(*output_cloud_pcl, aux_output);
  pcl_conversions::fromPCL(aux_output, output);
}

void ReactiveVelodyneAlgorithm::findClosestDistance(sensor_msgs::PointCloud2& input_pointcloud2,
                                                    float& closest_front_distance, float& closest_back_distance)
{
  //std::cout << "findClosestDistance" << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PCLPointCloud2 cloudPCL;
  pcl_conversions::toPCL(input_pointcloud2, cloudPCL);
  pcl::fromPCLPointCloud2(cloudPCL, *input_cloud);

  static const float OUT_OF_RANGE = 100.0;

  float x = 0.0;
  float y = 0.0;
  float distance;
  float min_front_distance = OUT_OF_RANGE;
  float min_back_distance = OUT_OF_RANGE;

  for (size_t i = 0; i < input_cloud->points.size(); ++i)
  {
    x = input_cloud->points[i].x;
    y = input_cloud->points[i].y;

    distance = sqrt(x * x + y * y);
    if (x < 0.0 && distance < min_back_distance)
    {
      min_back_distance = distance;
    }
    if (x > 0.0 && distance < min_front_distance)
    {
      min_front_distance = distance;
    }
  }

  closest_front_distance = min_front_distance;
  closest_back_distance = min_back_distance;
}
