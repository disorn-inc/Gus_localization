#ifndef __robot_localization_demo__tf_Vector3Stamped_to_ImuData__
#define __robot_localization_demo__tf_Vector3Stamped_to_ImuData__

#include <random>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

namespace robot_localization_demo {

  class tf_Odometry_to_ImuData {
    public:
      tf_Odometry_to_ImuData(ros::NodeHandle node_handle, double frequency, double error_x_systematic,
          double error_x_random, double error_y_systematic, double error_y_random, double error_yaw_systematic,
          double error_yaw_random, bool visualize=false);
      ~tf_Odometry_to_ImuData();
      void spin();
    private:
      ros::NodeHandle node_handle_;
      ros::Subscriber imudata_subscriber_;
      ros::Subscriber Odometry_subscriber_;
      ros::Publisher tf_Odometry_to_ImuData_publisher_;
      double frequency_;
      std::default_random_engine random_generator_;
      std::normal_distribution<double> random_distribution_x_;
      std::normal_distribution<double> random_distribution_y_;
      std::normal_distribution<double> random_distribution_yaw_;
      bool visualize_;
      std::string visualization_turtle_name_;
      unsigned frame_sequence_;

      ros::Time cached_imudata_timestamp_;
      sensor_msgs::Imu cached_imudata_;

      void Imu_data_Callback(const sensor_msgs::Imu & message);

      ros::Time cached_odom_timestamp_;
      nav_msgs::Odometry cached_odom_;

      void Odometry_Callback(const nav_msgs::Odometry & message);
  };

}

#endif