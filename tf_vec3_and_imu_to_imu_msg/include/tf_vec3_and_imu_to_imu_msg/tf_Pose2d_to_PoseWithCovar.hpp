#ifndef __robot_localization_demo__tf_Pose2D_to_PoseWithCovar__
#define __robot_localization_demo__tf_Pose2D_to_PoseWithCovar__

#include <random>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

namespace robot_localization_demo {

  class tf_Pose2D_to_PoseWithCovar {
    public:
      tf_Pose2D_to_PoseWithCovar(ros::NodeHandle node_handle, double frequency, double error_x_systematic,
          double error_x_random, double error_y_systematic, double error_y_random, double error_yaw_systematic,
          double error_yaw_random, bool visualize=false);
      ~tf_Pose2D_to_PoseWithCovar();
      void spin();
    private:
      ros::NodeHandle node_handle_;
      ros::Subscriber tf_Pose2D_subscriber_;
      //ros::Subscriber turtle_imurpy_subscriber_;
      ros::Publisher tf_Pose2D_to_PoseWithCovar_publisher_;
      double frequency_;
      std::default_random_engine random_generator_;
      std::normal_distribution<double> random_distribution_x_;
      std::normal_distribution<double> random_distribution_y_;
      std::normal_distribution<double> random_distribution_yaw_;
      bool visualize_;
      std::string visualization_turtle_name_;
      unsigned frame_sequence_;

      ros::Time cached_data_timestamp_;
      //geometry_msgs::Vector3Stamped cached_rpy_;
      geometry_msgs::Pose2D cached_data_;

      //void turtleImu_rpy_Callback(const geometry_msgs::Vector3Stamped & message);
      void tf_Pose2D_Callback(const geometry_msgs::Pose2D & message);
  };

}

#endif