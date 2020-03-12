#ifndef __robot_localization_demo__imu_data__
#define __robot_localization_demo__imu_data__

#include <random>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

namespace robot_localization_demo {

  class TurtleImuSim {
    public:
      TurtleImuSim(ros::NodeHandle node_handle, double frequency, double error_vx_systematic, double error_vx_random,
          double error_wz_systematic, double error_wz_random, double error_yaw_systematic,
          double error_yaw_random, bool visualize=false);
      ~TurtleImuSim();
      void spin();
    private:
      ros::NodeHandle node_handle_;
      ros::Subscriber turtle_pose_subscriber_;
      ros::Publisher turtle_imudata_publisher_;
      ros::Publisher turtle_imurpy_publisher_;

      double frequency_;
      std::default_random_engine random_generator_;
      std::normal_distribution<double> random_distribution_vx_;
      std::normal_distribution<double> random_distribution_wz_;
      std::normal_distribution<double> random_distribution_yaw_;
      bool visualize_;
      std::string visualization_turtle_name_;
      unsigned frame_sequence_;
      ros::Time cached_pose_timestamp_;
      turtlesim::Pose cached_pose_;
      void turtlePoseCallback(const turtlesim::PoseConstPtr & message);
      inline bool isVisualizationRequested() { return visualize_; };
      inline bool isVisualizationTurtleAvailable() { return visualization_turtle_name_ != ""; };
      void spawnAndConfigureVisualizationTurtle(const turtlesim::Pose & initial_pose);
      void moveVisualizationTurtle(const turtlesim::Pose & measurement);
  };

}

#endif
