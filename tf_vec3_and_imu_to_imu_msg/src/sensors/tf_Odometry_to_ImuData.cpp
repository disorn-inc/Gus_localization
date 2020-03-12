#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf_vec3_and_imu_to_imu_msg/tf_Odometry_to_ImuData.hpp>


namespace robot_localization_demo {

  tf_Odometry_to_ImuData::tf_Odometry_to_ImuData(ros::NodeHandle node_handle, double frequency,
      double error_x_systematic, double error_x_random, double error_y_systematic, double error_y_random,
      double error_yaw_systematic, double error_yaw_random,
      bool visualize):
    node_handle_{node_handle},
    imudata_subscriber_{node_handle_.subscribe("gx5/imu/data", 16, &tf_Odometry_to_ImuData::Imu_data_Callback, this)},
    Odometry_subscriber_{node_handle_.subscribe("odometry/robot_localization_attitude", 16, &tf_Odometry_to_ImuData::Odometry_Callback, this)},
    tf_Odometry_to_ImuData_publisher_{node_handle_.advertise<sensor_msgs::Imu>("msg_transform/attitude/imu_msg", 16)},
    frequency_{frequency},
    random_generator_{},
    random_distribution_x_{error_x_systematic, error_x_random},
    random_distribution_y_{error_y_systematic, error_y_random},
    random_distribution_yaw_{error_yaw_systematic, error_yaw_random},
    frame_sequence_{0},
    visualize_{visualize_},
    visualization_turtle_name_{""}
  {
    ;
  }


  tf_Odometry_to_ImuData::~tf_Odometry_to_ImuData() {
    ;
  }


  void tf_Odometry_to_ImuData::spin() {
    ros::Rate rate(frequency_);
    while(node_handle_.ok()) {
      ros::spinOnce();

      auto measurement_imudata = cached_imudata_;
      auto measurement_odom = cached_odom_;
      // Publish measurement.
      sensor_msgs::Imu current_imu_data;
      current_imu_data.header.seq = ++ frame_sequence_;
      current_imu_data.header.stamp = ros::Time::now();
      current_imu_data.header.frame_id = "imu_link";

      current_imu_data.orientation = measurement_odom.pose.pose.orientation;
      current_imu_data.orientation_covariance = boost::array<double, 9>({
          measurement_odom.pose.covariance[21], 0., 0.,
          0., measurement_odom.pose.covariance[28], 0.,
          0., 0., measurement_odom.pose.covariance[35]});

      current_imu_data.angular_velocity = measurement_odom.twist.twist.angular;
      current_imu_data.angular_velocity_covariance = boost::array<double, 9>({
          measurement_odom.twist.covariance[21], 0., 0.,
          0., measurement_odom.twist.covariance[28], 0.,
          0., 0., measurement_odom.twist.covariance[35]});

      current_imu_data.linear_acceleration = measurement_imudata.linear_acceleration;
      current_imu_data.linear_acceleration_covariance = measurement_imudata.linear_acceleration_covariance;

      tf_Odometry_to_ImuData_publisher_.publish(current_imu_data);

      rate.sleep();
    }
  }


  void tf_Odometry_to_ImuData::Imu_data_Callback(const sensor_msgs::Imu & message) {
    cached_imudata_timestamp_ = ros::Time::now();
    cached_imudata_ = message;
  }

  void tf_Odometry_to_ImuData::Odometry_Callback(const nav_msgs::Odometry & message) {
    cached_odom_timestamp_ = ros::Time::now();
    cached_odom_ = message;
  }

/*
  void TurtleTf_ned_to_enu::spawnAndConfigureVisualizationTurtle(const turtlesim::Pose & initial_pose) {
    if(isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
      // Spawn a new turtle and store its name.
      ros::service::waitForService("spawn");
      turtlesim::Spawn spawn_visualization_turtle;
      spawn_visualization_turtle.request.x = initial_pose.x;
      spawn_visualization_turtle.request.y = initial_pose.y;
      spawn_visualization_turtle.request.theta = initial_pose.theta;
      auto client_spawn = node_handle_.serviceClient<decltype(spawn_visualization_turtle)>("spawn");
      client_spawn.call(spawn_visualization_turtle);
      visualization_turtle_name_ = spawn_visualization_turtle.response.name;
      // Set pen color to blue.
      turtlesim::SetPen configure_visualization_turtle;
      configure_visualization_turtle.request.r = 0;
      configure_visualization_turtle.request.g = 0;
      configure_visualization_turtle.request.b = 255;
      configure_visualization_turtle.request.width = 1;
      configure_visualization_turtle.request.off = 0;
      auto client_configure = node_handle_.serviceClient<decltype(configure_visualization_turtle)>(
          visualization_turtle_name_ + "/set_pen");
      client_configure.call(configure_visualization_turtle);
      // Log message.
      ROS_INFO("Absolute position measurement visualized by '%s' using a blue pen.", visualization_turtle_name_.c_str());
    }
  }


  void TurtleTf_ned_to_enu::moveVisualizationTurtle(const turtlesim::Pose & measurement) {
    if(isVisualizationRequested() && isVisualizationTurtleAvailable()) {
      // Move visualization turtle to the 'measured' position.
      turtlesim::TeleportAbsolute visualize_current_pose;
      visualize_current_pose.request.x = measurement.x;
      visualize_current_pose.request.y = measurement.y;
      visualize_current_pose.request.theta = measurement.theta;
      auto client = node_handle_.serviceClient<decltype(visualize_current_pose)>(
          visualization_turtle_name_ + "/teleport_absolute");
      client.call(visualize_current_pose);
    }
  }*/

}