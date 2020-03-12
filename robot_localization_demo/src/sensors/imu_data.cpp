#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Transform.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/TeleportRelative.h>
#include <robot_localization_demo/positioning_system.hpp>
#include <robot_localization_demo/odometry.hpp>
#include <robot_localization_demo/imu_data.hpp>


namespace robot_localization_demo {

  TurtleImuSim::TurtleImuSim(ros::NodeHandle node_handle, double frequency,
      double error_vx_systematic, double error_vx_random, double error_wz_systematic, double error_wz_random,
      double error_yaw_systematic, double error_yaw_random, bool visualize):
    node_handle_{node_handle},
    turtle_pose_subscriber_{node_handle_.subscribe("turtle1/pose", 16, &TurtleImuSim::turtlePoseCallback, this)},
    turtle_imudata_publisher_{node_handle_.advertise<sensor_msgs::Imu>("turtle1/sensors/imu", 16)},
    turtle_imurpy_publisher_{node_handle_.advertise<geometry_msgs::Vector3Stamped>("turtle1/sensors/imu_rpy", 16)},
    //turtle_imudata_publisher_{node_handle_.advertise<sensor_msgs::Imu>("base_imu", 16)},
    frequency_{frequency},
    random_generator_{},
    random_distribution_vx_{error_vx_systematic, error_vx_random},
    random_distribution_wz_{error_wz_systematic, error_wz_random},
    random_distribution_yaw_{error_yaw_systematic, error_yaw_random},
    frame_sequence_{0},
    visualize_{visualize},
    visualization_turtle_name_{""}
  {
    ;
  }


  TurtleImuSim::~TurtleImuSim() {
    ;
  }


  void TurtleImuSim::spin() {
    ros::Rate rate(frequency_);
    while(node_handle_.ok()) {
      ros::spinOnce();
      // Distort real twist to get a 'measurement'.
      auto measurement = cached_pose_; 
      //measurement.linear_velocity *= (1. + random_distribution_vx_(random_generator_));
      //measurement.angular_velocity += measurement.angular.z * random_distribution_wz_(random_generator_);
      //measurement.theta += random_distribution_yaw_(random_generator_);

      //ENU orientation:
      //measurement.angular_velocity = measurement.angular_velocity;
      //measurement.theta = measurement.theta;
      // Publish measurement.
      sensor_msgs::Imu current_imudata;
      geometry_msgs::Vector3Stamped current_rpy;

      current_imudata.header.seq = ++ frame_sequence_;
      current_imudata.header.stamp = ros::Time::now();
      current_imudata.header.frame_id = "base_imu";

      current_rpy.header.seq = ++ frame_sequence_;
      current_rpy.header.stamp = ros::Time::now();
      current_rpy.header.frame_id = "base_imu";

      current_imudata.orientation = tf::createQuaternionMsgFromRollPitchYaw(0., 0., measurement.theta);

      current_rpy.vector.x = 0.;
      current_rpy.vector.y = 0.;
      current_rpy.vector.z = measurement.theta;

      current_imudata.orientation_covariance = boost::array<double, 9>({
          0.001, 0., 0.,
          0., 0.001, 0.,
          //0., 0., std::pow(random_distribution_yaw_.mean() + random_distribution_yaw_.stddev(), 2) });
          0., 0., 0.001 });

      current_imudata.angular_velocity.x = 0.;
      current_imudata.angular_velocity.y = 0.;
      current_imudata.angular_velocity.z = measurement.angular_velocity;

      current_imudata.angular_velocity_covariance = boost::array<double, 9>({
          0.001, 0., 0.,
          0., 0.001, 0.,
          //0., 0., std::pow(measurement.linear_velocity * (random_distribution_wz_.mean() + random_distribution_wz_.stddev()), 2) });
          0., 0., 0.001 });

      current_imudata.linear_acceleration.x = 0.; //Zero is incorrect, but it's not will be fused
      current_imudata.linear_acceleration.y = 0.;
      current_imudata.linear_acceleration.z = 0.;
      current_imudata.linear_acceleration_covariance = boost::array<double, 9>({
          0.001, 0., 0.,
          0., 0.001, 0.,
          0., 0., 0.001 });

      turtle_imudata_publisher_.publish(current_imudata);
      turtle_imurpy_publisher_.publish(current_rpy);

      if(isVisualizationRequested() && isVisualizationTurtleAvailable()) {
        moveVisualizationTurtle(measurement); //CAMBIAR POR EL VALOR QUE CORRESPONDE AL MOVIMIENTO DEL IMU
      }
      // Sleep until we need to publish a new measurement.
      rate.sleep();
    }
  }


  void TurtleImuSim::turtlePoseCallback(const turtlesim::PoseConstPtr & message) {
    cached_pose_timestamp_ = ros::Time::now();
    cached_pose_ = *message;
    // If this is the first message, initialize the visualization turtle.
    if(isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
      spawnAndConfigureVisualizationTurtle(*message);
    }
  }


  void TurtleImuSim::spawnAndConfigureVisualizationTurtle(const turtlesim::Pose & initial_pose) {
    if(isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
      // Spawn a new turtle and store its name.
      ros::service::waitForService("spawn");
      turtlesim::Spawn spawn_visualization_turtle;
      spawn_visualization_turtle.request.x = initial_pose.x;
      spawn_visualization_turtle.request.y = initial_pose.y;
      spawn_visualization_turtle.request.theta = initial_pose.theta;
      /*
      tf::Quaternion quat_tf;
      geometry_msgs::Quaternion quat_msg;
      quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0., 0., initial_pose.angular.z);

      //quaternionTFToMsg(quat_tf , quat_msg);
      quaternionMsgToTF(quat_msg , quat_tf);

      tf::Matrix3x3 m(quat_tf);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      spawn_visualization_turtle.request.theta = yaw;
      */

      auto client = node_handle_.serviceClient<decltype(spawn_visualization_turtle)>("spawn");
      client.call(spawn_visualization_turtle);
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
      ROS_INFO("Relative position measurement (IMU) visualized by '%s' with a blue pen.", visualization_turtle_name_.c_str());
    }
  }


  void TurtleImuSim::moveVisualizationTurtle(const turtlesim::Pose & measurement) {
    if(isVisualizationRequested() && isVisualizationTurtleAvailable()) {
      // Move visualization turtle to the 'measured' position.
      turtlesim::TeleportRelative visualize_current_imudata;
      visualize_current_imudata.request.linear = measurement.linear_velocity / frequency_;
      visualize_current_imudata.request.angular = measurement.angular_velocity / frequency_;
      auto client = node_handle_.serviceClient<decltype(visualize_current_imudata)>(
          visualization_turtle_name_ + "/teleport_relative");
      client.call(visualize_current_imudata);
    }
  }

}