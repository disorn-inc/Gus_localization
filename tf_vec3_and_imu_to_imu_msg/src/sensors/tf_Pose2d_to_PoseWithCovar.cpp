#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf_vec3_and_imu_to_imu_msg/tf_Pose2d_to_PoseWithCovar.hpp>


namespace robot_localization_demo {

  tf_Pose2D_to_PoseWithCovar::tf_Pose2D_to_PoseWithCovar(ros::NodeHandle node_handle, double frequency,
      double error_x_systematic, double error_x_random, double error_y_systematic, double error_y_random,
      double error_yaw_systematic, double error_yaw_random,
      bool visualize):
    node_handle_{node_handle},
    tf_Pose2D_subscriber_{node_handle_.subscribe("pose2D", 16, &tf_Pose2D_to_PoseWithCovar::tf_Pose2D_Callback, this)},
    //turtle_imurpy_subscriber_{node_handle_.subscribe("imu/rpy", 16, &Turtletf_Vector3Stamped_to_ImuData::turtleImu_rpy_Callback, this)},
    //turtle_imurpy_subscriber_{node_handle_.subscribe("pose2D", 16, &Turtletf_Pose2D_to_PoseWithCovar::turtleImu_rpy_Callback, this)},
    tf_Pose2D_to_PoseWithCovar_publisher_{node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("created/PoseWCStamped", 16)},
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


  tf_Pose2D_to_PoseWithCovar::~tf_Pose2D_to_PoseWithCovar() {
    ;
  }


  void tf_Pose2D_to_PoseWithCovar::spin() {
    ros::Rate rate(frequency_);
    while(node_handle_.ok()) {
      ros::spinOnce();
      // Distort real pose to get a 'measurement'.
      auto measurement_data = cached_data_;
      //auto measurement_rpy = cached_rpy_;
      //measurement.x += random_distribution_x_(random_generator_);
      //measurement.y += random_distribution_y_(random_generator_);
      //measurement.theta += random_distribution_yaw_(random_generator_);
      // Publish measurement.
      geometry_msgs::PoseWithCovarianceStamped current_pose_wc_stamped_data;
      current_pose_wc_stamped_data.header.seq = ++ frame_sequence_;
      current_pose_wc_stamped_data.header.stamp = ros::Time::now();
      current_pose_wc_stamped_data.header.frame_id = "odom";
      current_pose_wc_stamped_data.pose.pose.position.x = measurement_data.x;
      current_pose_wc_stamped_data.pose.pose.position.y = measurement_data.y;
      current_pose_wc_stamped_data.pose.pose.position.z = 0;
      //current_pose.theta = -measurement.theta;
      //current_pose.linear_velocity = measurement.linear_velocity;
      //current_pose.angular_velocity = -measurement.angular_velocity;

      //current_imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(measurement_rpy.vector.x, measurement_rpy.vector.y, measurement_rpy.vector.z);
      current_pose_wc_stamped_data.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, measurement_data.theta);
      current_pose_wc_stamped_data.pose.covariance = boost::array<double, 36>({
          0.01, 0., 0., 0., 0., 0.,
          0., 0.01, 0., 0., 0., 0.,
          0., 0., 0.01, 0., 0., 0.,
          0., 0., 0., 0.01, 0., 0.,
          0., 0., 0., 0., 0.01, 0.,
          0., 0., 0., 0., 0., 0.01,
        });

      tf_Pose2D_to_PoseWithCovar_publisher_.publish(current_pose_wc_stamped_data);

      rate.sleep();
    }
  }


  void tf_Pose2D_to_PoseWithCovar::tf_Pose2D_Callback(const geometry_msgs::Pose2D & message) {
    cached_data_timestamp_ = ros::Time::now();
    cached_data_ = message;
    // If this is the first message, initialize the visualization turtle.
    //if(isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
    //  spawnAndConfigureVisualizationTurtle(*message);
    //}
  }

  //void Turtletf_Vector3Stamped_to_ImuData::turtleImu_rpy_Callback(const geometry_msgs::Vector3Stamped & message) {
  /*void Turtletf_Vector3Stamped_to_ImuData::turtleImu_rpy_Callback(const geometry_msgs::Pose2D & message) {
    cached_rpy_timestamp_ = ros::Time::now();
    cached_rpy_ = message;
    // If this is the first message, initialize the visualization turtle.
    //if(isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
    //  spawnAndConfigureVisualizationTurtle(*message);
    //}
  }*/

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