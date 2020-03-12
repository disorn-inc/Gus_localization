#include <move_base/move_base.h>
#include <string>
#include <std_msgs/String.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "nav_move_base");
  ros::NodeHandle nh;

  // ros::Publisher points_pub = nh.advertise<std_msgs::String>("/points_files", 5);
  // std::stringstream filename, adjacency, s;
  // filename << "/home/" << getenv("USER") << "/catkin_ws/src/navgraph_solver/navgraphs/points_rviz.csv";
  // adjacency << "/home/" << getenv("USER") << "/catkin_ws/src/navgraph_solver/navgraphs/adjacency_rviz.csv";
  // s << filename.str() << "+" << adjacency.str();
  // std_msgs::String files;
  // files.data = s.str();
  // points_pub.publish(files);
  // ROS_INFO("Starting MoveBase");
  // ros::spinOnce();

  tf::TransformListener tf(ros::Duration(10));
  move_base::MoveBase move_base( tf );
  ros::spin();

  return(0);
}
