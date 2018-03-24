#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_broadcaster/Pose.h>

// Copyied from turtle_tf2

std::string robot_name;

void poseCallback(const tf2_broadcaster::PoseConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
  
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = robot_name;
        transformStamped.transform.translation.x = msg->x;
        transformStamped.transform.translation.y = msg->y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  if (! private_node.hasParam("robot"))
  {
    if (argc != 2){ROS_ERROR("need robot name as argument"); return -1;};
    robot_name = argv[1];
  }
  else
  {
    private_node.getParam("robot", robot_name);
  }
    
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(robot_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};
