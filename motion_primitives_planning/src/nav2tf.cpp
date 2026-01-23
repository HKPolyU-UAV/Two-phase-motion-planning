#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ros/ros.h>

std::string child_frame_id;

void posCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    static tf2_ros::TransformBroadcaster pos_br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header = msg->header;
    // transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = child_frame_id;
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
    pos_br.sendTransform(transformStamped);
    ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "nav2tf");
    ros::NodeHandle nh;
    child_frame_id = "suav1/base_link";
    ros::Subscriber sub = nh.subscribe("mavros/global_position/local", 10, posCallback);

    ros::spin();
    return 0;
};
