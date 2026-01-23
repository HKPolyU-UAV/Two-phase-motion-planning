#include "motion_primitives.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "motion_primitives_test");
  ros::NodeHandle nh;

  MotionPrimitives mp(nh);
  // ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}