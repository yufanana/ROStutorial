#include "ros/ros.h"
#include "ros_essentials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;

  // create client object
  ros::ServiceClient client = n.serviceClient<ros_essentials::AddTwoInts>("add_two_ints");
  // ros_essentials::AddTwoInts: service type
  // "add_two_ints": service name

  // create service object
  ros_essentials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}