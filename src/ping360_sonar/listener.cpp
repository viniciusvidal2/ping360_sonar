#include <ros/ros.h>
#include <vector>
#include <string>
#include <stdio.h>
#include <ping360_sonar/SonarEcho.h>
#include <ping360_sonar/sonarConfig.h>

void myCallback(const ping360_sonar::SonarEchoConstPtr& msg)
{
  ROS_WARN("I am listening to the echo!");
  std::cout << "The angle: " << msg->angle << "\n";
  std::cout << "The range: " << static_cast<int>(msg->range) << "\n";
  std::cout << "The intensities: \n"; 
  for (auto i : msg->intensities)
    std::cout << static_cast<int>(i) << " ";
  std::cout << "\nThe # of samples: " << msg->number_of_samples << "\n\n\n";
}

/// MAIN
///
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_node");
  ros::NodeHandle nh;
  ROS_INFO("Initialyzing node ...");

  ros::Subscriber sub = nh.subscribe("/ping360_node/sonar/data", 1000, myCallback);

  ros::spin();

  return 0;
}
