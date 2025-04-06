#include "PhiROS2Aria.hpp"
#include "Pose.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<PhiROS2Aria> nh = std::make_shared<PhiROS2Aria>();

  if( nh->Setup() == false )
  {
    return -1;
  }

  rclcpp::spin(nh);
  rclcpp::shutdown();
  
  return 0;
}
