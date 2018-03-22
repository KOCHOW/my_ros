#include "ros/ros.h"
#include "flight_ctrl/key_input.h"

#include <stdlib.h>
//#include <kbhit.h>


using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "key_input_pub");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<flight_ctrl::key_input>("key_data", 10);
  ros::Rate loop_rate(50);

  flight_ctrl::key_input key;

  key.num = 0;
  pub.publish(key);

  while (ros::ok())
  {
    int key_value = 0;
    cout << "0:stop, 1:forword, 2:back" << endl;
    cin >> key_value;
    if (key_value == 0 || key_value == 1 || key_value == 2)
    {
      key.num = key_value;
    }
    else
    {
      ROS_ERROR("Please input 0 or 1 or 2");
    }
    pub.publish(key);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
