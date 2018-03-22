#include "ros/ros.h"
#include "flight_arduino_com/arduino_echo.h"
#include "flight_arduino_com/flight_status.h"
#include <stdlib.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduino_com_client");
  ros::NodeHandle nh;
  ros::ServiceClient arduino_service = nh.serviceClient<flight_arduino_com::arduino_echo>("arduino_com");
  ros::Publisher flightStatusPub = nh.advertise<flight_arduino_com::flight_status>("environment",10);
  flight_arduino_com::arduino_echo srv;
  flight_arduino_com::flight_status flight_status_msg;
  int pulse_len = 150000;
  while(ros::ok())
  {
    std::cout << "Press Enter !" << std::endl;
    getchar();
    srv.request.pulse_len = pulse_len;
    if (arduino_service.call(srv))
    {
      ROS_INFO("Control_mode : %d", (int)srv.response.ctrl_mode);
      flight_status_msg.flight_status = srv.response.ctrl_mode;
      flightStatusPub.publish(flight_status_msg);
    }
    else
    {
      ROS_ERROR("Failed to communicate arduino");
    }
  }
  return 0;
}
