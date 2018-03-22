/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use Local position control
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "flight_arduino_com/flight_ctrl.h"
#include "dji_sdk/dji_sdk.h"


//ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
uint8_t current_gps_health = 0;
int num_targets = 0;
geometry_msgs::PointStamped local_position;
sensor_msgs::NavSatFix current_gps_position;

int ctrl_status = 0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flight_ctrl_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  ros::Subscriber gpsHealth      = nh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback);
  ros::Subscriber environmentSub = nh.subscribe("environment", 10, &environmentCallback);

  // Publish the control signal
  ctrlPosYawPub =  nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;

    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
    sleep(2);
    //ctrl_status = 1;
  ros::spin();
  return 0;
}


/*!
 * This function calculates the difference between target and current local position
 * and sends the commands to the Position and Yaw control topic.
 *
 */


bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool control_release()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=0;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

/*roll,pitch : [degree], yaw_rate : [degree / sec]*/
void flight_ctrl_RPYawRate(double x, double y, double z, double yaw_rate){
  sensor_msgs::Joy flight_data;
  uint8_t flag = (DJISDK::VERTICAL_VELOCITY |
                  DJISDK::HORIZONTAL_VELOCITY  |
                  DJISDK::YAW_RATE          |
                  DJISDK::HORIZONTAL_BODY   |
                  DJISDK::STABLE_DISABLE);
  flight_data.axes.push_back(x);
  flight_data.axes.push_back(y);
  flight_data.axes.push_back(z);
  flight_data.axes.push_back(yaw_rate * 3.14159265358979 / 180.0);
  flight_data.axes.push_back(flag);
  ctrlPosYawPub.publish(flight_data);
}

void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  double x, y, z, yaw_rate;
  sensor_msgs::Joy controlPosYaw;

  // Down sampled to 50Hz loop
  if (elapsed_time > ros::Duration(0.02)) {
    start_time = ros::Time::now();
    if (ctrl_status == 0) {
      x = 0.0;
      y = 0.0;
      z = 0.0;
      yaw_rate = 0.0;
      flight_ctrl_RPYawRate(x, y, z, yaw_rate);
    } else if (ctrl_status == 1){
      x = 2.0;
      y = 0.0;
      z = 0.0;
      yaw_rate = 0.0;
      flight_ctrl_RPYawRate(x, y, z, yaw_rate);
    }
  }
}


void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void environmentCallback(const flight_arduino_com::flight_status::ConstPtr& msg)
{
  ctrl_status = msg->flight_status;
}
/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
*/


/*!
 * This function demos how to use M100 flight_status
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps_position.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     current_gps_position.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}
