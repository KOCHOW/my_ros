//#include "flight_ctrl/flight_ctrl.h"
#include <tf/tf.h>
#include "dji_sdk/dji_sdk.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

ros::Publisher ctrlRPYratePub;

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

geometry_msgs::Vector3 current_atti;
geometry_msgs::Vector3 current_velocity;

const double PI = 3.14159265358979;
const double rad2deg = PI / 180.0;
const double deg2rad = 180.0 / PI;
const double K_P = 0.1; //P-Gain
const double K_I = 0.0; //I-Gain
const double K_D = 0.0; //D-Gain
int step = 0;

uint8_t flight_status = 255;
sensor_msgs::NavSatFix current_gps_position;
ros::ServiceClient sdk_ctrl_authority_service;
bool takeoff_result = 0;

void takeoff_callback(flight_ctrl::keyinput:ConstPtr& msg)
{
  takeoff_result = msg->num;
}

double target_velocity = 0.0;
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}
void gps_position_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps_position = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pid_ctrl_node");
  ros::NodeHandle nh;

  ros::Subscriber velocitySub = nh.subscribe("dji_sdk/attitude", 10, &velocity_callback);
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_position_callback);
  ros::Subscriber takeoffSub = nh.subscribe("takeoffResult", 10, &takeoff_callback);
  ctrlRPYratePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);

  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");



  if (takeoff_result)
  {
    step = 1;
  }

  ros::spin();
  return 0;
}

void flight_control(double roll, double pitch, double zVel, double yawRate)
{
  sensor_msgs::Joy flight_data;
  flight_data.axes.push_back(roll);
  flight_data.axes.push_back(pitch);
  flight_data.axes.push_back(zVel);
  flight_data.axes.push_back(yawRate);
  ctrlRPYratePub.publish(flight_data);
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  static double errInt = 0.0;
  static double preErr = 0.0;

  current_velocity = msg->vector;
  double err = target_velocity - current_velocity.y;
  errInt += err * elapsed_time.toSec();
  double diffErr = ( err - preErr ) / elapsed_time.toSec();
  double pitch = K_P * err + K_I * errInt + K_D * diffErr;
  preErr = err;
  flight_control(0.0,pitch,0.0,0.0);
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = toEulerAngle(msg->quaternion);
}
