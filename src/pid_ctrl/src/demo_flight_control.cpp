/** @file demo_flight_control.cpp
*  @version 3.3
*  @date May, 2017
*
*  @brief
*  demo sample of how to use flight control APIs
*
*  @copyright 2017 DJI. All rights reserved.
*
*/

#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include "pid_ctrl/flight_data.h"

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;
ros::Publisher flightDataPub;
// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 current_pos;
geometry_msgs::Vector3 object_pos;
bool take_off_condition = false;
double input = 0.5;

pid_ctrl::flight_data data;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  flightDataPub = nh.advertise<pid_ctrl::flight_data>("flight_data", 10);

  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

  bool obtain_control_result = obtain_control();
  bool takeoff_result;

  ROS_INFO("M100 taking off!");
  takeoff_result = M100monitoredTakeoff();

  if(takeoff_result)
  {
    take_off_condition = true;
    if (0)
    {
      dji_sdk::SDKControlAuthority authority;
      authority.request.control_enable=0;
      sdk_ctrl_authority_service.call(authority);
      if (!authority.response.result) std::cout << "failed!!" << '\n';
    }
  }

  ros::spin();
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
  sensor_msgs::NavSatFix& target,
  sensor_msgs::NavSatFix& origin)
  {
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;

    deltaNed.y = deltaLat * deg2rad * C_EARTH;
    deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
    deltaNed.z = target.altitude - origin.altitude;
  }


  geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
  {
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;
  }


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

  void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
  {
    current_atti = msg->quaternion;
  }

  void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    static sensor_msgs::NavSatFix start_gps = *msg;
    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    current_gps = *msg;
    if (take_off_condition)
    {
      static ros::Time first_time = ros::Time::now();
      ros::Duration current_time = ros::Time::now() - first_time;
      double omega = 0.0 * C_PI ;
      object_pos.y = 2.0 * sin( omega * current_time.toSec() ) + 5.0;
      if ( current_time.toSec() > 7 && current_time.toSec() < 15)
      {
	      input = 0.5 + 0.3 * (current_time.toSec() - 7.0);
      }
      localOffsetFromGpsOffset(current_pos, current_gps, start_gps);
      static double errInt = 0.0;
      static double preErr = 0.0;
      double errDiff = 0.0;
      double dist2object = ((double)((int)((object_pos.y - current_pos.y) *100)))/100.0;
      //double pitch = dist2object > 0.5 ? 1 * deg2rad : -1 * deg2rad;
      double Kp = -10.0;
      double Kd = -12.0;
      double Ki = -0.0;
      double pitchFactor = 10.0;
      static double pitch = 0.0;
      if (elapsed_time > ros::Duration(0.02))
      {
        if(elapsed_time > ros::Duration(0.1))
        {
          start_time = ros::Time::now();
          double err = input - dist2object;
          errDiff = (err - preErr) / (elapsed_time.toSec());
          errInt += err * elapsed_time.toSec();
          pitch = (Kp * err + Kd * errDiff + Ki * errInt);
          if (abs(pitch) > pitchFactor)
          {
            pitch = pitch > pitchFactor ? pitchFactor : -1.0 * pitchFactor;
          }
          preErr = err;
        }
        sensor_msgs::Joy controlVelYawRate;
        uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                        DJISDK::HORIZONTAL_ANGLE |
                        DJISDK::YAW_RATE            |
                        DJISDK::HORIZONTAL_BODY   |
                        DJISDK::STABLE_DISABLE);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(pitch * deg2rad);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(flag);
        ctrlBrakePub.publish(controlVelYawRate);
        
        data.t = current_time.toSec();
        data.r = input;
        data.pos = current_pos.y;
        data.pitch = pitch;
        flightDataPub.publish(data);
        //std::cout << "t = " << current_time.toSec() << ", r = " << input << ", pos = " << current_pos.y << ", pitch = " << pitch << '\n';
        
      }
    }
  }

    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
    {
      flight_status = msg->data;
    }

    void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
    {
      display_mode = msg->data;
    }


    /*!
    * This function demos how to use the flight_status
    * and the more detailed display_mode (only for A3/N3)
    * to monitor the take off process with some error
    * handling. Note M100 flight status is different
    * from A3/N3 flight status.
    */
    bool
    monitoredTakeoff()
    {
      ros::Time start_time = ros::Time::now();

      if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
        return false;
      }

      ros::Duration(0.01).sleep();
      ros::spinOnce();

      // Step 1.1: Spin the motor
      while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
        display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
        ros::Time::now() - start_time < ros::Duration(5)) {
          ros::Duration(0.01).sleep();
          ros::spinOnce();
        }

        if(ros::Time::now() - start_time > ros::Duration(5)) {
          ROS_ERROR("Takeoff failed. Motors are not spinnning.");
          return false;
        }
        else {
          start_time = ros::Time::now();
          ROS_INFO("Motor Spinning ...");
          ros::spinOnce();
        }


        // Step 1.2: Get in to the air
        while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
          }

          if(ros::Time::now() - start_time > ros::Duration(20)) {
            ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
            return false;
          }
          else {
            start_time = ros::Time::now();
            ROS_INFO("Ascending...");
            ros::spinOnce();
          }

          // Final check: Finished takeoff
          while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
          }

          if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
          {
            ROS_INFO("Successful takeoff!");
            start_time = ros::Time::now();
          }
          else
          {
            ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
            return false;
          }

          return true;
        }


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

          float home_altitude = current_gps.altitude;
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
            current_gps.altitude - home_altitude < 1.0)
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
