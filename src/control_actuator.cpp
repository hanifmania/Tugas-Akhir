/* Control HUG which is include :
# Actuator Setpoint Control (Buoyancy Engine (depth), Moving Mass (Pitch), Servo (Yaw), & Thruster (X) ) 
# Attitude Control
*/

#include "ros/ros.h"
//#include <PID_v1.h>
#include <auv_msgs/WorldWaypointReq.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/NavSts.h>
#include <cola2_msgs/Setpoints.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>


// variable PID rudder
float sat_rd_u = 0.5, sat_rd_d = -0.5,
    heading_ref, heading_fb,
    heading_err = 0, heading_sumerr = 0,
    heading_laster, PID_rd_out,
    PID_rd_cal, PID_rd_unsat,
    rd_AW = 0;

float PID_rd_KP = 3,
    PID_rd_KI = 1,
    PID_rd_KD = 0.1,
    PID_rd_AW = 1;

float PID_rd_pro, PID_rd_int, PID_rd_dev;

// variable PID ballast
float sat_blst_u = 0.5, sat_blst_d = -0.5,
    pitch_ref, pitch_fb,
    pitch_err = 0, pitch_sumerr = 0,
    pitch_lasterr, PID_blst_out,
    PID_blst_cal, PID_blst_unsat,
    blst_AW = 0;

float PID_blst_KP = 3,
    PID_blst_KI = 1,
    PID_blst_KD = 0.1,
    PID_blst_AW = 1;

float PID_blst_pro, PID_blst_int, PID_blst_dev;

// variable PID bladder
float sat_bld_u = -0.5, sat_bld_d = 0.5,
    depth_ref, depth_fb,
    depth_err = 0, depth_sumerr = 0,
    depth_lasterr, PID_bld_out,
    PID_bld_cal, PID_bld_unsat,
    bld_AW = 0;

float PID_bld_KP = 20,
    PID_bld_KI = 0.1,
    PID_bld_KD = 0.5,
    PID_bld_AW = 1;

float PID_bld_pro, PID_bld_int, PID_bld_dev;

float temp0,temp1,temp2;
float temp3,temp4,temp5;

std::vector< double > pose_feedback;
std::vector< double > twist_feedback;
std::vector< bool > disabled_axis_wwr;
std::vector<double> values_wwr;
std::vector< bool > disabled_axis_bvr;
std::vector< double > values_bvr;
std::vector< double > ultrasonik_feedback;
std::vector< long > step_feedback;

/*-------------- Variable, Procedure, & ROS tool Declaration ------------------*/

void updateNav(const ros::MessageEvent<auv_msgs::NavSts const> & msg)
{
  // Update pose feedback
  
  pose_feedback.push_back(msg.getMessage()->position.north);
  pose_feedback.push_back(msg.getMessage()->position.east);
  pose_feedback.push_back(msg.getMessage()->position.depth);
  pose_feedback.push_back(msg.getMessage()->orientation.roll);
  pose_feedback.push_back(msg.getMessage()->orientation.pitch);
  pose_feedback.push_back(msg.getMessage()->orientation.yaw);
  //_auv_controller->updatePoseFeedback(pose_feedback);

  // Update twist feedback
  
  //twist_feedback.push_back(msg.getMessage()->body_velocity.x);
  //twist_feedback.push_back(msg.getMessage()->body_velocity.y);
  //twist_feedback.push_back(msg.getMessage()->body_velocity.z);
  //twist_feedback.push_back(msg.getMessage()->orientation_rate.roll);
  //twist_feedback.push_back(msg.getMessage()->orientation_rate.pitch);
  //twist_feedback.push_back(msg.getMessage()->orientation_rate.yaw);
  //_auv_controller->updateTwistFeedback(twist_feedback);

  /*
  // Stores last altitude. If altitude is invalid, during 5 seconds estimate it wrt last altitude and delta depth.
  // If more than 5 seconds put it a 0.5.
  if (msg.getMessage()->altitude > 0.0)
  {
    _last_altitude = msg.getMessage()->altitude;
    _last_altitude_age = msg.getMessage()->header.stamp.toSec();
    _last_depth = msg.getMessage()->position.depth;
  }
  else
  {
    if ((ros::Time::now().toSec() - _last_altitude_age)  > 5.0)
    {
      _last_altitude = 0.5;
    }
    else
    {
      _last_altitude = _last_altitude - (msg.getMessage()->position.depth - _last_depth);
      _last_depth = msg.getMessage()->position.depth;
    }
  }*/
}

void updateWWR(const ros::MessageEvent<auv_msgs::WorldWaypointReq const> & msg)
{
  // Init request
  /*Request req(msg.getMessage()->goal.requester,
              msg.getMessage()->header.stamp.toSec(),
              msg.getMessage()->goal.priority,
              6);
  */
  // Set disable axis
  
  disabled_axis_wwr.push_back(msg.getMessage()->disable_axis.x);
  disabled_axis_wwr.push_back(msg.getMessage()->disable_axis.y);
  disabled_axis_wwr.push_back(msg.getMessage()->disable_axis.z);
  disabled_axis_wwr.push_back(msg.getMessage()->disable_axis.roll);
  disabled_axis_wwr.push_back(msg.getMessage()->disable_axis.pitch);
  disabled_axis_wwr.push_back(msg.getMessage()->disable_axis.yaw);
  //req.setDisabledAxis(disabled_axis);

  // Set values
  
  values_wwr.push_back(msg.getMessage()->position.north);
  values_wwr.push_back(msg.getMessage()->position.east);
  values_wwr.push_back(msg.getMessage()->position.depth);
  

  values_wwr.push_back(msg.getMessage()->orientation.roll);
  values_wwr.push_back(msg.getMessage()->orientation.pitch);
  values_wwr.push_back(msg.getMessage()->orientation.yaw);
  //req.setValues(values); */

  // Add request to controller ptr.
  //_auv_controller->updatePoseRequest(req_bvr_bvr);
}

void updateBVR(const ros::MessageEvent<auv_msgs::BodyVelocityReq const> & msg)
{
  // Init request
  /*Request req(msg.getMessage()->goal.requester,
              msg.getMessage()->header.stamp.toSec(),
              msg.getMessage()->goal.priority,
              6);
  */
  // Set disable axis
  
  disabled_axis_bvr.push_back(msg.getMessage()->disable_axis.x);
  disabled_axis_bvr.push_back(msg.getMessage()->disable_axis.y);
  disabled_axis_bvr.push_back(msg.getMessage()->disable_axis.z);
  disabled_axis_bvr.push_back(msg.getMessage()->disable_axis.roll);
  disabled_axis_bvr.push_back(msg.getMessage()->disable_axis.pitch);
  disabled_axis_bvr.push_back(msg.getMessage()->disable_axis.yaw);
  //req.setDisabledAxis(disabled_axis);

  // Set values
  
  values_bvr.push_back(msg.getMessage()->twist.linear.x);
  values_bvr.push_back(msg.getMessage()->twist.linear.y);
  values_bvr.push_back(msg.getMessage()->twist.linear.z);
  values_bvr.push_back(msg.getMessage()->twist.angular.x);
  values_bvr.push_back(msg.getMessage()->twist.angular.y);
  values_bvr.push_back(msg.getMessage()->twist.angular.z);
  //req.setValues(values);

  // Add request to controller ptr.
  //_auv_controller->updateTwistRequest(req);
}

void updateUltrasonik(const ros::MessageEvent<std_msgs::Float64 const> & msg)
{
  // Update pose feedback
  
  ultrasonik_feedback.push_back(msg.getMessage()->data);
}

void updateStep(const ros::MessageEvent<std_msgs::UInt16 const> & msg)
{
  // Update pose feedback
  
  step_feedback.push_back(msg.getMessage()->data);
}

/*
//Subscriber 
sub_nav_data = n.subscribe("/cola2_navigation/nav_sts", 2, updateNav);
sub_ww_req = n.subscribe("/cola2_control/world_waypoint_req", 10, updateWWR);
sub_bv_req = n.subscribe("/cola2_control/body_velocity_req", 10, updateBVR);
sub_feedback_actuator = n.subscribe("/cola2_control/feedback_actuator", 10, updateFeedbackActuator);

//Publisher
pub_actuator_setpoint = n.advertise<cola2_msgs::Setpoints>("/cola2_control/actuator_data", 1);
*/

/*
void init_node() 
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;
    ros::Subscriber sub_nav_data;
    ros::Subscriber sub_ww_req;
    ros::Subscriber sub_bv_req;
    ros::Subscriber sub_feedback_actuator;

    ros::Publisher pub_actuator_setpoint;
}
*/
/*
void pitch_controller() 
{
  pitch_ref = 0;
  pitch_fb = temp_pitch;
  pitch_lasterr = pitch_err;
  pitch_err = pitch_ref - pitch_fb;
  pitch_sumerr = pitch_sumerr + pitch_err + blst_AW;

  PID_blst_pro = pitch_err * PID_blst_KP;
  PID_blst_int = pitch_sumerr * PID_blst_KI * sampling_time;
  PID_blst_dev = (pitch_err - pitch_lasterr) * PID_blst_KD / sampling_time;
  PID_blst_cal = PID_blst_pro + PID_blst_int + PID_blst_dev;

  if(PID_blst_cal > sat_blst_u) {
    PID_blst_out = sat_blst_u;
  }

  else if(PID_blst_cal < sat_blst_d) {
    PID_blst_out = sat_blst_d;
  }

  else {
    PID_blst_out = PID_blst_cal;
  }

  blst_AW = (PID_blst_out - PID_blst_cal) * PID_blst_AW;
  blst_PID_val = PID_blst_out;
}

void depth_controller()
{
  depth_ref = 0;
  depth_fb = temp_depth;
  depth_lasterr = depth_err;
  depth_err = depth_err - depth_fb;
  depth_sumerr = depth_sumerr + depth_err + bld_AW;

  PID_bld_pro = depth_err * PID_bld_KP;
  PID_bld_int = depth_sumerr * PID_bld_KI * sampling_time;
  PID_bld_dev = (depth_err - depth_lasterr) * PID_bld_KD / sampling_time;
  PID_bld_cal = PID_bld_pro + PID_bld_int + PID_bld_dev;

  if(PID_bld_cal > sat_bld_u) {
    PID_bld_out = sat_bld_u;
  }

  else if(PID_bld_cal < sat_bld_d) {
    PID_bld_out = sat_bld_d;
  }

  else {
    PID_bld_out = PID_bld_cal;
  }

  bld_AW = (PID_bld_out - PID_bld_cal) * PID_bld_AW;
  bld_PID_val = PID_bld_out;
}

void yaw_controller()
{
  heading_ref = yawref;
  heading_fb = temp_yaw;
  heading_ref = heading_ref * 3.1416 / 180.0;
  heading_fb = heading_fb * 3.1416 / 180.0;
  heading_laster = heading_err;
  heading_err = heading_ref - heading_fb;
  heading_sumerr = heading_sumerr + heading_err + rd_AW;

  PID_rd_pro = heading_err * PID_rd_KP;
  PID_rd_int = heading_sumerr * PID_rd_KI * sampling_time;
  PID_rd_dev = (heading_err - heading_laster) * PID_rd_KD / sampling_time;
  PID_rd_cal = PID_rd_pro + PID_rd_int + PID_rd_dev;

  if (PID_rd_cal > sat_rd_u) {
    PID_rd_out = sat_rd_u;
  } 

  else if (PID_rd_cal < sat_rd_d) {
    PID_rd_out = sat_rd_d;    
  }

  else {
    PID_rd_out = PID_rd_cal;
  }

  rd_AW = (PID_rd_out - PID_rd_cal) * PID_rd_AW;
  rd_PID_val = PID_rd_out;
}
*/


int main(int argc, char **argv)
{
    //init_node();
    ros::init(argc, argv, "controller_oke");
    ros::NodeHandle n;
    ros::Subscriber sub_nav_data = n.subscribe("/cola2_navigation/nav_sts", 2, updateNav);
    ros::Subscriber sub_ww_req = n.subscribe("/cola2_control/world_waypoint_req", 10, updateWWR);
    ros::Subscriber sub_bv_req = n.subscribe("/cola2_control/body_velocity_req", 10, updateBVR);
    ros::Subscriber sub_ultrasonik_feedback = n.subscribe("/cola2_control/feedback_ultrasonik", 10, updateUltrasonik);
    ros::Subscriber sub_step_feedback = n.subscribe("/cola2_control/feedback_step", 10, updateStep);

    ros::Publisher pub_actuator_setpoint = n.advertise<cola2_msgs::Setpoints>("/cola2_control/actuator_data", 1);
    
    //pitch_controller();
    //depth_controller();
    //yaw_controller();
    temp0 = pose_feedback.at(0);
    temp1 = pose_feedback.at(1);
    temp2 = pose_feedback.at(2);
    temp3 = pose_feedback.at(3);
    temp4 = pose_feedback.at(4);
    temp5 = pose_feedback.at(5);

    ROS_INFO("Datanya adalah : %f, %f,%f, %f,%f, %f ", temp0 , temp1, temp2, temp3, temp4, temp5) ;






    ros::spin();
    return 0;

}