/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Irstea nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <cmath>
#include <four_wheel_steering_controller/four_wheel_steering_controller.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <tf2/LinearMath/Matrix3x3.h>

namespace four_wheel_steering_controller{

  FourWheelSteeringController::FourWheelSteeringController()
    : command_struct_twist_()
    , track_(0.0)// distance between left and right wheels (from the midpoint of the wheel width)
    , wheel_steering_y_offset_(0.0)
    , wheel_radius_(0.0)
    , wheel_base_(0.0)  // (distance between front and rear wheel)
    , cmd_vel_timeout_(0.5)
    , base_frame_id_("base_link")
    , enable_odom_tf_(false)
    , enable_twist_cmd_(true)
    , linear_x_acc_(RollingWindow::window_size = 20 )
  , angular_z_acc_(RollingWindow::window_size = 20 )
  {
  }

  bool FourWheelSteeringController::init( ros::NodeHandle& root_nh , XmlRpc::XmlRpcValue initial_odom_list)
  {

    const std::string complete_ns = root_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");

    name_ = complete_ns.substr(id + 1);
      ROS_ERROR_STREAM_NAMED(name_,
          "#root_nh NAMESPACE : " << root_nh.getNamespace());

    bool found_name = root_nh.getParam("/rover_name", rover_name);
    if ( !found_name )
    {
      ROS_ERROR_STREAM( "rover_name not found, check namespace.");
      return 1;
    }
    joint_state_topic_name = "/"+rover_name+"/joint_states" ;
    sub_joint_ = root_nh.subscribe(joint_state_topic_name, 1, &FourWheelSteeringController::getJointState, this);
    //   ROS_ERROR_STREAM( "+++++++++++++++++++++++"<< joint_state_topic_name);
    fl_wheel_pub    = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/fl_wheel_controller/command", 1);
    fr_wheel_pub    = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/fr_wheel_controller/command", 1);
    bl_wheel_pub    = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/bl_wheel_controller/command", 1);
    br_wheel_pub    = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/br_wheel_controller/command", 1);
    fl_steering_pub = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/fl_steering_arm_controller/command", 1);
    fr_steering_pub = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/fr_steering_arm_controller/command", 1);
    bl_steering_pub = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/bl_steering_arm_controller/command", 1);
    br_steering_pub = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/br_steering_arm_controller/command", 1);


    wheel_odom_publisher = root_nh.advertise<nav_msgs::Odometry>("/"+rover_name+"/wheel_odom", 1);
    fl_ang_vel_setpoint = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/fl_axle/setpoint", 1);
    fr_ang_vel_setpoint = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/fr_axle/setpoint", 1);
    bl_ang_vel_setpoint = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/bl_axle/setpoint", 1);
    br_ang_vel_setpoint = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/br_axle/setpoint", 1);
    fl_axle_state       = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/fl_axle/state", 1);
    fr_axle_state       = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/fr_axle/state", 1);
    bl_axle_state       = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/bl_axle/state", 1);
    br_axle_state       = root_nh.advertise<std_msgs::Float64>("/"+rover_name+"/br_axle/state", 1);

    // Odometry related:
    double publish_rate;
    root_nh.param("publish_rate", publish_rate, 50.0);
    ROS_DEBUG_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    root_nh.param("open_loop", open_loop_, open_loop_);

    int velocity_rolling_window_size = 10;
    root_nh.param("velocity_rolling_window_size", velocity_rolling_window_size, velocity_rolling_window_size);
    ROS_DEBUG_STREAM_NAMED(name_, "Velocity rolling window size of "
                          << velocity_rolling_window_size << ".");

    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);

    // Twist command related:
    root_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_DEBUG_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                          << cmd_vel_timeout_ << "s.");

    root_nh.param("base_frame_id", base_frame_id_, base_frame_id_);
    ROS_DEBUG_STREAM_NAMED(name_, "Base frame_id set to " << base_frame_id_);

    root_nh.param("enable_odom_tf", enable_odom_tf_, enable_odom_tf_);
    ROS_DEBUG_STREAM_NAMED(name_, "Publishing to tf is " << (enable_odom_tf_?"enabled":"disabled"));
    // Velocity and acceleration limits:
    root_nh.param("linear/x/has_velocity_limits"    , limiter_lin_.has_velocity_limits    , limiter_lin_.has_velocity_limits    );
    root_nh.param("linear/x/has_acceleration_limits", limiter_lin_.has_acceleration_limits, limiter_lin_.has_acceleration_limits);
    root_nh.param("linear/x/max_velocity"           , limiter_lin_.max_velocity           ,  limiter_lin_.max_velocity          );
    root_nh.param("linear/x/min_velocity"           , limiter_lin_.min_velocity           , -limiter_lin_.max_velocity          );
    root_nh.param("linear/x/max_acceleration"       , limiter_lin_.max_acceleration       ,  limiter_lin_.max_acceleration      );
    root_nh.param("linear/x/min_acceleration"       , limiter_lin_.min_acceleration       , -limiter_lin_.max_acceleration      );
    
    root_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    root_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    root_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    root_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    root_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    root_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );
    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_track = !root_nh.getParam("track", track_);
    bool lookup_wheel_radius = !root_nh.getParam("wheel_radius", wheel_radius_);
    bool lookup_wheel_base = !root_nh.getParam("wheel_base", wheel_base_);

    // Regardless of how we got the separation and radius, use them
    // to set the odometry parameters
    odometry_.setWheelParams(track_-2*wheel_steering_y_offset_, wheel_steering_y_offset_, wheel_radius_, wheel_base_);


    ROS_DEBUG_STREAM_NAMED(name_,
                          "Odometry params : track " << track_
                          << ", wheel radius " << wheel_radius_
                          << ", wheel base " << wheel_base_
                          << ", wheel steering offset " << wheel_steering_y_offset_);

    setOdomPubFields(root_nh , initial_odom_list);

    fl_ctrl_eff = root_nh.subscribe("/"+rover_name+"/fl_axle/control_effort", 1, &FourWheelSteeringController::fl_ctrlEffCallback, this); 
    fr_ctrl_eff = root_nh.subscribe("/"+rover_name+"/fr_axle/control_effort", 1, &FourWheelSteeringController::fr_ctrlEffCallback, this); 
    bl_ctrl_eff = root_nh.subscribe("/"+rover_name+"/bl_axle/control_effort", 1, &FourWheelSteeringController::bl_ctrlEffCallback, this); 
    br_ctrl_eff = root_nh.subscribe("/"+rover_name+"/br_axle/control_effort", 1, &FourWheelSteeringController::br_ctrlEffCallback, this); 

    sub_command_ = root_nh.subscribe("/"+rover_name+"/cmd_vel", 1, &FourWheelSteeringController::cmdVelCallback, this);
    /*
    sub_vis_ = root_nh.subscribe("/rtabmap/visual_odom"    , 1, &FourWheelSteeringController::visCallback, this); 
    */
    sub_ekf_  = root_nh.subscribe("/"+rover_name+"/ekf_odom"  , 1, &FourWheelSteeringController::ekfCallback, this); 
    sub_imu_ = root_nh.subscribe("/"+rover_name+"/imu_rect" , 1, &FourWheelSteeringController::imuCallback, this); 

    return true;
  }





  /*

  MAIN CONTROLLER LOOP
  
  */


  void FourWheelSteeringController::update()
  {
    internal_time = ros::Time(0) ;
    starting( internal_time );

    ros::Rate r(20);
    ros::Time begin = ros::Time::now();
    ros::Time end = ros::Time::now();
    double elapsed_secs = 0;
    const ros::Duration& period = getPeriod();

    while( ros::ok() )
    {
        begin = ros::Time::now();

        updateOdometry(internal_time);
        updateCommand(internal_time, period);

        end = ros::Time::now();

        elapsed_secs = (begin-end).toSec();
        if (period.toSec() - elapsed_secs < 0.0)
        {
          ROS_ERROR_STREAM_THROTTLE(
                0.1, "Control cycle is taking to much time, elapsed: " << elapsed_secs);
        }
        else
        {
          ROS_DEBUG_STREAM_THROTTLE(1.0, "Control cycle is, elapsed: " << elapsed_secs);
          ros::Duration(period.toSec() - elapsed_secs).sleep();
        }

        internal_time += period;
        ros::spinOnce();
        r.sleep();
    }

  }







  void FourWheelSteeringController::starting(const ros::Time& time)
  {
    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;
    odometry_.init(time);

    
  }








  void FourWheelSteeringController::updateOdometry(const ros::Time& time)
  {
    // COMPUTE AND PUBLISH ODOMETRY
    //ROS_ERROR_STREAM( ""<< joint_map[ state.name[counter] ]["pos"]);
    //ROS_ERROR_STREAM( ""<< joint_map[ state.name[counter] ]["vel"]);
    //ROS_ERROR_STREAM( ""<< joint_map[ state.name[counter] ]["eff"]);
    //ROS_ERROR_STREAM( ""<< joint_map[ state.name[counter] ]["lin_vel"]);
    /*
            bl_arm_joint , "bl_steering_arm_joint" , "bl_wheel_joint" ,
            br_arm_joint , "br_steering_arm_joint" , "br_wheel_joint" ,
            fl_arm_joint , "fl_steering_arm_joint" , "fl_wheel_joint" ,
            fr_arm_joint , "fr_steering_arm_joint" , "fr_wheel_joint" ,
            sensor_joint ;
    */
 //ADDON#####################################################################################
    double fl_speed = joint_map[ "fl_wheel_joint" ]["vel"] ;// replaces-> //front_wheel_joints_[0].getVelocity();
    double fr_speed = joint_map[ "fr_wheel_joint" ]["vel"] ;// replaces->//front_wheel_joints_[1].getVelocity();
    double rl_speed = joint_map[ "bl_wheel_joint" ]["vel"] ;// replaces->//rear_wheel_joints_[0].getVelocity();
    double rr_speed = joint_map[ "br_wheel_joint" ]["vel"] ;// replaces->//rear_wheel_joints_[1].getVelocity();
    if (std::isnan(fl_speed) || std::isnan(fr_speed)
        || std::isnan(rl_speed) || std::isnan(rr_speed))
      return;

    ROS_DEBUG_STREAM_THROTTLE(.5, "AFTER VELOCITY difference : "<<
    "fl_speed "<<fl_speed<<" fr_speed "<<fr_speed<<" rl_speed "<<rl_speed<<" rr_speed "<<rr_speed);

    // Estimate linear and angular velocity using joint information
    bool USING_FILE_WRITER = true ;
    if( USING_FILE_WRITER ){
      // for training and eventually use of error correction
      bool isExplicitSteer = steerState == EXPLICIT;

      joint_map_mutex.lock();
      std::map<std::string, std::map< std::string , double >  > jm(joint_map) ;
      joint_map_mutex.unlock();

      float nn_deltas[4] = {0,0,0,0};
      //if(delta_x_mutex.try_lock()){ nn_deltas[0]= delta_x; delta_x_mutex.unlock();}
      //if(delta_y_mutex.try_lock()){ nn_deltas[1]= delta_y; delta_y_mutex.unlock();}
      //if(delta_z_mutex.try_lock()){ nn_deltas[2]= delta_z; delta_z_mutex.unlock();}
      //if(delta_yaw_mutex.try_lock()){ nn_deltas[3]= delta_yaw; delta_yaw_mutex.unlock();}

      jm["cmd_vel"]["ang"] = command_struct_twist_.ang ;
      odometry_.trainedSkidIntegration( isExplicitSteer ,
                               &jm ,
                               &poses ,
                               time ,
                               &poses_mutex , nn_deltas);

    } else if( steerState == EXPLICIT ){
      odometry_.update_for_explicit_steer(fl_speed,
                                          fr_speed,
                                          rl_speed,
                                          rr_speed,
                                          time);
    } else {
      const double fl_steering = joint_map[ "fl_steering_arm_joint" ]["pos"] ;// replaces-> //front_steering_joints_[0].getPosition();
      const double fr_steering = joint_map[ "fr_steering_arm_joint" ]["pos"] ;// replaces-> //front_steering_joints_[1].getPosition();
      const double rl_steering = joint_map[ "bl_steering_arm_joint" ]["pos"] ;// replaces-> //rear_steering_joints_[0].getPosition();
      const double rr_steering = joint_map[ "br_steering_arm_joint" ]["pos"] ;// replaces-> //rear_steering_joints_[1].getPosition();
      if (std::isnan(fl_steering) || std::isnan(fr_steering)
          || std::isnan(rl_steering) || std::isnan(rr_steering))
        return;
      double front_steering_pos = 0.0;
      if(fabs(fl_steering) > 0.001 || fabs(fr_steering) > 0.001)
      {
        front_steering_pos = atan(2*tan(fl_steering)*tan(fr_steering)/
                                        (tan(fl_steering) + tan(fr_steering)));
      }
      double rear_steering_pos = 0.0;
      if(fabs(rl_steering) > 0.001 || fabs(rr_steering) > 0.001)
      {
        rear_steering_pos = atan(2*tan(rl_steering)*tan(rr_steering)/
                                       (tan(rl_steering) + tan(rr_steering)));
      }

      // In order to match the 4ws getVelocity function I had to divide by wheel_radius
      // It really shouldn't be that way and I'm aware of what values It should be
      // For example consider that :
      // 1.
      //    joint_map[ "fl_wheel_joint" ]["vel"]   holds the angular velocity of the joint 
      //    SEE -- getJointState(...) in four_wheel_steering_controller.h
      // 2.
      //    getVelocity() -- for ROS velocity joint -- should return the angular velocity of the joint
      //    SEE -- update(...) in odometry.h , "\param fl_speed front left wheel vehicle speed [rad/s]"
      //    SEE -- http://docs.ros.org/jade/api/hardware_interface/html/c++/joint__state__interface_8h_source.html
      // -------------------------------------------------------------------------------------------
      // This is all assuming NASA hasn't changed it from when I was last working on my own odometry
      // functions in python. But the explicit calculations suggest NASA has not changed it.
      // matching_4ws_getVelocity gets the functions closer to ground_truth but it is not exact.
      double matching_4ws_getVelocity = 1/wheel_radius_ ;  // ctrl-f OTHERFORMULACHANGE
      odometry_.update(fl_speed*matching_4ws_getVelocity, fr_speed*matching_4ws_getVelocity, rl_speed*matching_4ws_getVelocity, rr_speed*matching_4ws_getVelocity,
                     front_steering_pos, rear_steering_pos, command_struct_twist_.ang ,poses["imu_cur_rol"],poses["imu_cur_pit"], time);

    }

    ROS_DEBUG_STREAM_THROTTLE(.1, "HEADING WHEELODOM : "<<odometry_.getHeading() << " VISUAL : "<< yaw_vis << " IMU : " << poses["imu_cur_yaw"] );

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      double covariance_diagonal = 1e-6 ;
      bool init_odom_is_norm = 20 < time.toSec(); //simple_measure_that init odom is stable
      if( increase_covariance_for_slow_stop && init_odom_is_norm ){
        was_slow_stopping = true ;
        covariance_diagonal = 1000;//std::numeric_limits<float>::max() ;
      } else {
        if( was_slow_stopping && init_odom_is_norm ){
            poses_mutex.lock();
            was_slow_stopping = false ;
            odometry_.set_xyzheading( poses["ekf_cur_x"] ,
                                        poses["ekf_cur_y"] ,
                                        poses["ekf_cur_z"] ,
                                        poses["ekf_cur_yaw"] );
            poses_mutex.unlock();
        }
      }
        odom_pub_->msg_.pose.covariance = {
        covariance_diagonal, 0., 0., 0., 0., 0.,
        0., covariance_diagonal, 0., 0., 0., 0.,
        0., 0., covariance_diagonal, 0., 0., 0.,
        0., 0., 0., covariance_diagonal, 0., 0.,
        0., 0., 0., 0., covariance_diagonal, 0.,
        0., 0., 0., 0., 0., covariance_diagonal };

      // Populate odom message and publish
      if ( odom_pub_->trylock()) // false )//
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.position.z = odometry_.getZ();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinearX();
        odom_pub_->msg_.twist.twist.linear.y  = odometry_.getLinearY();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }

    }
  }













  void FourWheelSteeringController::updateCommand(const ros::Time& time, const ros::Duration& period)
  {
    boost::lock_guard<boost::mutex> guard( joint_map_mutex ) ; // runs thread safety through the RAII idiom
    // Retreive current velocity command and time step:
    Command* cmd = &command_struct_twist_ ;
    CommandTwist curr_cmd_twist = command_struct_twist_;//*(command_twist_.readFromRT());

    const double dt = (time - cmd->stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd_twist.lin_x = 0.0;
      curr_cmd_twist.lin_y = 0.0;
      curr_cmd_twist.ang = 0.0;

    }

    const double cmd_dt(period.toSec());

    const double angular_speed = odometry_.getAngular(); // [rad/s]
    const double steering_track = track_-2*wheel_steering_y_offset_;

    ROS_DEBUG_STREAM("angular_speed "<<angular_speed<< " wheel_radius_ "<<wheel_radius_);
    double vel_left_front = 0, vel_right_front = 0;
    double vel_left_rear = 0, vel_right_rear = 0;
    double front_left_steering = 0, front_right_steering = 0;
    double rear_left_steering = 0, rear_right_steering = 0;


    if( steerState == EXPLICIT ){
      double rover_center_to_axle ,tangent_slope , dot_product , slopey_norm , tangent_angle ;
      rover_center_to_axle = std::hypot( wheel_base_ / 2,track_);

      tangent_slope = -1 * (wheel_base_ / 2) /
                          (sqrt( pow(rover_center_to_axle,2) - pow(wheel_base_ / 2, 2)));
      // start cos angle law
      double standard[] = {1, 0};
      double slopey[] = {1, tangent_slope};
      dot_product = standard[0]*slopey[0]+standard[1]*standard[1];
      slopey_norm = sqrt(pow(slopey[0],2)+pow(slopey[1],2));
      tangent_angle = acos( dot_product / ( slopey_norm ) ); 
      // end cos angle law

      // define direction to turn and cap vel at effort of ??
      const double SLIP_SAFE_VEL = 0.50 ;// m/s
      const double sign = copysign(1, curr_cmd_twist.ang) ;
      double xspeed ;
      if( SLIP_SAFE_VEL < abs(joint_map[ "fr_wheel_joint" ]["lin_vel"]) ){
        xspeed= abs(.10) ; // in case it is switching to xplicit
      } else {
        xspeed= abs(curr_cmd_twist.lin_x) ; // in case it is switching to xplicit
      }
      vel_left_front    = -xspeed*sign ;
      vel_right_front   =  xspeed*sign ;
      vel_left_rear     = -xspeed*sign ;
      vel_right_rear    =  xspeed*sign ;
      front_left_steering  =  -1*tangent_angle;
      front_right_steering =     tangent_angle;
      rear_left_steering   =     tangent_angle;
      rear_right_steering  =  -1*tangent_angle;

    } else {
      // four wheel steer 
      if(enable_twist_cmd_ == true)
      {
        // Limit velocities and accelerations:
        limiter_lin_.limit(curr_cmd_twist.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
        limiter_ang_.limit(curr_cmd_twist.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);
        last1_cmd_ = last0_cmd_;
        last0_cmd_ = curr_cmd_twist;

        // Compute wheels velocities:
        if(fabs(curr_cmd_twist.lin_x) > 0.001 && fabs(curr_cmd_twist.ang) > 0.001)
        { // The removal of division by wheel_radius here is because the stock ROS package expects lin_x
          // to represent a desired velocity // I've tried to see if this is why I had to 
          // divide the angular vel in updateOdometry but it didn't seem like it was so
          // This represents one of the few changes I made to the formulas. ctrl-f OTHERFORMULACHANGE
          // vel_steering_offset == 0                     b.c. of wheel_steering_y_offset
          const double vel_steering_offset = (curr_cmd_twist.ang*wheel_steering_y_offset_) /wheel_radius_;
          const double sign = copysign(1.0, curr_cmd_twist.lin_x);
          vel_left_front  = sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),
                                              (wheel_base_*curr_cmd_twist.ang/2.0)) // / wheel_radius_
                            - vel_steering_offset;
          vel_right_front = sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),
                                              (wheel_base_*curr_cmd_twist.ang/2.0)) // / wheel_radius_
                            + vel_steering_offset;
          vel_left_rear = sign * std::hypot((curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track/2),
                                            (wheel_base_*curr_cmd_twist.ang/2.0)) // / wheel_radius_
                          - vel_steering_offset;
          vel_right_rear = sign * std::hypot((curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track/2),
                                             (wheel_base_*curr_cmd_twist.ang/2.0)) // / wheel_radius_
                           + vel_steering_offset;
        } else if(fabs(curr_cmd_twist.lin_x) > 0.001 )
        {

          vel_left_front  = curr_cmd_twist.lin_x;
          vel_right_front = curr_cmd_twist.lin_x;
          vel_left_rear   = curr_cmd_twist.lin_x;
          vel_right_rear  = curr_cmd_twist.lin_x;
        } else 
        {
          vel_left_front  = 0.0;
          vel_right_front = 0.0;
          vel_left_rear   = 0.0;
          vel_right_rear  = 0.0;
        }

        // Compute steering angles
        if(fabs(2.0*curr_cmd_twist.lin_x) > fabs(curr_cmd_twist.ang*steering_track))
        {
          front_left_steering = atan(curr_cmd_twist.ang*wheel_base_ /
                                      (2.0*curr_cmd_twist.lin_x - curr_cmd_twist.ang*steering_track));
          front_right_steering = atan(curr_cmd_twist.ang*wheel_base_ /
                                       (2.0*curr_cmd_twist.lin_x + curr_cmd_twist.ang*steering_track));
        }
        else if(fabs(curr_cmd_twist.lin_x) > 0.001)
        {
          front_left_steering = copysign(M_PI_2, curr_cmd_twist.ang);
          front_right_steering = copysign(M_PI_2, curr_cmd_twist.ang);
        }
        //front_left_steering =  curr_cmd_twist.ang;
        //front_right_steering = curr_cmd_twist.ang;
        rear_left_steering  = -front_left_steering;//0;//
        rear_right_steering = -front_right_steering;//0;//
      }

    } 
    if( stateSwitchTimeDone() ){
      SWITCHING_STATES = false ; 
  }else if( SWITCHING_STATES && !stateSwitchTimeDone() ){
    vel_left_front  = 0.0;
    vel_right_front = 0.0;
    vel_left_rear   = 0.0;
    vel_right_rear  = 0.0;
  }
    ROS_DEBUG_STREAM_THROTTLE(1, "vel_left_rear "<<vel_left_rear<<" front_right_steering "<<front_right_steering);


    // Set wheels velocities:

      // publish pid information to let the controller know what to do
      double fl_speed = joint_map[ "fl_wheel_joint" ]["vel"] ;
      double fr_speed = joint_map[ "fr_wheel_joint" ]["vel"] ;
      double rl_speed = joint_map[ "bl_wheel_joint" ]["vel"] ;
      double rr_speed = joint_map[ "br_wheel_joint" ]["vel"] ;

      publishFromPointer( &fl_axle_state       , fl_speed  );
      publishFromPointer( &fr_axle_state       , fr_speed  );
      publishFromPointer( &bl_axle_state       , rl_speed  );
      publishFromPointer( &br_axle_state       , rr_speed  );
      double min_vel = 0 ;
      // nasa usually has the front axles spinning faster
      if( fabs(fr_speed) < fabs(rr_speed) ){ 
        publishFromPointer( &br_ang_vel_setpoint , fr_speed  );
        min_vel = fr_speed ;
      } else if ( fabs(rr_speed) < fabs(fr_speed) ){
        publishFromPointer( &fr_ang_vel_setpoint , rr_speed  );
        min_vel = rr_speed ;
      }
      double left_right_ratio = vel_left_front/( vel_right_front + std::numeric_limits<double>::min() );
      publishFromPointer( &bl_ang_vel_setpoint , min_vel*( left_right_ratio )  );
      publishFromPointer( &fl_ang_vel_setpoint , min_vel*( left_right_ratio )  );

      // publish joint effort
      original_fl_effort = vel_left_front  + joint_map["fl_wheel_joint"]["pid_ctrl_eff"];
      original_fr_effort = vel_right_front + joint_map["fr_wheel_joint"]["pid_ctrl_eff"];
      original_rl_effort = vel_left_rear   + joint_map["bl_wheel_joint"]["pid_ctrl_eff"];
      original_rr_effort = vel_right_rear  + joint_map["br_wheel_joint"]["pid_ctrl_eff"];

      publishFromPointer( &fl_wheel_pub , original_fl_effort  );
      publishFromPointer( &fr_wheel_pub , original_fr_effort  );
      publishFromPointer( &bl_wheel_pub , original_rl_effort  );
      publishFromPointer( &br_wheel_pub , original_rr_effort  );
      /* // as far as I can tell  these functions are
        // not available in with nasa's premade hardware
          UPDATED NOTE :
          // as of june 22 I now believe it would've been possible
          // to get the joint in another way I didnt try previously

      front_wheel_joints_[0].setCommand( vel_left_front);
      front_wheel_joints_[1].setCommand( vel_right_front);
      rear_wheel_joints_[0].setCommand(  vel_left_rear);
      rear_wheel_joints_[1].setCommand(  vel_right_rear);
      */
      publishFromPointer( &fl_steering_pub , front_left_steering  );
      publishFromPointer( &fr_steering_pub , front_right_steering );
      publishFromPointer( &bl_steering_pub , rear_left_steering   );
      publishFromPointer( &br_steering_pub , rear_right_steering  );

      /*// as far as I can tell  these functions are
        // not available in with nasa's premade hardware
          UPDATED NOTE :
          // as of june 22 I now believe it would've been possible
          // to get the joint in another way I didnt try previously

      front_steering_joints_[0].setCommand( front_left_steering);
      front_steering_joints_[1].setCommand( front_right_steering);
      rear_steering_joints_[0].setCommand(  rear_left_steering);
      rear_steering_joints_[1].setCommand(  rear_right_steering);
      */



      // THE FOLLOWING MAY BE USEFUL TO MAKE THE WHEELS THE RIGHT VELOCITY W/O a PID

     ROS_DEBUG_STREAM_THROTTLE(2, "BEFORE VELOCITY difference : \n\t"<<
       SWITCHING_STATES << "STate Switch Time result: "<< stateSwitchTimeDone() 
       << "STEER STATE : "<< steerState
    );

     ROS_DEBUG_STREAM_THROTTLE(.5, "BEFORE VELOCITY difference : \n\t"<<
       "\n\tfl_speed "<<joint_map[ "fl_wheel_joint" ]["vel"]
    <<"\n\t fr_speed "<<joint_map[ "fr_wheel_joint" ]["lin_vel"]
    <<"\n\t rl_speed "<<joint_map[ "bl_wheel_joint" ]["vel"]
    <<"\n\t rr_speed "<<joint_map[ "br_wheel_joint" ]["vel"]
    <<"\nBEFORE eff difference : \n\t"<<
       "\n\tfl_eff adj. : "<<joint_map["fl_wheel_joint"]["pid_ctrl_eff"]
    <<"\n\t fr_eff adj. : "<<joint_map["fr_wheel_joint"]["pid_ctrl_eff"]
    <<"\n\t rl_eff adj. : "<<joint_map["bl_wheel_joint"]["pid_ctrl_eff"]
    <<"\n\t rr_eff adj. : "<<joint_map["br_wheel_joint"]["pid_ctrl_eff"]
    << "\n\tODOM_YAW : " << odometry_.getZ() 
    << "\n\timu_cur_rol : " << poses["imu_cur_rol"] 
    << "\n\timu_cur_pit : " << poses["imu_cur_pit"] 
    << "\n\timu_cur_yaw : " << poses["imu_cur_yaw"] 

    );


      }







  void FourWheelSteeringController::cmdVelCallback(const geometry_msgs::Twist& command)
  {

    if(std::isnan(command.angular.z) || std::isnan(command.linear.x))
    {
      ROS_WARN("Received NaN in geometry_msgs::Twist. Ignoring command.");
      return;
    }
    // CHANGE amplify_lin_x only to amplify the move_base
    int amplify_lin_x = 100 ; // eff or 100 usually gets us to 1.5 m/s // max vel of move_base is 1.5 m/s
    int amplify_ang = 1 ;

    command_struct_twist_.explicit_steer_on   = command.angular.x; // 1 or 0 
    command_struct_twist_.ang     = command.angular.z;
    // in case linear y is being outputted, lets just use that for linear x also.
    command_struct_twist_.lin_x   = command.linear.x * amplify_lin_x + command.linear.y * copysign(1,command.linear.x);
    command_struct_twist_.lin_y   = command.linear.y;
    command_struct_twist_.stamp = ros::Time::now();

    increase_covariance_for_slow_stop = command.angular.y == 1.0 ;

    // accumulate 
    linear_x_acc_( command_struct_twist_.lin_x );
    angular_z_acc_( command_struct_twist_.ang );


	setSteerState( (double)(command_struct_twist_.ang) ) ;

	if( 1 < abs(command_struct_twist_.lin_x) ){
		amplify_ang *= abs(command_struct_twist_.lin_x) ;
	}
	command_struct_twist_.ang     = command.angular.z * amplify_ang;

    //command_twist_.writeFromNonRT (command_struct_twist_);
    ROS_DEBUG_STREAM_NAMED(name_,
                           "Added values to command. "
                           << "Ang: "   << command_struct_twist_.ang << ", "
                           << "Lin x: " << command_struct_twist_.lin_x << ", "
                           << "Lin y: " << command_struct_twist_.lin_y << ", "
                           << "Stamp: " << command_struct_twist_.stamp);


  }



















  void FourWheelSteeringController::setOdomPubFields(ros::NodeHandle& root_nh ,
                                                     XmlRpc::XmlRpcValue initial_odom_list)
  {
    odometry_.set_xyzheading( static_cast<double>(initial_odom_list[0]) ,
                             static_cast<double>(initial_odom_list[1]) ,
                             static_cast<double>(initial_odom_list[2]) ,
                             static_cast<double>(initial_odom_list[5]) );
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    root_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    root_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "/"+rover_name+"/wheel_odom", 10));
    odom_pub_->msg_.header.frame_id = "map";
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.x = initial_odom_list[0];
    odom_pub_->msg_.pose.pose.position.y = initial_odom_list[1];
    odom_pub_->msg_.pose.pose.position.z = initial_odom_list[2];
    odom_pub_->msg_.pose.covariance = {
        static_cast<double>(pose_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(pose_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(pose_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(pose_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(pose_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(pose_cov_list[5]) };
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = initial_odom_list[3];
    odom_pub_->msg_.twist.twist.angular.y = initial_odom_list[4];
    odom_pub_->msg_.twist.twist.angular.z = initial_odom_list[5];
    odom_pub_->msg_.twist.covariance = {
        static_cast<double>(twist_cov_list[0]), 0., 0., 0., 0., 0.,
        0., static_cast<double>(twist_cov_list[1]), 0., 0., 0., 0.,
        0., 0., static_cast<double>(twist_cov_list[2]), 0., 0., 0.,
        0., 0., 0., static_cast<double>(twist_cov_list[3]), 0., 0.,
        0., 0., 0., 0., static_cast<double>(twist_cov_list[4]), 0.,
        0., 0., 0., 0., 0., static_cast<double>(twist_cov_list[5]) };

  }






    void FourWheelSteeringController::visCallback(const nav_msgs::Odometry& vis_ ) {
      double roll, pitch, yaw;
      tf2::Quaternion q_orig( static_cast< tf2Scalar>(vis_.pose.pose.orientation.x),
                              static_cast< tf2Scalar>(vis_.pose.pose.orientation.y) ,
                              static_cast< tf2Scalar>(vis_.pose.pose.orientation.z) ,
                              static_cast< tf2Scalar>(vis_.pose.pose.orientation.w) ) ;
      tf2::Matrix3x3 m( q_orig );
      m.getRPY(roll, pitch, yaw);
      double vis_difference = fabs( odometry_.getHeading() ) - fabs(yaw);
 
      yaw_vis = yaw ;

    }




  void FourWheelSteeringController::setSteerState( double relative_theta ){
    bool is_pos = 0 < relative_theta ;
    bool is_neg = 0 > relative_theta ;
    // determine if changing the steering state
  if( (steerState != EXPLICIT &&   command_struct_twist_.explicit_steer_on )
   || (steerState == EXPLICIT && ! command_struct_twist_.explicit_steer_on ) ){
    SWITCHING_STATES = true ;
    setStateSwitchTimer() ;
  }
    // TRYING WITHOUT EXPLICIT STEER NOW
    // Because move base is not as predictable
    if( command_struct_twist_.explicit_steer_on ){//M_PI/2 < fabs(relative_theta) ){
      steerState = EXPLICIT ;

    } else if( 0.0 == round(relative_theta) ){
      steerState = STRAIGHT ;

    } else if( is_neg  &&  M_PI/6 > fabs(relative_theta) ){
      steerState = SOFT_LEFT ;

    } else if( is_neg  &&  M_PI/6 < fabs(relative_theta) ){
      steerState = HARD_LEFT ;

    } else if( is_pos  &&  M_PI/6 > relative_theta ){
      steerState = SOFT_RIGHT ;

    } else if( is_pos  &&  M_PI/6 < relative_theta ){
      steerState = HARD_RIGHT ;

    } else {
      steerState = STRAIGHT ;
    }
       // MED_LEFT  , // TODO 
       // MED_RIGHT  // TODO

  }

  void FourWheelSteeringController::ekfCallback(const nav_msgs::Odometry& msg_ ) 
  {
    boost::lock_guard<boost::mutex> guard( poses_mutex ) ; // runs thread safety through the RAII idiom
    double roll, pitch, yaw;
    tf2::Quaternion q_orig( static_cast< tf2Scalar>(msg_.pose.pose.orientation.x),
                            static_cast< tf2Scalar>(msg_.pose.pose.orientation.y) ,
                            static_cast< tf2Scalar>(msg_.pose.pose.orientation.z) ,
                            static_cast< tf2Scalar>(msg_.pose.pose.orientation.w) ) ;
    tf2::Matrix3x3 m( q_orig );
    m.getRPY(roll, pitch, yaw);

    poses["ekf_cur_x"] = msg_.pose.pose.position.x ;
    poses["ekf_cur_y"] = msg_.pose.pose.position.y ;
    poses["ekf_cur_z"] = msg_.pose.pose.position.z ;
    poses["ekf_cur_pit"] = pitch ;
    poses["ekf_cur_rol"] = roll ;
    poses["ekf_cur_yaw"] = yaw ;


  }
  void FourWheelSteeringController::imuCallback(const sensor_msgs::Imu& imu_ ) 
  {
    if( 1 < internal_time.toSec() ){
        boost::lock_guard<boost::mutex> guard( poses_mutex ) ; // runs thread safety through the RAII idiom
        double roll, pitch, yaw;
        tf2::Quaternion q_orig( static_cast< tf2Scalar>(imu_.orientation.x),
                                static_cast< tf2Scalar>(imu_.orientation.y) ,
                                static_cast< tf2Scalar>(imu_.orientation.z) ,
                                static_cast< tf2Scalar>(imu_.orientation.w) ) ;
        tf2::Matrix3x3 m( q_orig );
        m.getRPY(roll, pitch, yaw);
        poses["imu_cur_rol"]  = roll ;
        poses["imu_cur_pit"] = pitch ;
        poses["imu_cur_yaw"]   = yaw ;
        if( !odometry_.yaw_diff_set && odometry_.heading_is_set ){
          odometry_.setImuYAwDifference( yaw - odometry_.getHeading() ) ;
        }
        poses["imu_ang_vx"] = imu_.angular_velocity.x;
        poses["imu_ang_vy"] = imu_.angular_velocity.y;
        poses["imu_ang_vz"] = imu_.angular_velocity.z;
      }
    }

} // namespace four_wheel_steering_controller
