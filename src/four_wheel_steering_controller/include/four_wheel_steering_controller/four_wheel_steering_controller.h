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

#pragma once


//#include <controller_interface/multi_interface_controller.h>
//#include <hardware_interface/joint_command_interface.h>
//#include <pluginlib/class_list_macros.hpp>

#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <tf2/LinearMath/Quaternion.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include "ros/param.h"

#include <four_wheel_steering_controller/odometry.h>
#include <four_wheel_steering_controller/speed_limiter.h>

#include <sstream>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <boost/thread.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/median.hpp>

//#include <torch/script.h>
//#include <torch/torch.h>



namespace four_wheel_steering_controller{
  namespace bacc = boost::accumulators;
  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
   */
  class FourWheelSteeringController  {
  public:
    boost::mutex poses_mutex ; // 
    boost::mutex joint_map_mutex ; // 
    boost::mutex delta_x_mutex , delta_y_mutex , delta_z_mutex ,delta_yaw_mutex ;
    float delta_x = 0 ;
    float delta_y = 0 ;
    float delta_z = 0 ;
    float delta_yaw = 0;
    //void nnThread( torch::jit::script::Module *module_4WS, torch::jit::script::Module *module_EXP , float *delta_var , boost::mutex *var_mutex );

    FourWheelSteeringController();

    ros::Time getTime() const {return ros::Time::now();}
    void setStateSwitchTimer() { end_state_switch_time = ros::Time::now().toSec() + 3.0 ;}
    bool stateSwitchTimeDone() { return (end_state_switch_time - ros::Time::now().toSec()) < 0.0 ;}
    ros::Duration getPeriod() const {return ros::Duration(0.01);}

    /**
     * \brief Initialize controller
     * \param robot_hw      Velocity and position joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(ros::NodeHandle& root_nh  , XmlRpc::XmlRpcValue initial_odom_list);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update();

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);



    void imuCallback(const sensor_msgs::Imu& imu_   ) ;
    void visCallback(const nav_msgs::Odometry& vis_ ) ;

    void ekfCallback(const nav_msgs::Odometry& msg_ );
    std::map< std::string , double > poses; // [m]

    ros::Subscriber sub_ekf_;
  private:
    std::string name_;

    /// Odometry related:
    ros::Duration publish_period_;
    ros::Time last_state_publish_time_ , internal_time ;
    bool open_loop_;

    /// Velocity command related:
    struct Command
    {
      ros::Time stamp;

      Command() : stamp(0.0) {}
    };
    struct CommandTwist : Command
    {
      double lin_x;
      double lin_y;
      double ang;
      double explicit_steer_on ;

      CommandTwist() : lin_x(0.0), lin_y(0.0), ang(0.0), explicit_steer_on(0.0) {}
    };
 
    realtime_tools::RealtimeBuffer<CommandTwist> command_twist_;
    CommandTwist command_struct_twist_;
    ros::Subscriber sub_command_;
    ros::Subscriber sub_imu_ ;

    /// Odometry related:
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    //std::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;
    Odometry odometry_;

    /// Wheel separation (or track), distance between left and right wheels (from the midpoint of the wheel width):
    double track_;
    /// Distance between a wheel joint (from the midpoint of the wheel width) and the associated steering joint:
    /// We consider that the distance is the same for every wheel
    double wheel_steering_y_offset_;

    /// Wheel radius (assuming it's the same for the left and right wheels):
    double wheel_radius_;

    /// Wheel base (distance between front and rear wheel):
    double wheel_base_;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Frame to use for the robot base:
    std::string base_frame_id_;

    /// rover id :
    std::string rover_name ;

    /// Whether to publish odometry to tf or not:
    bool enable_odom_tf_;

    /// Whether the control is make with four_wheel_steering msg or twist msg:
    bool enable_twist_cmd_;

    /// Speed limiters:
    CommandTwist last1_cmd_;
    CommandTwist last0_cmd_;
    SpeedLimiter limiter_lin_;
    SpeedLimiter limiter_ang_;

    // Keep track of the heading from the other sensors to allow group correction
    double yaw_imu , yaw_vis ;



    // using a ratio since the effort topic sent doesnot translate 1:1 to velocity
    // works because the controller depends on symmetry
    double l_to_r_speed_ratio ; // the ideal ratio between left and right
    double l_f_to_b_speed_ratio ; // the real ratio between left front and left
    double r_f_to_b_speed_ratio ; // the real ratio between left front and left
    double original_fl_effort ;
    double original_fr_effort ;
    double original_rl_effort ;
    double original_rr_effort ;
    // effort adjustment due to axle friction
    double adjustment_fl_effort ;
    double adjustment_fr_effort ;
    double adjustment_rl_effort ;
    double adjustment_rr_effort ;

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::median> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;
    
    // get the mean in order to filter volatile decisions from move_base
    RollingMeanAcc linear_x_acc_;
    RollingMeanAcc angular_z_acc_;
    /**
     * \brief linear acceleration getter
     * \return linear acceleration [m/s²]
     */
    double getLinearMean() const
    {
      return bacc::median(linear_x_acc_);
    }

    /**
     * \brief linear jerk getter
     * \return linear jerk [m/s³]
     */
    double getAngularMean() const
    {
      return bacc::median(angular_z_acc_);
    }

    ros::Publisher wheel_odom_publisher ;
    nav_msgs::Odometry msg  ;

    bool increase_covariance_for_slow_stop = false ;
    bool was_slow_stopping = false ;
    // nasa scout_x topics
    ros::Subscriber sub_joint_ ;
    ros::Publisher fl_steering_pub , fr_steering_pub , bl_steering_pub , br_steering_pub ;
    ros::Publisher fl_wheel_pub    , fr_wheel_pub    , bl_wheel_pub    , br_wheel_pub    ;
    // axle pid topics
    ros::Subscriber fl_ctrl_eff    , fr_ctrl_eff     , bl_ctrl_eff     , br_ctrl_eff     ;
    ros::Publisher  fl_axle_state   , fr_axle_state   , bl_axle_state   , br_axle_state  ;
    ros::Publisher  fl_ang_vel_setpoint    , fr_ang_vel_setpoint,
                    bl_ang_vel_setpoint    , br_ang_vel_setpoint    ;
    /// Frame to use for the robot base:
    std::string joint_state_topic_name ;
    // 
    // map[ joint_name ][ pos/vel/eff/lin_vel/pid_ctrl_eff ]
    std::map<std::string, std::map< std::string , double >  > joint_map ;
    /*JointStateStruct    
      bl_arm_joint , bl_steering_arm_joint , bl_wheel_joint ,
      br_arm_joint , br_steering_arm_joint , br_wheel_joint ,
      fl_arm_joint , fl_steering_arm_joint , fl_wheel_joint ,
      fr_arm_joint , fr_steering_arm_joint , fr_wheel_joint ,
      sensor_joint ;
    */
     void getJointState(const sensor_msgs::JointState& state){
      boost::lock_guard<boost::mutex> guard( joint_map_mutex ) ; // runs thread safety through the RAII idiom
      //ROS_ERROR_STREAM( "+++++++++++++++++++++++"<< joint_state_topic_name);
      std::map< std::string , double > holder ;

      for( int counter = state.name.size() ; counter-- ; ){
        holder["pos"]     =  state.position[counter] ;
        holder["vel"]     =  state.velocity[counter] ;
        holder["eff"]     =  state.effort[counter] ;
        holder["lin_vel"] =  state.velocity[counter]*wheel_radius_ ;
        joint_map[ state.name[counter] ] = holder ;

      }
     };
     void publishFromPointer( ros::Publisher* publisher_ptr ,
                            double new_val ){
      // "pos" | "vel" | "eff" | "lin_vel"
      std_msgs::Float64 tmp  ;
      tmp.data = new_val ; 
      (*publisher_ptr).publish(tmp);

     };


//---------------------- setCommand(...)      ~>      publishFromPointer(..)
//---------------------- getVelocity(...)     ~>      getJointState(..)
//---------------------- getPosition(...)     ~>      getJointState(..)
    /**
     * \brief get new control effort
     * \param command Velocity command message (Float64)
     */
    void fl_ctrlEffCallback(const std_msgs::Float64& control_effort_input)
    {
      boost::lock_guard<boost::mutex> guard( joint_map_mutex ) ; // runs thread safety through the RAII idiom
      joint_map["fl_wheel_joint"]["pid_ctrl_eff"] = control_effort_input.data;
    }
    void fr_ctrlEffCallback(const std_msgs::Float64& control_effort_input)
    {
      boost::lock_guard<boost::mutex> guard( joint_map_mutex ) ; // runs thread safety through the RAII idiom
      joint_map["fr_wheel_joint"]["pid_ctrl_eff"] = control_effort_input.data;
    }
    void bl_ctrlEffCallback(const std_msgs::Float64& control_effort_input)
    {
      boost::lock_guard<boost::mutex> guard( joint_map_mutex ) ; // runs thread safety through the RAII idiom
      joint_map["bl_wheel_joint"]["pid_ctrl_eff"] = control_effort_input.data;
    }
    void br_ctrlEffCallback(const std_msgs::Float64& control_effort_input)
    {
      boost::lock_guard<boost::mutex> guard( joint_map_mutex ) ; // runs thread safety through the RAII idiom
      joint_map["br_wheel_joint"]["pid_ctrl_eff"] = control_effort_input.data;
    }
//##########################################################################################
//##########################################################################################
//##########################################################################################










  //private:

    /**
     * \brief Update and publish odometry
     * \param time   Current time
     */
    void updateOdometry(const ros::Time &time);
    /**
     * \brief Compute and publish command
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void updateCommand(const ros::Time& time, const ros::Duration& period);


    /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
    void cmdVelCallback(const geometry_msgs::Twist& command);


    /**
     * \brief Get the wheel names from a wheel param
     * \param [in]  controller_nh Controller node handler
     * \param [in]  wheel_param   Param name
     * \param [out] wheel_names   Vector with the whel names
     * \return true if the wheel_param is available and the wheel_names are
     *        retrieved successfully from the param server; false otherwise
     */
    bool getWheelNames(ros::NodeHandle& controller_nh,
                       const std::string& wheel_param,
                       std::vector<std::string>& wheel_names);

    /**
     * \brief Sets the odometry publishing fields
     * \param root_nh Root node handle
     * \param controller_nh Node handle inside the controller namespace
     */
    void setOdomPubFields(ros::NodeHandle& root_nh , XmlRpc::XmlRpcValue initial_odom_list);


   void setSteerState( double relative_theta );
    enum SteerState {
        EXPLICIT   ,
        STRAIGHT   ,
        SOFT_LEFT  ,
        HARD_LEFT  ,
        SOFT_RIGHT ,
        HARD_RIGHT ,
        MED_LEFT  ,
        MED_RIGHT  
    }; 
    int steerState ;
    bool SWITCHING_STATES ;
    double end_state_switch_time ;

    // the neural network(s)
    //torch::jit::script::Module module_4WS_x, module_4WS_y, module_4WS_z, module_4WS_yaw,
      //                          module_EXPLICIT_x, module_EXPLICIT_y ,module_EXPLICIT_z, module_EXPLICIT_yaw;

  };

//  PLUGINLIB_EXPORT_CLASS(four_wheel_steering_controller::FourWheelSteeringController, controller_interface::ControllerBase);
} // namespace four_wheel_steering_controller
