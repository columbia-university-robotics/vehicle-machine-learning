/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Irstea
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

#include <ros/node_handle.h>
#include <ros/time.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <four_wheel_steering_controller/gather_to_file.h>
//#include <torch/script.h>
//#include <torch/torch.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace four_wheel_steering_controller
{
  namespace bacc = boost::accumulators;

  /**
   * \brief The Odometry class handles odometry readings
   * (2D pose and velocity with related timestamp)
   */
  class Odometry
  {
  public:

    /**
     * \brief Constructor
     * Timestamp will get the current time value
     * Value will be set to zero
     * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
     */
    Odometry(size_t velocity_rolling_window_size = 10);

    /**
     * \brief Initialize the odometry
     * \param time Current time
     */
    void init(const ros::Time &time);

 //ADDON#####################################################################################
 //##########################################################################################
 //VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
    void set_xyheading(const double &x, const double &y,const double &h);

    bool heading_is_set ;
    void set_xyzheading(const double &x, const double &y, const double &z,const double &h);

    void normalize_heading();

    bool update_for_explicit_steer(const double &fl_speed, const double &fr_speed,
                      const double &rl_speed, const double &rr_speed, const ros::Time &time);
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//##########################################################################################
//##########################################################################################

    /**
     * \brief Updates the odometry class with latest wheels and steerings position
     * \param fl_speed front left wheel vehicle speed [rad/s]
     * \param fr_speed front right wheel vehicle speed [rad/s]
     * \param rl_speed rear left wheel vehicle speed [rad/s]
     * \param rr_speed rear right wheel vehicle speed [rad/s]
     * \param front_steering  steering position [rad]
     * \param rear_steering  steering position [rad]
     * \param time      Current time
     * \return true if the odometry is actually updated

     */
    bool update(const double& fl_speed, const double& fr_speed, const double& rl_speed, const double& rr_speed,
                      double front_steering, double rear_steering, double ang_cmd ,
                      double imu_cur_rol , double imu_cur_pit,  const ros::Time &time);

  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  //        START                 FILE WRITER CALLS
  ///////////////////////////////////////////////////////////////////////////////////////////////

    /**
     *  \brief calculate the odometry based on parameters from regression
     */
    bool trainedSkidIntegration( bool steerState ,
                                    std::map<std::string, std::map< std::string , double >  > *joint_map ,
                                    std::map< std::string , double > *poses ,
                                    const ros::Time &time,
                                    boost::mutex *poses_mutex , float nn_deltas[4]);
                                    // std::map< std::string , std::map< std::string , std::vector<double> >> *param_map ,
    /*
    torch::Tensor getInputTensor( std::map<std::string, std::map< std::string , double >  >* jm ,
                                std::map< std::string , double > *p,
                                torch::jit::script::Module *nn_module );
    */
    void writeToFile( int steerState ,
                        std::map<std::string, std::map< std::string , double >  > *joint_map ,
                        std::map< std::string , double > *poses  );
    gather_to_file::GatherToFile make_csv_obj ;

    // for delta movement calculation
    std::map< std::string , double > odom_calc ;

  ///////////////////////////////////////////////////////////////////////////////////////////////
  //        END                   FILE WRITER CALLS
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////





    double imu_yaw_difference  = 0;
    bool yaw_diff_set  = false;
    /**
     * \brief NASA has the IMU yaw changing per spawn, we should calibrate yaw diff
     */
    void setImuYAwDifference(double diff) 
    {
      yaw_diff_set = true ;
      imu_yaw_difference = diff;
    }

    /**
     * \brief heading getter
     * \return heading [rad]
     */
    double getHeading() const
    {
      return heading_;
    }

    /**
     * \brief x position getter
     * \return x position [m]
     */
    double getX() const
    {
      return x_;
    }

    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getY() const
    {
      return y_;
    }
    /**
     * \brief y position getter
     * \return y position [m]
     */
    double getZ() const
    {
      return z_;
    }

    /**
     * \brief linear velocity getter norm
     * \return linear velocity [m/s]
     */
    double getLinear() const
    {
      return linear_;
    }

    /**
     * \brief linear velocity getter along X on the robot base link frame
     * \return linear velocity [m/s]
     */
    double getLinearX() const
    {
      return linear_x_;
    }

    /**
     * \brief linear velocity getter along Y on the robot base link frame
     * \return linear velocity [m/s]
     */
    double getLinearY() const
    {
      return linear_y_;
    }

    /**
     * \brief angular velocity getter
     * \return angular velocity [rad/s]
     */
    double getAngular() const
    {
      return angular_;
    }

    /**
     * \brief linear acceleration getter
     * \return linear acceleration [m/s²]
     */
    double getLinearAcceleration() const
    {
      return bacc::rolling_mean(linear_accel_acc_);
    }

    /**
     * \brief linear jerk getter
     * \return linear jerk [m/s³]
     */
    double getLinearJerk() const
    {
      return bacc::rolling_mean(linear_jerk_acc_);
    }

    /**
     * \brief front steering velocity getter
     * \return front_steer_vel [m/s³]
     */
    double getFrontSteerVel() const
    {
      return bacc::rolling_mean(front_steer_vel_acc_);
    }

    /**
     * \brief rear steering velocity getter
     * \return rear_steer_vel [m/s³]
     */
    double getRearSteerVel() const
    {
      return bacc::rolling_mean(rear_steer_vel_acc_);
    }

    /**
     * \brief Sets the wheel parameters: radius and separation
     * \param steering_track          Seperation between left and right steering joints [m]
     * \param wheel_steering_y_offset Offest between the steering and wheel joints [m]
     * \param wheel_radius            Wheel radius [m]
     * \param wheel_base              Wheel base [m]
     */
    void setWheelParams(double steering_track, double wheel_steering_y_offset, double wheel_radius, double wheel_base);

    /**
     * \brief Velocity rolling window size setter
     * \param velocity_rolling_window_size Velocity rolling window size
     */
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

  private:

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    /**
     * \brief Integrates the velocities (linear on x and y and angular)
     * \param linear_x  Linear  velocity along x of the robot frame  [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param linear_y  Linear  velocity along y of the robot frame   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateXY(double linear_x, double linear_y, double angular);

    /**
     * \brief Integrates the velocities (linear on x and y and angular) and rotates along Yaw Pitch Roll, http://planning.cs.uiuc.edu/node102.html
     * \param linear_x  Linear  velocity along x of the robot frame  [m] 
     * \param linear_y  Linear  velocity along y of the robot frame   [m] 
     * \param linear_z  Linear  velocity along y of the robot frame   [m] 
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     * \param imu_cur_rol [rad] roll computed by IMU
     * \param imu_cur_pit [rad] pitch computed by IMU
     */
    void integrateXYZ(double linear_x, double linear_y, double angular, double imu_cur_rol , double  imu_cur_pit);


    /**
     * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateRungeKutta2(double linear, double angular);

    /**
     * \brief Integrates the velocities (linear and angular) using exact method
     * \param linear  Linear  velocity   [m] (linear  displacement, i.e. m/s * dt) computed by encoders
     * \param angular Angular velocity [rad] (angular displacement, i.e. m/s * dt) computed by encoders
     */
    void integrateExact(double linear, double angular);

    /**
     *  \brief Reset linear and angular accumulators
     */
    void resetAccumulators();

    /// Current timestamp:
    ros::Time last_update_timestamp_;

    /// Current pose:
    double x_;        //   [m]
    double y_;        //   [m]
    double z_;        //   [m]
    double heading_;  // [rad]

    /// Current velocity:
    double linear_, linear_x_, linear_y_ , linear_z_;  //   [m/s]
    double angular_; // [rad/s]

    /// Wheel kinematic parameters [m]:
    double steering_track_;
    double wheel_steering_y_offset_;
    double wheel_radius_;
    double wheel_base_;

    /// Previous wheel position/state [rad]:
    double wheel_old_pos_;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_accel_acc_;
    RollingMeanAcc linear_jerk_acc_;
    RollingMeanAcc front_steer_vel_acc_;
    RollingMeanAcc rear_steer_vel_acc_;
    double linear_vel_prev_, linear_accel_prev_;
    double front_steer_vel_prev_, rear_steer_vel_prev_;
    double fl_wheel_pos_prev_, fr_wheel_pos_prev_,
           rl_wheel_pos_prev_, rr_wheel_pos_prev_;


    // the neural network(s)
    //torch::jit::script::Module module_x, module_y, module_z, module_yaw;
  };
}
