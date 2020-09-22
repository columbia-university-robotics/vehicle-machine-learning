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

#include <four_wheel_steering_controller/odometry.h>

#include <boost/bind.hpp>

namespace four_wheel_steering_controller
{
  namespace bacc = boost::accumulators;

  Odometry::Odometry(size_t velocity_rolling_window_size)
  : last_update_timestamp_(0.0)
  , x_(0.0)
  , y_(0.0)
  , heading_(0.0)
  , linear_(0.0)
  , linear_x_(0.0)
  , linear_y_(0.0)
  , angular_(0.0)
  , steering_track_(0.0)
  , wheel_steering_y_offset_(0.0)
  , wheel_radius_(0.0)
  , wheel_base_(0.0)
  , wheel_old_pos_(0.0)
  , velocity_rolling_window_size_(velocity_rolling_window_size)
  , linear_accel_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , linear_jerk_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , front_steer_vel_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , rear_steer_vel_acc_(RollingWindow::window_size = velocity_rolling_window_size)
  , make_csv_obj("/abc_xyz_as_augmented_.csv") 
  , heading_is_set(false)
  {}

  void Odometry::init(const ros::Time& time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    last_update_timestamp_ = time;
  }

  void Odometry::set_xyheading(const double &x, const double &y,const double &h)
  {
    x_ = x ;
    y_ = y ;
    heading_ = h ;
  }
  void Odometry::set_xyzheading(const double &x, const double &y, const double &z,const double &h)
  {
    heading_is_set = true ;
    x_ = x ;
    y_ = y ;
    z_ = z ;
    heading_ = h ;
    odom_calc["yaw"] = h ;
  }
  void Odometry::normalize_heading()
  { // display like gazebo : between -pi to pi
    if( 0 < fabs(heading_) - M_PI){
      int _sign = copysign( 1 , heading_ ) ; 
      switch( _sign ){
        case -1 :
              heading_ = heading_ + 2*M_PI ; 
              break ;
        case  1 :
              heading_ = heading_ - 2*M_PI ; 
              break ;
      }
      ROS_DEBUG_STREAM_THROTTLE(.5, "RUNNING NORMALIZE HEADING !!!!!!!!!!!!!!!!!!!!!!!!!!");
    } 

  }


  bool Odometry::update_for_explicit_steer(const double &fl_speed, const double &fr_speed,
                      const double &rl_speed, const double &rr_speed, const ros::Time &time)
  { 

    double center_to_wheel_distance = std::hypot( steering_track_,wheel_base_ / 2.0) ;
    const double dt = (time - last_update_timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with
    last_update_timestamp_ = time;
    int explicit_dir;
    double explicit_vel , time_for_full_rot , theta_delta , circumference ;
    explicit_vel = (  fabs(fr_speed) + fabs(rr_speed) 
                    + fabs(fl_speed) + fabs(rl_speed))/4.0 ; // get average
    explicit_dir = copysign( 1 , fr_speed ) ; 
    circumference = (center_to_wheel_distance * 2.0 * M_PI) ; 
    if( explicit_vel ){
      time_for_full_rot = circumference /( explicit_vel ) ; 
      theta_delta =  ( dt / time_for_full_rot );
      odom_calc["yaw"] += 8*explicit_dir*theta_delta ; 
    }
    normalize_heading() ;
    // TODO
    linear_accel_acc_(0.0);
    linear_vel_prev_ = 0.0 ;
    linear_jerk_acc_((0.0 - bacc::rolling_mean(linear_accel_acc_))/dt);
    linear_accel_prev_ = bacc::rolling_mean(linear_accel_acc_);
    front_steer_vel_acc_(0.0);
    front_steer_vel_prev_ = 0.0;
    rear_steer_vel_acc_(0.0);
    rear_steer_vel_prev_ = 0.0;

    return true;
  }




  bool Odometry::update( const double &fl_speed, const double &fr_speed,
                        const double &rl_speed, const double &rr_speed,
                        double front_steering, double rear_steering, double ang_cmd ,
                        double imu_cur_rol , double imu_cur_pit,  const ros::Time &time)
  {

    if (fabs(ang_cmd) < 1e-6)
      front_steering = rear_steering = 0 ;
          
    /// Compute linear and angular diff:


    const double front_tmp = cos(front_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
    const double front_left_tmp = front_tmp/sqrt(1-steering_track_*front_tmp*cos(front_steering)
                                               +pow(steering_track_*front_tmp/2,2));
    const double front_right_tmp = front_tmp/sqrt(1+steering_track_*front_tmp*cos(front_steering)
                                                +pow(steering_track_*front_tmp/2,2));
    const double fl_speed_tmp = fl_speed * (1/(1-wheel_steering_y_offset_*front_left_tmp));
    const double fr_speed_tmp = fr_speed * (1/(1-wheel_steering_y_offset_*front_right_tmp));
    const double front_linear_speed = wheel_radius_ * copysign(1.0, fl_speed_tmp+fr_speed_tmp)*
        sqrt((pow(fl_speed,2)+pow(fr_speed,2))/(2+pow(steering_track_*front_tmp,2)/2.0));

    const double rear_tmp = cos(rear_steering)*(tan(front_steering)-tan(rear_steering))/wheel_base_;
    const double rear_left_tmp = rear_tmp/sqrt(1-steering_track_*rear_tmp*cos(rear_steering)
                                               +pow(steering_track_*rear_tmp/2,2));
    const double rear_right_tmp = rear_tmp/sqrt(1+steering_track_*rear_tmp*cos(rear_steering)
                                                +pow(steering_track_*rear_tmp/2,2));
    const double rl_speed_tmp = rl_speed * (1/(1-wheel_steering_y_offset_*rear_left_tmp));
    const double rr_speed_tmp = rr_speed * (1/(1-wheel_steering_y_offset_*rear_right_tmp));
    const double rear_linear_speed = wheel_radius_ * copysign(1.0, rl_speed_tmp+rr_speed_tmp)*
        sqrt((pow(rl_speed_tmp,2)+pow(rr_speed_tmp,2))/(2+pow(steering_track_*rear_tmp,2)/2.0));

    angular_ = (front_linear_speed*front_tmp + rear_linear_speed*rear_tmp)/2.0;

    linear_x_ = (front_linear_speed*cos(front_steering) + rear_linear_speed*cos(rear_steering))/1.55;// 2.0
    linear_y_ = (front_linear_speed*sin(front_steering) + rear_linear_speed*sin(rear_steering))/1.55;// 2.0
    linear_ =  copysign(1.0, rear_linear_speed)*sqrt(pow(linear_x_,2)+pow(linear_y_,2));

    ROS_DEBUG_STREAM_THROTTLE(.5, "FOURWHEEL : "<<"fl_speed "<<fl_speed<<"fr_speed "<<fr_speed<<" angular_ "<<angular_);

    /// Compute x, y and heading using velocity

    const double dt = (time - last_update_timestamp_).toSec();
    if (dt < 0.0001)
      return false; // Interval too small to integrate with


    last_update_timestamp_ = time;
    /// Integrate odometry:
    //integrateXY(linear_x_*dt, linear_y_*dt, angular_*dt);
    integrateXYZ(linear_x_*dt, linear_y_*dt, angular_*dt, imu_cur_rol , imu_cur_pit);
    linear_accel_acc_((linear_vel_prev_ - linear_)/dt);
    linear_vel_prev_ = linear_;
    linear_jerk_acc_((linear_accel_prev_ - bacc::rolling_mean(linear_accel_acc_))/dt);
    linear_accel_prev_ = bacc::rolling_mean(linear_accel_acc_);
    front_steer_vel_acc_((front_steer_vel_prev_ - front_steering)/dt);
    front_steer_vel_prev_ = front_steering;
    rear_steer_vel_acc_((rear_steer_vel_prev_ - rear_steering)/dt);
    rear_steer_vel_prev_ = rear_steering;

    normalize_heading() ;

    return true; 
  }




  void Odometry::setWheelParams(double steering_track, double wheel_steering_y_offset, double wheel_radius, double wheel_base)
  {
    steering_track_   = steering_track;
    wheel_steering_y_offset_ = wheel_steering_y_offset;
    wheel_radius_     = wheel_radius;
    wheel_base_       = wheel_base;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateXY(double linear_x, double linear_y, double angular)
  {
    const double delta_x = linear_x*cos(heading_) - linear_y*sin(heading_);
    const double delta_y = linear_x*sin(heading_) + linear_y*cos(heading_);

    x_ += delta_x;
    y_ += delta_y;
    heading_ += angular;
  }

  void Odometry::integrateXYZ(double linear_x, double linear_y, double angular, double imu_cur_rol , double  imu_cur_pit)
  {
    double a = heading_    ;
    double b = imu_cur_pit ;
    double c = imu_cur_rol ;
    const double delta_x =    linear_x*( cos(a)*cos(b) ) + linear_y*( cos(a)*sin(b)*sin(c)-sin(a)*cos(c) ) ;
    const double delta_y =    linear_x*( sin(a)*cos(b) ) + linear_y*( sin(a)*sin(b)*sin(c)-cos(a)*cos(c) ) ;
    const double delta_z = -1*linear_x*( sin(b) )        + linear_y*( cos(b)*sin(c) ) ; 

    odom_calc["x"] += delta_x;
    odom_calc["y"] += delta_y;
    odom_calc["z"] += delta_z;
    odom_calc["yaw"] += angular;
  }

  void Odometry::integrateRungeKutta2(double linear, double angular)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_       += linear * cos(direction);
    y_       += linear * sin(direction);
    heading_ += angular;
  }

  void Odometry::integrateExact(double linear, double angular)
  {
    if (fabs(angular) < 1e-6)
      integrateRungeKutta2(linear, angular);
    else
    {
      /// Exact integration (should solve problems when angular is zero):
      const double heading_old = heading_;
      const double r = linear/angular;
      heading_ += angular;
      x_       +=  r * (sin(heading_) - sin(heading_old));
      y_       += -r * (cos(heading_) - cos(heading_old));
    }
  }

  void Odometry::resetAccumulators()
  {
    linear_accel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    linear_jerk_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    front_steer_vel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
    rear_steer_vel_acc_ = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  }


  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  //                              FILE WRITER CALLS
  ///////////////////////////////////////////////////////////////////////////////////////////////

  void Odometry::writeToFile( int steerState ,
                                  std::map<std::string, std::map< std::string , double >  > *joint_map ,
                                  std::map< std::string , double > *poses  )

  {
    make_csv_obj.write_to_file( steerState , joint_map , poses );
  }

/*
update_for_explicit_steer(const double &fl_speed, const double &fr_speed, const double &rl_speed, const double &rr_speed,
                       const ros::Time &time,
                      const double &center_to_wheel_distance )

update( const double &fl_speed, const double &fr_speed, const double &rl_speed, const double &rr_speed,
                        double front_steering, double rear_steering, double ang_cmd , const ros::Time &time)
                      */

  bool Odometry::trainedSkidIntegration( bool isExplicitSteer ,
                                          std::map<std::string, std::map< std::string , double >  > *joint_map ,
                                          std::map< std::string , double > *poses ,
                                          const ros::Time &time,
                                          boost::mutex *poses_mutex , float nn_deltas[4] )

                                          //,  std::map< std::string , std::map< std::string , std::vector<double> >> *param_map ,
  {

    // 4ws steer estimation
    double fl_speed = (*joint_map)[ "fl_wheel_joint" ]["vel"] ;
    double fr_speed = (*joint_map)[ "fr_wheel_joint" ]["vel"] ;
    double rl_speed = (*joint_map)[ "bl_wheel_joint" ]["vel"] ;
    double rr_speed = (*joint_map)[ "br_wheel_joint" ]["vel"] ;

    /// We cannot estimate the speed with very small time intervals:
    const double dt = (time - last_update_timestamp_).toSec();
    if (dt < 0.0001 ){
      return false ; // Interval too small to integrate with
    }
    ///////////////////////////////////////////
    // FOR DATA AQUISITION / TRAINING PURPOSES
    (*poses_mutex).lock() ; // runs thread safety
    std::map< std::string , double >  p(*poses);
    (*poses_mutex).unlock() ; // runs thread safety

    odom_calc["x"] = 0 ;
    odom_calc["y"] = 0 ;
    odom_calc["z"] = 0 ;
    //odom_calc["yaw"] = 0 ;
    // update odom_calc
    if( isExplicitSteer ){
        update_for_explicit_steer(fl_speed,
                                    fr_speed,
                                    rl_speed,
                                    rr_speed,
                                    time);
    } else {
        const double fr_steering = (*joint_map)[ "fr_steering_arm_joint" ]["pos"] ;
        const double fl_steering = (*joint_map)[ "fl_steering_arm_joint" ]["pos"] ;
        const double rl_steering = (*joint_map)[ "bl_steering_arm_joint" ]["pos"] ;
        const double rr_steering = (*joint_map)[ "br_steering_arm_joint" ]["pos"] ;
        if (std::isnan(fl_steering) || std::isnan(fr_steering)
            || std::isnan(rl_steering) || std::isnan(rr_steering))
          return false ;
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

        double matching_4ws_getVelocity = 1/wheel_radius_ ;  // ctrl-f OTHERFORMULACHANGE
        update(fl_speed*matching_4ws_getVelocity, fr_speed*matching_4ws_getVelocity, rl_speed*matching_4ws_getVelocity, rr_speed*matching_4ws_getVelocity,
                       front_steering_pos, rear_steering_pos, (*joint_map)["cmd_vel"]["ang"] , p["imu_cur_rol"] , p["imu_cur_pit"], time);                            
    



    }

    double err_x , err_y , err_z , err_heading ;
    // all 
    err_x = err_y = err_z = err_heading = 0 ;

    /*
    
    std::string s_state , odom_axis ;
    switch( static_cast<SteerState>(steerState) ){
      case EXPLICIT : // TODO LEFT RIGHT
              s_state = "EXPLICIT";
              break;
      case STRAIGHT : 
              s_state = "STRAIGHT";
              break;
      case SOFT_LEFT :
              s_state = "SOFT_LEFT";
              break;
      case HARD_LEFT :
              s_state = "HARD_LEFT";
              break;  
      case SOFT_RIGHT :
              s_state = "SOFT_RIGHT";
              break;
      case HARD_RIGHT :
              s_state = "HARD_RIGHT";
              break;                            
    }
    if ( (*param_map)[ s_state ][ "x" ].size() == 0 ){
      return false ; // Interval too small to integrate with
    }
    odom_axis = "x"   ; err_x       = predictOn( speed , (*param_map)[ s_state ][ odom_axis ] );
    odom_axis = "y"   ; err_y       = predictOn( speed , (*param_map)[ s_state ][ odom_axis ] );
    odom_axis = "z"   ; err_z       = predictOn( speed , (*param_map)[ s_state ][ odom_axis ] );
    odom_axis = "yaw" ; err_heading = predictOn( speed , (*param_map)[ s_state ][ odom_axis ] );
    */
    if(!isExplicitSteer){
      //torch::Tensor input_tensor = getInputTensor( joint_map , &p , &module_x  );
      //ROS_INFO_STREAM( "TENSOR SIZE " << input_tensor.sizes());
      err_x = nn_deltas[0];//module_x.forward({input_tensor}).toTensor().item<float>();
      err_y = nn_deltas[1];//module_y.forward({input_tensor}).toTensor().item<float>();
      err_z = nn_deltas[2];//module_z.forward({input_tensor}).toTensor().item<float>();
      err_heading = nn_deltas[3];//module_yaw.forward({input_tensor}).toTensor().item<float>();

    }

    ///////////////////////////
    // finalized the odometry /
    ///////////////////////////
    x_ +=   odom_calc["x"] ;
    y_ +=   odom_calc["y"] ;
    z_ +=   odom_calc["z"] ; //t_z
    if( 2 < time.toSec() ){
      // now that the imu is stabalized lets use imu yaw for better XY wheel odom
      tf2::Quaternion q = tf2::Quaternion();
      q.setRPY(0,0, p["imu_cur_yaw"]  - imu_yaw_difference );
      tf2::Matrix3x3 m( q );
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      heading_ = yaw ;
    }else{
      heading_ = odom_calc["yaw"]  ;
      normalize_heading() ;
    }
    ///////////////////////////
    ROS_DEBUG_STREAM( "ITEM  WHEEL ODOM VAR :"  << x_ << " " << y_ << " "<< z_ << " "<< heading_);

    /*
    ///////////////////////////////////////////
    // FOR DATA AQUISITION / TRAINING PURPOSES
    if( (*poses).count( "gt_prev_x" ) > 0 ){
      // raw wheel odom
      (*poses)["odom_calc_x"]   = odom_calc["x"]  ;
      (*poses)["odom_calc_y"]   = odom_calc["y"]  ;
      (*poses)["odom_calc_z"]   = 0  ;
      (*poses)["odom_calc_yaw"] = odom_calc["yaw"] ;

      
      writeToFile( (int)isExplicitSteer,
                       joint_map,
                       &p );
      

      (*poses)["gt_prev_x"] =   p["gt_cur_x"] ;
      (*poses)["gt_prev_y"] =   p["gt_cur_y"] ;
      (*poses)["gt_prev_z"] =   p["gt_cur_z"] ;
      (*poses)["gt_prev_pit"] = p["gt_cur_pit"] ;
      (*poses)["gt_prev_rol"] = p["gt_cur_rol"] ;
      (*poses)["gt_prev_yaw"] = p["gt_cur_yaw"] ;
    
    } else if ( (*poses).count( "gt_cur_x" ) > 0  ){
      // first time going through trainedSkidIntegration() // could have lock_gaurd here but is only run once.. for init
      (*poses)["gt_prev_x"] =   p["gt_cur_x"] ;
      (*poses)["gt_prev_y"] =   p["gt_cur_y"] ;
      (*poses)["gt_prev_z"] =   p["gt_cur_z"] ;
      (*poses)["gt_prev_pit"] = p["gt_cur_pit"] ;
      (*poses)["gt_prev_rol"] = p["gt_cur_rol"] ;
      (*poses)["gt_prev_yaw"] = p["gt_cur_yaw"] ;
    }
    */


    return true ;
  }

  /*
  // 
  // Calculate difference that should be added to the odom calculation
  // @param          s : speed map to wheel speed values
  // @param param_vect : the weights that should be applied to parameters that were trained on
  //
  torch::Tensor Odometry::getInputTensor( std::map<std::string, std::map< std::string , double >  > * jm ,
                                std::map< std::string , double >* p,
                                torch::jit::script::Module *nn_module )
  {
    // TODO UPDATE "s" to actually be the joint_map and access values accordingly
    float _diff_val , last_param ;
    float left_avg , right_avg ,
           vel_fl , vel_fr , vel_bl , vel_br ,
           pos_fl , pos_fr , pos_bl , pos_br ;

    vel_fl = (*jm)["fl_wheel_joint"]["lin_vel"];// using lin_vel might make learned weights more resistant to radii changes
    vel_fr = (*jm)["fr_wheel_joint"]["lin_vel"];
    vel_bl = (*jm)["bl_wheel_joint"]["lin_vel"];
    vel_br = (*jm)["br_wheel_joint"]["lin_vel"];

    pos_fl = (*jm)["fl_steering_arm_joint"]["pos"];
    pos_fr = (*jm)["fr_steering_arm_joint"]["pos"];
    pos_bl = (*jm)["bl_steering_arm_joint"]["pos"];
    pos_br = (*jm)["br_steering_arm_joint"]["pos"];

    float l_avg , r_avg ;
    left_avg  = (vel_fl + vel_bl)/2 ;
    right_avg = (vel_fr + vel_br)/2 ;
    left_avg  = ( left_avg == 0 )? std::numeric_limits<float>::min() : left_avg ;
    right_avg = (right_avg == 0 )? std::numeric_limits<float>::min() : right_avg ;

    // the front to back ratio is consistently different 
    float fb_l_ratio , fb_r_ratio , tmp_denom ;
    // will run through tanh to prevent unbounded ratios
    // dividing by 10 to stretch when it goes to 1 
    tmp_denom = (vel_bl == 0 )? std::numeric_limits<float>::min() : vel_bl ;
    fb_l_ratio = tanh( (vel_fl/tmp_denom )/10 )  ;
    tmp_denom = (vel_br == 0 )? std::numeric_limits<float>::min() : vel_br ;
    fb_r_ratio = tanh( (vel_fr/tmp_denom )/10 ) ;

    float rl_ratio , lr_ratio ;
    rl_ratio = tanh( (right_avg / left_avg)/10  ) ;
    lr_ratio = tanh( (left_avg / right_avg)/10 ) ;
    // https://pytorch.org/cppdocs/frontend.html
    // https://pytorch.org/cppdocs/api/classat_1_1_tensor.html#_CPPv4NK2at6Tensor3gerERK6Tensor
    // https://pytorch.org/cppdocs/notes/tensor_creation.html
    //vel_fl= vel_fr= vel_bl = vel_br = rl_ratio = lr_ratio = fb_l_ratio = fb_r_ratio =1.0;
    float input_params[] = { vel_fl, vel_fr, vel_bl , vel_br , rl_ratio , lr_ratio , fb_l_ratio , fb_r_ratio ,(float)(*p)["imu_cur_pit"]};
    torch::Device device = torch::kCPU;
    if (torch::cuda::is_available()) {
      ROS_INFO_STREAM( "CUDA is available!" );
      device = torch::kCUDA;
    }
    ROS_INFO_STREAM( "TENSOR SIZE ");
    auto options = torch::TensorOptions().dtype(torch::kFloat32);
    torch::Tensor t = torch::from_blob(input_params, {9}, options);

    torch::Tensor input_tensor  = t.ger(t).expand({4,9,9}).view({1,4,9,9}).to(device) ;
    return input_tensor ;

  }
  */



} // namespace four_wheel_steering_controller
