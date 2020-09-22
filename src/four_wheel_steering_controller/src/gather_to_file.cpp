/*
  author : Jonathan Sanabria
  Write topic information directly to a file in csv format.
*/
#include <four_wheel_steering_controller/gather_to_file.h>

// for roll, pitch, yaw
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2/LinearMath/Matrix3x3.h>

// to get heading coordinate system
#include <cmath>

namespace gather_to_file
{
  void linearInterpolation( const double num )
  {
    const double ang = 0;
  }

  void GatherToFile::calculate_time_derivatives( std::map< std::string , double > *p )
  {
    // Calculate the time derivatives here in order to rotate
    // the previous coordinates w.r.t. the prev_yaw
    // assures that the current coordinates act like a ground truth.
    // ang is negative to rotate towards 0 yaw
             
    const double ang = -1*(*p)["gt_prev_yaw"] ;
    x_dot =  ( (*p)["gt_cur_x"]*cos( ang )  - (*p)["gt_cur_y"]*sin( ang ))
          -1*( (*p)["gt_prev_x"]*cos( ang ) - (*p)["gt_prev_y"]*sin( ang ));
    y_dot =  ( (*p)["gt_cur_x"]*sin( ang )  + (*p)["gt_cur_y"]*cos( ang ))
          -1*( (*p)["gt_prev_x"]*sin( ang ) + (*p)["gt_prev_y"]*cos( ang ));
    z_dot = (*p)["gt_cur_z"] - (*p)["gt_prev_z"] ; // does not depend on yaw rotation // as much..

    // custom closed form - signed angle formula between two angles with range [pi,-pi]
    // derived by Jonathan Sanabria
    // defines the degrees and direction "from" would have to turn in order to get to "to"
    auto get_angle_with_direction = []( double to , double from ){
      // next lambda is cosine law for two vectors but working on angles only
      auto ang_between = []( double to , double from ){return acos(cos( to - from ));};

      double main_angle    = ang_between( to , from );
      double _sub_of_angle = ang_between( to , from - main_angle/2 ); 
      double sign = copysign( 1 ,_sub_of_angle  - main_angle  );// easier than adding an epsilon to the denominator and other stuff....
      return sign*main_angle ;
    };
    // end closed form 
    // example logic
    ROS_ASSERT( get_angle_with_direction (        0 ,    M_PI/2 ) < 0 );
    ROS_ASSERT( get_angle_with_direction (        0 , -1*M_PI/2 ) > 0 );
    ROS_ASSERT( get_angle_with_direction (   M_PI/4 ,    M_PI/2 ) < 0 );
    ROS_ASSERT( get_angle_with_direction (   M_PI/4 , -1*M_PI/2 ) > 0 );
    ROS_ASSERT( get_angle_with_direction (   M_PI/2 ,    M_PI ) < 0 );
    ROS_ASSERT( get_angle_with_direction (   M_PI/2 , -1*M_PI ) < 0 );
    ROS_ASSERT( get_angle_with_direction (-1*M_PI/2 ,  3*M_PI/4 ) > 0 );
    ROS_ASSERT( get_angle_with_direction (-1*M_PI/2 , -3*M_PI/4 ) > 0 );

    double gt_yaw_dot = get_angle_with_direction( (*p)["gt_cur_yaw"] , (*p)["gt_prev_yaw"])  ; 
    x_dot   = (*p)["odom_calc_x"] - x_dot   ;
    y_dot   = (*p)["odom_calc_y"] - y_dot  ;
    z_dot   = (*p)["odom_calc_z"] - z_dot ;//TODO : since pitch is good // - z_dot  ;
    yaw_dot = get_angle_with_direction( gt_yaw_dot ,  (*p)["odom_calc_yaw"])  ; 


  }


  void GatherToFile::write_to_file( int steerState , 
                                    std::map<std::string, std::map< std::string , double >  > *jm , // joint_map
                                    std::map< std::string , double > *p ) // poses 
  {

    char buf[1024];

    if( (*p).count( "gt_prev_yaw" ) > 0 && (*p).count( "gt_prev_x" ) > 0 
      && (*p).count( "gt_prev_y" ) > 0 && (*p).count( "gt_prev_z" ) > 0){
      if (  aug_file.is_open())
      { // csv will represent an augmented matrix 
        calculate_time_derivatives( p );

        double left_avg , right_avg ,
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

        left_avg  = (vel_fl + vel_bl)/2 ;
        right_avg = (vel_fr + vel_br)/2 ;
        left_avg  = ( left_avg == 0 )? std::numeric_limits<double>::min() : left_avg ;
        right_avg = (right_avg == 0 )? std::numeric_limits<double>::min() : right_avg ;

        // the front to back ratio is consistently different 
        // and might later be useful for classification of what steer type to use
        double fb_l_ratio , fb_r_ratio , tmp_denom ;
        // will run through tanh to prevent unbounded ratios
        // dividing by 10 to stretch when it goes to 1 
        tmp_denom = (vel_bl == 0 )? std::numeric_limits<double>::min() : vel_bl ;
        fb_l_ratio = tanh( (vel_fl/tmp_denom )/10 )  ;
        tmp_denom = (vel_br == 0 )? std::numeric_limits<double>::min() : vel_br ;
        fb_r_ratio = tanh( (vel_fr/tmp_denom )/10 ) ;

        double rl_ratio , lr_ratio ;
        rl_ratio = tanh( (right_avg / left_avg)/10  ) ;
        lr_ratio = tanh( (left_avg / right_avg)/10 ) ;
        // TODO, what is domain of all these input variables
        //                 |                                            |   (difference between gt delta and wheel odom )
        //           CLASS |                 INPUT VARIABLES            | TRUE ODOM DELTA     
        //                 |                                            |      
        //      idx :  0     1    2    3    4    5    6    7    8    9    10  11  12  13                                
        //             ste,  sfl, sfr, sbl, sbr, r/l, l/r, fb , fb , pit, GTx, GTy, GTz, GTyaw                                
        sprintf( buf ,"%d ,  %f , %f , %f , %f , %f , %f , %f , %f , %f , %f , %f , %f , %f \n",
   steerState , vel_fl , vel_fr , vel_bl , vel_br , rl_ratio , lr_ratio , fb_l_ratio , fb_r_ratio , (*p)["imu_cur_pit"] , x_dot, y_dot, z_dot , yaw_dot  );
        aug_file << buf; 
        ROS_DEBUG_STREAM_THROTTLE(.5, buf ); 

      }
    }
  }



} // namespace four_wheel_steering_controller
