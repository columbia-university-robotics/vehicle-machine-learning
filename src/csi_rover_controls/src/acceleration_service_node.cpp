/*
AccelerationService.srv

bool    e_stop
float64 velocity_current
float64 relative_theta
float64 front_arm_angle
float64 rear_arm_angle
---
float64 new_velocity

*/
#include <math.h> /* exp */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <ros/assert.h>
// ROS_ASSERT( x > y ); http://wiki.ros.org/CppStyleGuide
#include <csi_rover_controls/AccelerationService.h>


class AccelerationServiceClass 
{
  public:
    AccelerationServiceClass( ){
      // set variables
      //while( ros::ok() ){;}
    }  

    bool callback(  csi_rover_controls::AccelerationService::Request& request , csi_rover_controls::AccelerationService::Response& response);

    const double e_const = exp(1.0) ;
    const double pi_const = 4*atan(1) ;


};


bool AccelerationServiceClass::callback(  csi_rover_controls::AccelerationService::Request& request , csi_rover_controls::AccelerationService::Response& response){
  double e_to_x_vel , estop_result , velocity_result , turning_result , f_arm_result ;

  e_to_x_vel = 1/(1+exp( request.velocity_current / (7*exp(1.0)) ));

#if FP_FAST_FMA
  velocity_result = fma( -200 , e_to_x_vel , 100 ); 
#else
  velocity_result = 100 - 200*( e_to_x_vel ) ;
#endif

  double base_slow_func = -velocity_result*( 2/pi_const ) ;
  // e_stops are serious conditions
  if( request.e_stop )
    estop_result = base_slow_func ;
  else 
    estop_result = 0 ;

  // turn range 0-pi
  turning_result = base_slow_func*( abs(request.relative_theta )*( 1/pow(pi_const,4))) ; 
  
  // arm range 0-pi/2 # hardly ever goes over 0.1
  if( isgreater( request.front_arm_angle , -0.037 ) )
    f_arm_result = base_slow_func*(  abs(request.front_arm_angle)*( 80/pi_const)); 
  else 
    f_arm_result = 0 ;


  response.new_velocity = estop_result + velocity_result + turning_result + f_arm_result ; 

  return true ;
}





int main( int argc , char** argv )
{

  ros::init( argc , argv , "acceleration_service_node" );
  ros::NodeHandle nh ;

  AccelerationServiceClass ASC_object ; 
  ros::ServiceServer service = nh.advertiseService("acceleration_service", &AccelerationServiceClass::callback, &ASC_object);

  ros::spin();

  return 0 ;
}
