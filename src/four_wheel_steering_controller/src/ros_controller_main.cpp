/*
 * Start the steering controller
 *
 *
 * For CSI 
*/

#include <chrono>
#include <thread>
#include <four_wheel_steering_controller/four_wheel_steering_controller.h>

#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sstream>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

int main(int argc, char **argv)
{


  ros::init(argc, argv, "four_wheel_steering");
  ros::NodeHandle nh("four_wheel_steering_controller");

  // This should be set in launch files as well
  nh.setParam("/use_sim_time", true);
    std::string rover_name ; 
  bool found_name = nh.getParam("/rover_name", rover_name);


  std::string param_name = "/localization/ekf_odom_node/initial_state";
  while ( ! nh.hasParam(param_name) ){;}
  double initial_odom_[15] ;
  XmlRpc::XmlRpcValue initial_odom_list;
  if (!nh.getParam(param_name, initial_odom_list))
  {
    ROS_ERROR_STREAM("Couldn't retrieve wheel param '" << param_name << "'.");
    return 1;
  }
  // Fill Odom_list with initial pose information
  if (initial_odom_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {

      for (int i = 0; i < 15 ; ++i )
      {
        if (initial_odom_list[i].getType() == XmlRpc::XmlRpcValue::TypeInvalid)
        {
          boost::shared_ptr<sensor_msgs::Imu const> imu_data;
          imu_data = ros::topic::waitForMessage<sensor_msgs::Imu>("/"+rover_name+"/imu_rect");
          double roll, pitch, yaw;
          sensor_msgs::Imu imu_ ;
          if(imu_data != NULL){
            imu_ = *imu_data;
            tf2::Quaternion q_orig( static_cast< tf2Scalar>(imu_.orientation.x),
                                    static_cast< tf2Scalar>(imu_.orientation.y) ,
                                    static_cast< tf2Scalar>(imu_.orientation.z) ,
                                    static_cast< tf2Scalar>(imu_.orientation.w) ) ;
            tf2::Matrix3x3 m( q_orig );
            m.getRPY(roll, pitch, yaw);
          }

          // logic to get imu data
          switch( i ){
            case 3 : initial_odom_list[i]=roll ;
                  ROS_INFO_STREAM(
                  "Initialized wheel_odom roll  with IMU data-----");
                  break ;
            case 4 : initial_odom_list[i]=pitch;
                      ROS_INFO_STREAM(
                  "Initialized wheel_odom roll  with IMU data-----");
                  break ;
            case 5 : initial_odom_list[i]=yaw  ;
                  ROS_INFO_STREAM(
                  "Initialized wheel_odom yaw  with IMU data-----");
                  ROS_ASSERT(0);
                  break ;
          }
          ROS_DEBUG_STREAM( "is a TypeInvalid."<< initial_odom_list[i] << "   IDX : "<< i);
          initial_odom_list[i] = 0.0 ;
        } else if (initial_odom_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
        {
          ROS_DEBUG_STREAM( "is a TypeInt."<< initial_odom_list[i] << "   IDX : "<< i);
          initial_odom_list[i] = 0.0 ;

        } else if (initial_odom_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
          ROS_DEBUG_STREAM( "isn't a TypeDouble."<< initial_odom_list[i] << "   IDX : "<< i);
          initial_odom_list[i] = 0.0 ;

        }
        ROS_DEBUG_STREAM( "past TypeDouble."<< initial_odom_list[i] << "   IDX : "<< i);

      }

  }
  four_wheel_steering_controller::FourWheelSteeringController robot;
  ROS_DEBUG_STREAM("period: " << robot.getPeriod().toSec());
  robot.init(nh , initial_odom_list);

  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  try
  {
    robot.update();
  }
  catch (int e)
  {
    ROS_ERROR_STREAM( "Try catch error."<< e );
  }
  return 0;
}