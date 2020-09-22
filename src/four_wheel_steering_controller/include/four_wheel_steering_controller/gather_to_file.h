/*
  author : Jonathan Sanabria
  Write topic information directly to a file in csv format.
*/
#pragma once
//using namespace std;
#include <iostream>
#include <fstream>

#include <ros/node_handle.h>
//#include <ros/time.h>
#include <sensor_msgs/Imu.h>
//#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
namespace gather_to_file
{
  /**
   * \brief File writing class that also gets the ground truth location for later pose estimation based on desired parameters
   */
  class GatherToFile
  {
  public:
    //imu_info::ImuInfo gt_cb_info ; 

    /**
     * \brief Constructor
     * set file name for the eventual matrix operations equal to 
     * \param augmented_file_name  
     */
    GatherToFile( std::string augmented_file_name )
    //: gt_cb_info()
    {
      boost::filesystem::path cwd(boost::filesystem::current_path());// /home/hada/.ros
      ROS_INFO_STREAM( "\n\n\nCurrent path is " << cwd.string() << "\n\n\n\n");
      if (augmented_file_name.empty() ){
        augmented_matrix_file_name = "af.csv";
      } else {
        augmented_matrix_file_name = cwd.string() + augmented_file_name ;
      }
      aug_file.open(augmented_matrix_file_name, std::ofstream::out | std::ofstream::app);
      ROS_ASSERT( aug_file.is_open() );
      file_is_augmented = true ;


    }
    /**
     * \brief Destructor 
     */
    ~GatherToFile(){

      if (  aug_file.is_open()){   aug_file.close(); }
      if (param_file.is_open()){ param_file.close(); }
      if ( pose_file.is_open()){  pose_file.close(); }
      //boost::thread *gt_thread = gt_cb_info.getThread();
      //gt_thread->join();
    }


    //void init();

    void write_to_file( int steerState , 
                          std::map<std::string, std::map< std::string , double >  > *jm , // joint_map
                          std::map< std::string , double > *p ); // poses

    void calculate_time_derivatives( std::map< std::string , double > *p  );
  private :

    bool file_is_augmented ;
    std::ofstream   aug_file ;
    std::ofstream param_file ;
    std::ofstream  pose_file ;

    // file names
    std::string augmented_matrix_file_name ;
    std::string     param_matrix_file_name ;
    std::string      pose_matrix_file_name ;

    /// Current time derivative :
    double x_dot;        //   [m]
    double y_dot;        //   [m]
    double z_dot;        //   [m]
    double yaw_dot;      //   [rad]

    ros::NodeHandle n;

  };
}
