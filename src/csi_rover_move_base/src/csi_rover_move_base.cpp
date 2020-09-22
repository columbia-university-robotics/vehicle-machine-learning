/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/

#include <csi_rover_move_base/csi_rover_move_base.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base {

    MoveBase::MoveBase(tf2_ros::Buffer& tf) :
            tf_(tf),
            as_(nullptr),
            planner_costmap_ros_(nullptr), controller_costmap_ros_(nullptr),
            bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
            blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
            recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
            planner_plan_(nullptr), latest_plan_(nullptr), controller_plan_(nullptr),
            runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) ,
            dist_accumulator(RollingWindow::window_size = 60){

        as_ = new MoveBaseActionServer(ros::NodeHandle(), "csi_move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        recovery_trigger_ = PLANNING_R;

        //get some parameters that will be global to the move base node
        std::string global_planner, local_planner, global_plan_topic , rover_name;
        private_nh.param("/rover_name", rover_name, std::string(""));
        private_nh.param("base_global_planner", global_planner, std::string("global_planner/GlobalPlanner"));
        private_nh.param("base_local_planner", local_planner, std::string("dwa_local_planner/DWAPlannerROS"));
        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("scout_1_tf/chassis"));
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("odom"));// std::string("map")
        private_nh.param("planner_frequency", planner_frequency_, 0.0);
        private_nh.param("controller_frequency", controller_frequency_, 20.0);
        private_nh.param("planner_patience", planner_patience_, 5.0);
        private_nh.param("controller_patience", controller_patience_, 15.0);
        private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default
        private_nh.param("global_plan_topic", global_plan_topic, std::string("/navigation/global_planner/path"));

        private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
        private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

        // parameters of make_plan service
        private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
        private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

        //set up plan triple buffer
        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        global_plan_ = new std::vector<geometry_msgs::PoseStamped>();

        new_plan_sub_ = nh.subscribe<std_msgs::Bool>("/scout_1/new_plan", 1, boost::bind(&MoveBase::newPlanCB, this, _1));
        detection_sub_ = nh.subscribe<std_msgs::Bool>("/"+rover_name+"/obstacle_detected", 1, boost::bind(&MoveBase::detectionCB, this, _1));
        safe_angle_sub_ = nh.subscribe<std_msgs::Float64>("/"+rover_name+"/avoid_obstacle_angle", 1, boost::bind(&MoveBase::avoidObstacleCB, this, _1));
        safe_dist_sub_ = nh.subscribe<std_msgs::Float64>("/"+rover_name+"/avoid_obstacle_distance", 1, boost::bind(&MoveBase::obstacleDistCB, this, _1));
        // The ROS GlobalPlanner plan // Created by dynamic computations 
        dwa_global_plan_sub_ = nh.subscribe<nav_msgs::Path>("/navigation/csi_rover_move_base/GlobalPlanner/plan", 1,
                                                               boost::bind(&MoveBase::dwaGlobalPlanCb, this, _1));
        //set up the planner's thread
        planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

        //for commanding the base
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

        ros::NodeHandle action_nh("move_base");
        action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

        // ==========================
        // we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
        // they won't get any useful information back about its status, but this is useful for tools
        // like nav_view and rviz
        ros::NodeHandle simple_nh("move_base_simple");
        goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

        // global planner subscriber // THE COMPLETE MAP TRAVERSAL PLAN WE CREATED
        ROS_DEBUG("Global planner topic: %s", global_plan_topic.c_str());
        global_plan_sub_ = simple_nh.subscribe<nav_msgs::Path>("/navigation/global_planner/path", 1,
                                                               boost::bind(&MoveBase::globalPlanCb, this, _1));

        robot_pose_sub_ = simple_nh.subscribe<nav_msgs::Odometry>("/scout_1/ekf_odom", 1,
                                                                  boost::bind(&MoveBase::roverOdomCb, this, _1));
        // ==========================

        //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
        private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
        private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

        private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
        private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
        private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

        bool verbose = false;
        if (verbose) {
            print_config();
        }

        //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
        //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
        controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        controller_costmap_ros_->pause();

        planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_->pause();

        //initialize the global planner
        try {
            planner_ = bgp_loader_.createInstance(global_planner);
            planner_->initialize(bgp_loader_.getName(global_planner), controller_costmap_ros_);//planner_costmap_ros_);
        } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }

        //create a local planner
        try {
            tc_ = blp_loader_.createInstance(local_planner);
            ROS_DEBUG("Created local_planner %s", local_planner.c_str());
            tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
        } catch (const pluginlib::PluginlibException& ex) {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
            exit(1);
        }

        // Start actively updating costmaps based on sensor data
        planner_costmap_ros_->start();
        controller_costmap_ros_->start();

        //advertise a service for getting a plan
        make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

        //advertise a service for clearing the costmaps
        clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }

        //load any user specified recovery behaviors, and if that fails load the defaults
        if(!loadRecoveryBehaviors(private_nh)){
            loadDefaultRecoveryBehaviors();
        }

        //initially, we'll need to make a plan
        state_ = PLANNING;

        //we'll start executing recovery behaviors at the beginning of our list
        recovery_index_ = 0;

        //we're all set up now so we can start the action server
        as_->start();

        dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
        dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void MoveBase::print_config() {

        std::cout << "Rover base frame: " << robot_base_frame_ << std::endl;
        std::cout << "Static map frame: " << global_frame_ << std::endl;
    }

    void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){

        ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
        ROS_DEBUG_NAMED("move_base", "Global path waypoints count: %lu", global_plan_->size());

        move_base_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;

        action_goal_pub_.publish(action_goal);
        ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    }

    void MoveBase::globalPlanCb(const nav_msgs::Path::ConstPtr& plan) {

        // UNCOMMENT HERE   -- for unmodified move_base
        // *global_plan_ = (std::vector<geometry_msgs::PoseStamped>)(plan->poses);


        // /* START COMMENT -- for unmodified move_base// start place
        if( ! path_set ){
          // the global plan vector eventually gets modified in place by actionLoop
          // TODO if end it reached then just get the unadulterated one again
          ROS_DEBUG_NAMED("move_base", "In valid globalPlanCb.");
          *global_plan_ = (std::vector<geometry_msgs::PoseStamped>)(plan->poses);
          path_set = true ;
          new_plan = false ;
          if( requires_reset ){
            client_thread_->interrupt();
            client_thread_->join();
            delete client_thread_;
            requires_reset = false ;
          }
          client_thread_ = new boost::thread(boost::bind(&MoveBase::actionLoop, this)); 
        }
        // END COMMENT      -- for unmodified move_base */

        //////////////////////////////////////////////////////////////
        // Relevant Action Client example :
        // http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
        //
        // kinda bare bones "MoveBase::actionLoop(..)" :
        // commit -- 9861602e964a8abb17393d4feb34ba308d48a6ba -- "Working Threaded action publisher which evaluates cost.."
        //////////////////////////////////////////////////////////////
    }
    void MoveBase::dwaGlobalPlanCb(const nav_msgs::Path::ConstPtr& plan) {
          dwa_global_plan_ = (std::vector<geometry_msgs::PoseStamped>)(plan->poses);
    }

    void MoveBase::actionLoop( void ) {
          actionlib::SimpleClientGoalState goalState = actionlib::SimpleClientGoalState::PENDING;
          bool moveToGoal , atGoal , isLethal , inMap , isValidGoal , isCorrectedGoal ;
          int direction = 1 ; 

          MoveBaseClient ac("csi_move_base", true);
          //wait for the action server to come up
          while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_DEBUG_THROTTLE(.5,"Waiting for the move_base action server to come up");
          }

          geometry_msgs::PoseStamped way_point ;
          move_base_msgs::MoveBaseGoal goal;
          bool withOutCancelGoal = true ;


          // Find the closest path-coordinate and start actionloop from there
          static double d_x = global_pose_.pose.position.x ;
          static double d_y = global_pose_.pose.position.y ;

          struct dist_cmp{
            bool operator()( geometry_msgs::PoseStamped &waypoint1 ,
                     geometry_msgs::PoseStamped &waypoint2 ){
              double dist_1 = std::sqrt( std::pow( d_x - waypoint1.pose.position.x , 2 )
                                        + std::pow( d_y - waypoint1.pose.position.y , 2 ) );
              double dist_2 = std::sqrt( std::pow( d_x - waypoint2.pose.position.x , 2 )
                                        + std::pow( d_y - waypoint2.pose.position.y , 2 ) );
              return dist_1 > dist_2;
            }
          };
          typedef std::priority_queue< geometry_msgs::PoseStamped ,
                                        std::vector< geometry_msgs::PoseStamped > ,
                                        dist_cmp > dist_min_heap_type ;

          std::vector< geometry_msgs::PoseStamped > dc_vect = *global_plan_ ;
          dist_min_heap_type distMinHeap( dc_vect.begin() , dc_vect.end()  ) ;

          int foundIndex = distMinHeap.top().header.seq ;  
          p_idx = foundIndex ; // 0 ;
          ROS_DEBUG_STREAM("The closest way point is at index : "<<  p_idx );


          //////////////////////////////////////////////////////////////////////////
          // START LAMDAs -- used multiple time only in actionLoop
          // lamda fun -- to send a goal
          //              & capture-by-reference
          auto updateGoal = [&]( void ){
            if( withOutCancelGoal ){
              // no goal to cancel 
              // no " last_goal_* " to check
              way_point = (*global_plan_)[ p_idx ] ;
              goal.target_pose = way_point ;
              goal.target_pose.header.stamp = ros::Time::now();
              ROS_DEBUG_STREAM("Sending goal ( "<<  goal.target_pose.pose.position.x 
                                          <<" , "<< goal.target_pose.pose.position.y<<" )" );
              ac.sendGoal(goal);
              // update last_goal_*
              last_goal_x = goal.target_pose.pose.position.x ;
              last_goal_y = goal.target_pose.pose.position.y ;
              withOutCancelGoal = false ;

            }else if( notSameGoalAsLast() ){
              ac.cancelGoal();
              way_point = (*global_plan_)[ p_idx ] ;
              goal.target_pose = way_point ;
              goal.target_pose.header.stamp = ros::Time::now();
              ROS_DEBUG_STREAM("Sending goal ( "<<  goal.target_pose.pose.position.x 
                                          <<" , "<< goal.target_pose.pose.position.y<<" )" );
              ac.sendGoal(goal);
              // update last_goal_*
              last_goal_x = goal.target_pose.pose.position.x ;
              last_goal_y = goal.target_pose.pose.position.y ;
            }
          };
          // lamda fun -- fit the NextGoal and put it in the current global_plan idx
          auto correctNextGoal = [&]( void ){
            // avoid p_idx ++ 
            // continue in general direction til we can evaluate lethality of true nextGoal
            fitNextGoalToMap();
            updateGoal() ;
            return (isCorrectedGoal = true) ;
          }; 
          // lamda fun -- fit the NextGoal and put it in the current global_plan idx
          auto correctCurrGoal = [&]( void ){
            // avoid p_idx ++ 
            // continue in general direction til we can evaluate lethality of true nextGoal
            fitCurrGoalToMap();
            updateGoal() ;
            return (isCorrectedGoal = true) ;
          };        
          // lamda fun -- check if curr goal lethal and next goal out of scope
          auto checkCurrentAndCorrectNextGoal = [&]( void ){
              inMap = nextGoalInLocalMap() ; 
              if( inMap ){
                return (isCorrectedGoal = false) ; //  not lethal or nextGoal inMap
              } else {
                correctNextGoal();
                return (isCorrectedGoal = true) ;
              }
          };
          auto checkAndCorrectNextGoal = [&]( void ){
              inMap = nextGoalInLocalMap() ; 
              if( inMap ){
                return (isCorrectedGoal = false) ; //  not lethal or nextGoal inMap
              } else {
                correctNextGoal();
                return (isCorrectedGoal = true) ;
              }
          };
          auto checkAndCorrectCurrGoal = [&]( void ){
              inMap = currGoalInLocalMap() ; 
              if( inMap ){
                updateGoal() ;  
                return (isCorrectedGoal = false) ; //  not lethal or nextGoal inMap
              } else {
                correctCurrGoal();
                return (isCorrectedGoal = true) ;
              }
          };
          bool correctedToNext ;
          auto clearArea = [&]( int i_x , int i_y ,  int lim_x , int lim_y ){

            for( ; i_x < lim_x ; i_x ++ ){
              for( ; i_y < lim_y ; i_y ++ ){
                planner_costmap_ros_->getCostmap()->setCost( i_x , i_y , 0 );
              }
            }

          };

          int last_refresh_time = 0 ;
          int current_time = 0 ;
          auto refreshMaps = [&]( void){
              current_time = (int)(ros::Time::now().toSec());
              if( (last_refresh_time != current_time) && (current_time % 2 == 0) ){
                last_refresh_time = current_time ;
                if( debug_mode ){
                  ROS_INFO_STREAM_THROTTLE(3, "============= TIME :  "<< current_time ); 
                }
                boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
                controller_costmap_ros_->resetLayers();
                lock_controller.unlock();
              }
          };
          // END LAMDAs -- used multiple time only in actionLoop
          //////////////////////////////////////////////////////////////////////////

          ros::NodeHandle n;
          ros::Rate r(20);

          ////////////////////////////        
          /* The Main Action Loop *///
          ////////////////////////////
          while(n.ok() and !new_plan ){ // TODO:"NEWPLAN" add a statement for "|| new_plan_" // SEE BOTTOM OF METHOD
   
            if( debug_mode ){
              ROS_INFO_STREAM_THROTTLE(3, "Outer Action Loop "<< new_plan); 
            }
            isValidGoal = true ; 
            isCorrectedGoal = false ;

            isCorrectedGoal = checkAndCorrectCurrGoal() ;
            atGoal = isAtGoal() ;

            isLethal  = isLethalGoal( goal.target_pose.pose.position.x ,
                                      goal.target_pose.pose.position.y);
            isValidGoal = ( isLethal == false ) ;
            // TODO EDGE CASES :
            //    1. the planner wants the rover to make a large loop around an obstacle 
            //    but in order to do that then explicit steer becomes engaged.
            //    solution is to engage explicit on a wider turn range 
            //    2. ROVER falls into a crater while doing to "slowlyStop()"
            //    3. rover on an incline -- the move_base sends a signal to move
            //          BUT since move_base takes the moving velocity as feedback to know what the
            //          next cmd_vel should be, if the rover needs a greater effort value it is not sent one.
            //          thus the rovers wheels try and try at a small effort value when they need a greater effort.
            //          perhaps the imu orientation could be useful in such scenarios..
            // move toward goal if valid
            if( !isValidGoal ){
              // "Inner Action Loop " avoided because goal is lethal
              findSafeGoalInMapFromCurrent();
              checkAndCorrectCurrGoal();
              isLethal  = isLethalGoal( goal.target_pose.pose.position.x ,
                                        goal.target_pose.pose.position.y);
              isValidGoal = ( isLethal == false ) ;
            } else {
              while( isValidGoal && ! new_plan ){
                if( debug_mode ){
                  ROS_INFO_STREAM_THROTTLE(3, "Inner Action Loop "<< new_plan); 
                }
                atGoal = isAtGoal() ;
                isLethal  = isLethalGoal( goal.target_pose.pose.position.x ,
                                          goal.target_pose.pose.position.y);
                inMap = currGoalInLocalMap();
                isValidGoal = ! atGoal && ! isLethal && inMap  ;

                // this is where I tried solving EDGECASE#1 

                if( isValidGoal  && !isRoverInLethalCell() && !goalWithinTurnRange()  ){
                  ac.cancelGoal();          //  cancel current goal to initiate own cmd_vel
                  explicitTurnSequence();   //  start turn in place publisher
                  withOutCancelGoal = true ;//  go back to moveBaseAction cmd_vel planning
                  checkAndCorrectCurrGoal();
                }
                if( isStuckRover() ){ 
                // here in case target in valid location and rover just gets stuck
                  ac.cancelGoal();          //  cancel current goal to initiate own cmd_vel
                  correctStuckRover();
                  withOutCancelGoal = true ;//  go back to moveBaseAction cmd_vel planning
                  checkAndCorrectCurrGoal();
                }
                refreshMaps();
                r.sleep(); // allow quick escape if goal not valid

              } 
            }
            // next goal might be fit within map and is now the current goal
            isCorrectedGoal = checkAndCorrectNextGoal();//
            if( ( !isCorrectedGoal && !isValidGoal  ) || atGoal ){//|| (&& isValidGoal) 

              // iterate through plan as expected
              if( p_idx -1 < 0 ){
                direction = 1 ;
              } else if( (*global_plan_).size() < p_idx + 1 ){
                direction = -1 ;
              }
              p_idx += direction ; 
              if( debug_mode ){
                ROS_INFO_STREAM_THROTTLE(3, "============== p_idx  "<< p_idx); 
              }
            }
            if( isStuckRover() ){
              // here in case target in invalid location and rover is stuck
              ac.cancelGoal();          //  cancel current goal to initiate own cmd_vel
              correctStuckRover();
              withOutCancelGoal = true ;//  go back to moveBaseAction cmd_vel planning
              checkAndCorrectCurrGoal();
            }
            refreshMaps();    
            r.sleep();

          }
          ROS_INFO_STREAM_THROTTLE(3, "============== END OF MAIN LOOP ============== "<< new_plan);         
          ac.cancelGoal();
          path_set = false ; 
          requires_reset = true ;
      }

    bool MoveBase::isStuckRover( void ){
        // check if rover has moved 
        double STUCK_DISTANCE = 9.9e-05 ; // 
        if( debug_mode ){
          ROS_INFO_STREAM_THROTTLE(3, "============== isStuckRover =============== "<< getTraveledDistanceMean()); 
        }
        return getTraveledDistanceMean() < STUCK_DISTANCE ;
    }
    void MoveBase::correctStuckRover( void ){
        // for now only reverses a stuck rover
        if( debug_mode ){
          ROS_INFO_STREAM_THROTTLE(3, "============== correctStuckRover =============== "); 
        }
        if( isRoverInLethalCell() ){
          // should reverse 
          // cant turn in place because boulder near
          simpleRoverReverse();
          slowlyStop();
        }
    }


    void MoveBase::moveToAvoidObstacle(void){

      auto cmdPublisher = [&]( auto vX , auto aX , auto aZ ){
        global_cmd_vel.linear.x =  vX ;
        global_cmd_vel.angular.x = aX ;
        global_cmd_vel.angular.z = aZ ;
        vel_pub_.publish(global_cmd_vel);
      };
      double velX , angX , angZ ;
      velX = global_cmd_vel.linear.x ;
      angX = 0 ;
      // vector push in front a point which is a standard vector at safe_angle degrees
      if(debug_mode){
        ROS_INFO_STREAM_THROTTLE(1, "\nEntering moveToAvoidObstacle ");
      }
      double VELOCITY_INCREMENT = .01 ;
      // safe_dist
      ros::Rate r(20);
      while( obstacle_detected ){

        angZ = safe_angle ;

        if( velX < .30 ){
          velX += VELOCITY_INCREMENT ;
        }
        if( 1.0 < safe_dist ){
          cmdPublisher( velX , angX , angZ ); // explicit
        } else {
          cmdPublisher( -1*velX , 0 , 0 ); // reverse
        }
        r.sleep();
      }
    }

    double MoveBase::getRoverYaw( void ){

      double roverRoll , roverPitch , roverYaw ;

      // get roll pitch and yaw
      tf2::Quaternion quat_tf;
      geometry_msgs::Quaternion quat_msg = global_pose_.pose.orientation;
      tf2::convert(quat_msg , quat_tf);
      tf2::Matrix3x3 headingBasedMatrix ( quat_tf );
      unsigned int solution_number = 1 ;
      headingBasedMatrix.getRPY(
             roverRoll , roverPitch , roverYaw , solution_number );
      return roverYaw ;
    }

    double MoveBase::getRelativeAngleToGoal( void ) {
      // transform spaces to get angles
      // Returns :
      //        the angle to the goal in radians
      //
      double roverYaw = getRoverYaw() ;

      // get heading vector
      tf2::Vector3 stdVector (1,0,0);
      tf2::Vector3 rotationalAxis(0,0,1) ;
      tf2::Vector3 headingVector =  stdVector.rotate( rotationalAxis , roverYaw ) ;
      headingVector = headingVector.normalize();

      // simply interpolate to get :
      // between vector defining the rover's position and the goal
      double curr_x , curr_y ;
      curr_x = global_pose_.pose.position.x ;        
      curr_y = global_pose_.pose.position.y ;

      double goal_x , goal_y ;
      goal_x = (*global_plan_)[ p_idx ].pose.position.x ;        
      goal_y = (*global_plan_)[ p_idx ].pose.position.y ; 

      tf2::Vector3 roverVect ( curr_x , curr_y , 0 );
      tf2::Vector3  goalVect ( goal_x , goal_y , 0 );

      // simple interpolation
      tf2::Vector3   difVect ( goalVect - roverVect ); 
      difVect = difVect.normalize();
      // get it's direction
      tf2::Vector3   signVect ( difVect - headingVector );  
      signVect =  signVect.rotate( rotationalAxis , -1*roverYaw ) ;
      double direction = copysign( 1.0 , atan2( signVect.getY(), signVect.getX() ));

      //calculate the angle between the two vectors
      double difAngle = direction * headingVector.angle( difVect ) ;
      if(debug_mode){
        ROS_INFO_STREAM_THROTTLE(3, 
          "\nHeading is "<<headingVector.getX()<<", "<<headingVector.getY()<<"LEN : "<<headingVector.length()
          <<"\ndifVect is "<<difVect.getX()<<", "<<difVect.getY()<<"LEN : "<<difVect.length()
          <<"\nsignVect is "<<signVect.getX()<<", "<<signVect.getY()<<"LEN : "<<signVect.length()
          <<"\nAngle between heading and goal is " << difAngle);
      }
      return difAngle ;
        
    }

    bool MoveBase::goalWithinTurnRange( void ) {
      // Ask whether the goal is greater than the ANGLELIMIT
      // Returns :
      //        bool
      //
      double ANGLELIMIT = M_PI / 3  ;
      
      double difAngle = getRelativeAngleToGoal() ;
      if( debug_mode ){
        ROS_DEBUG_STREAM_THROTTLE(2, "\nAngle between heading and goal is " << difAngle);
      }
      if( ANGLELIMIT < abs(difAngle) ){
          return false ;
      }
      return true ;
        
    }
    void MoveBase::simpleRoverReverse( void ) {
      // hard reverse at REVERSE_VELOCITY effort, for 2 seconds
      // 
      double REVERSE_VELOCITY = 20 ;
      auto cmdPublisher = [&]( auto vX , auto aZ ){
        global_cmd_vel.linear.x =  vX ;
        global_cmd_vel.angular.z = aZ ;
        vel_pub_.publish(global_cmd_vel);
      };
      cmdPublisher( -1*REVERSE_VELOCITY , 0 );
      ros::Duration(4.0).sleep(); 
    }

    bool MoveBase::slowlyStop( void ) {
      // Try to Avoid sliding by stopping slowly
      // Returns :
      //        true on completion
      //
      double startVel , startTheta  ;
      double SCALE = 0.80 ;
      double scalar = 1 ;

      ros::Rate stopRate(8);
      auto updateCurrVel = [&]( void ){
        int sign = copysign( 1 , getRelativeAngleToGoal() );
        startVel = global_cmd_vel.linear.x ;
        startTheta = sign*global_cmd_vel.angular.z ;
      };
      auto cmdPublisher = [&]( auto vX , auto aY ,  auto aZ ){
        global_cmd_vel.linear.x =  vX ;
        global_cmd_vel.angular.y = aY ;
        global_cmd_vel.angular.z = aZ ;
        vel_pub_.publish(global_cmd_vel);
      };
      updateCurrVel();
      if(debug_mode){
        ROS_INFO_STREAM_THROTTLE(2, "START SLOWLY STOP WWWWWWWWWWWWWW startvel and startTheta : " << startVel <<" "<< startTheta);
      }

      // send slow down values 
      while( 0.0001 < abs(global_cmd_vel.linear.x) ){
        updateCurrVel();
        scalar *= SCALE ;
        cmdPublisher( startVel*scalar ,
                        1.0 ,
                      startTheta*scalar) ;
        stopRate.sleep();
      }
      // make wheels go in opposite direction to force stop
      updateCurrVel();
      scalar *= -1 ;
      cmdPublisher( startVel*scalar , 1.0 , startTheta*scalar) ;
      ros::Duration(4.0).sleep(); // simplest avoid slip solution, not the most robust TODO
      
      // once 'stopped' continue
      cmdPublisher( 0.0 , 1.0 , 0.0 ) ;
      ros::Duration(4.0).sleep(); // simplest avoid slip solution, not the most robust TODO
      cmdPublisher( 0.0 , 0.0 , 0.0 ) ;
      if(debug_mode){
        ROS_INFO_STREAM_THROTTLE(2, "END SLOWLYSTOP END SLOWLYSTOP NOWWWWWWWWWWWWWWWWWWWWWWWWWWWW");
      }
      return true ;
        
    }

    bool MoveBase::explicitTurnSequence( void ) {
      // Ask whether the goal is greater than the ANGLELIMIT
      // Returns :
      //        true on completion
      //
      double INNERLIMIT = M_PI/3 ;
      double BEGINSLOWLIMIT = 1 ;

      auto cmdPublisher = [&]( auto vX , auto aX , auto aZ ){
        global_cmd_vel.linear.x =  vX ;
        global_cmd_vel.angular.x = aX ;
        global_cmd_vel.angular.z = aZ ;
        vel_pub_.publish(global_cmd_vel);
      };

      slowlyStop() ;
      cmdPublisher( 0.0 , 0.0 , 0.0 ) ; 
      // Turn
      ros::Rate angleCheckRate(10);
      double difAngle = getRelativeAngleToGoal() ;
      double velX = 0.15 ;
      double startVelX = velX ;
      double hillEffortMultiplier = 1 ;
      double currYaw  = 0 ;     
      double prevYaw  = currYaw + 1 ; // just making it different

      while( INNERLIMIT < abs(difAngle) ){
        // ease into alignment with goal
        currYaw  = getRoverYaw() ;
        if( abs(difAngle) < BEGINSLOWLIMIT ){
          velX = (startVelX*difAngle)*hillEffortMultiplier ; 
          // TODO : hillEffortMultiplier
          //   remove/add more effort depending if 
          //   ekf says wer're oriented the same way ( example on a hill )
          if( round(100*prevYaw) == round(100*currYaw) ){
            hillEffortMultiplier += .01 ;
          }
        }
        cmdPublisher( velX , 1.0 , difAngle ) ;
        difAngle = getRelativeAngleToGoal();
        prevYaw = currYaw ;
        ros::spinOnce();
        angleCheckRate.sleep();

      }

      slowlyStop();
      cmdPublisher( 0.0 , 0.0 , 0.0 ) ; // for angX

      return true ;
        
    }

    bool MoveBase::isAtGoal( void ) {
        double MIN_GOAL_DISTANCE = 1 ;

        double curr_x , curr_y ;
        curr_x = global_pose_.pose.position.x ;        
        curr_y = global_pose_.pose.position.y ;    
        double goal_x , goal_y ;
        goal_x = (*global_plan_)[ p_idx ].pose.position.x ;        
        goal_y = (*global_plan_)[ p_idx ].pose.position.y ; 

        double distance = sqrt( pow( curr_x - goal_x ,2)+pow( curr_y - goal_y ,2) );

        if( distance < MIN_GOAL_DISTANCE ){
            ROS_DEBUG( "Arrived isAtGoal.");
            return true ;
        }
        ROS_DEBUG_STREAM_THROTTLE(2, "In isAtGoal. : " );

        return false ;
    }

    bool MoveBase::isLethalGoal( double wx , double wy ) {
      // checks whether the world coordinates in the costmap are lethal
      // 
      // Lethal here is defined by our own measure as
      // Inscribed or Lethal 
      unsigned int mx ,my ; // map coordinates 
      controller_costmap_ros_->getCostmap()->worldToMap( wx , wy , mx , my );
      unsigned int cost = (unsigned int)controller_costmap_ros_->getCostmap()->getCost( mx,  my) ;

      int lethal = 60 ;
      if( lethal <= cost  
         && cost != costmap_2d::NO_INFORMATION ){
        ROS_DEBUG_STREAM( "COST : "<< cost); 
        return true ;
      }
      if( debug_mode ){
        ROS_INFO_STREAM_THROTTLE(3, "COST : "<< cost); 
      }
      return false ;
    }
    bool MoveBase::isRoverInLethalCell( void ) {
      // checks whether the rover in lethal coordinate
      // 
      // Lethal here is defined by our own measure as
      // Inscribed or Lethal 
      double curr_x , curr_y ;
      curr_x = global_pose_.pose.position.x ;        
      curr_y = global_pose_.pose.position.y ; 
      unsigned int mx ,my ; // map coordinates 
      controller_costmap_ros_->getCostmap()->worldToMap( curr_x , curr_y , mx , my );
      unsigned int cost = (unsigned int)controller_costmap_ros_->getCostmap()->getCost( mx,  my) ;

      int LETHAL = 60 ; //costmap_2d::LETHAL_OBSTACLE ;
      if( LETHAL <= cost ){
        ROS_DEBUG_STREAM( "++++++++++ Rover location COST : "<< cost); 
        return true ;
      }
      if( debug_mode ){
        ROS_INFO_STREAM_THROTTLE(3, "++------++ Rover location COST : "<< cost); 
      }
      return false ;
    }
    
    bool MoveBase::currGoalInLocalMap( void ) {
      // checks whether the world coordinates in the local costmap 
      // prevents the isLethalGoal(...) frmo cycling through all waypoints too fast
      // by maintaining out of bound coordinates within the bounds. 
      bool inMap = false ; 
      double curr_x , curr_y ;
      curr_x = (*global_plan_)[ p_idx ].pose.position.x ;
      curr_y = (*global_plan_)[ p_idx ].pose.position.y ;
      unsigned int mx ,my ; // map coordinates
      inMap = controller_costmap_ros_->getCostmap()->worldToMap( curr_x , curr_y , mx , my );
      if( debug_mode ){
        ROS_INFO_STREAM_THROTTLE(3, "currGoalInLocalMap : mx " << mx << " my "<< my
                                          << "next_x " << curr_x << " next_y "<< curr_y );
      }
      if( inMap ){
        ROS_INFO_STREAM_THROTTLE(3, "currGoalInLocalMap True " ); 
        return true ;
      }
      ROS_DEBUG_STREAM( "NOT currGoalInLocalMap ."); 
      return false ;
    }

    bool MoveBase::nextGoalInLocalMap( void ) {
      // checks whether the world coordinates in the local costmap 
      // prevents the isLethalGoal(...) frmo cycling through all waypoints too fast
      // by maintaining out of bound coordinates within the bounds. 
      bool inMap = false ; 
      double next_x , next_y ;
      next_x = (*global_plan_)[ p_idx + 1 ].pose.position.x ;
      next_y = (*global_plan_)[ p_idx + 1 ].pose.position.y ;
      unsigned int mx ,my ; // map coordinates
      inMap = controller_costmap_ros_->getCostmap()->worldToMap( next_x , next_y , mx , my );
      ROS_INFO_STREAM_THROTTLE(3, "nextGoalInLocalMap : mx " << mx << " my "<< my
                                          << "next_x " << next_x << " next_y "<< next_y );
      if( inMap ){
        ROS_INFO_STREAM_THROTTLE(3, "nextGoalInLocalMap : " ); 
        return true ;
      }
      ROS_DEBUG_STREAM( "NOT nextGoalInLocalMap ."); 
      return false ;
    }
    bool MoveBase::findSafeGoalInMapFromCurrent( void ) {
      // TODO : if( p_idx + 1 == indexOutOfBounds ) return false ;
      
      ///////////////////////// 
      // MAIN FIT 
      // get the out of bounds waypoint 
      double curr_x , curr_y ; 
      curr_x = (*global_plan_)[ p_idx ].pose.position.x ;
      curr_y = (*global_plan_)[ p_idx ].pose.position.y ;

      // fit to map
      int fitted_x ,fitted_y ; // map coordinates--- signed
      controller_costmap_ros_->getCostmap()->worldToMapEnforceBounds(
                                                    curr_x , curr_y , fitted_x , fitted_y );
      // get the coordinates in the world frame to later send
      unsigned int mx ,my ; // map coordinates ----- unsigned
      mx = (unsigned int)fitted_x ;
      my = (unsigned int)fitted_y ;
      controller_costmap_ros_->getCostmap()->mapToWorld( mx , my , curr_x , curr_y );
      // END MAIN FIT
      ///////////////////////// 

      ///////////////////////// 
      // CLOSER TO ROVER FIT 
      // AND NON-LETHAL
      // get dif between cur goal and rover location
      double SCALE = 0.80 ;
      double diff_vec[] = {0,0} ;
      diff_vec[0] = SCALE*(curr_x - global_pose_.pose.position.x) ;        
      diff_vec[1] = SCALE*(curr_y - global_pose_.pose.position.y) ; 

      int LETHAL = 60 ; 
      unsigned int cost , rotation_direction ;
      double angle = M_PI/6 ;
      // LAMDAS
      auto calculateRotation = [&]( double ang ){
        double rotated_x = (cos(ang)*diff_vec[0] - sin(ang)*diff_vec[1]);
        double rotated_y = (sin(ang)*diff_vec[0] + cos(ang)*diff_vec[1]);
        curr_x = rotated_x + global_pose_.pose.position.x ;
        curr_y = rotated_y + global_pose_.pose.position.y ;
      };
      auto calculateRotationCost = [&]( void ){
        controller_costmap_ros_->getCostmap()->worldToMap( curr_x , curr_y , mx , my );
        cost = (unsigned int)controller_costmap_ros_->getCostmap()->getCost( mx,  my) ;
      };
      // Test in which direction the new target can not be lethal in
      // unless in crater... there should be a non-lethal target by the end of the forloop
      for(; angle < M_PI ; angle += M_PI/6 ){
        calculateRotation( angle );
        calculateRotationCost();
        if( cost < LETHAL){ break ;}
        calculateRotation( -1*angle );
        calculateRotationCost();
        if( cost < LETHAL){ break ;}
      }

      // CLOSER TO ROVER FIT 
      // AND NON-LETHAL
      ///////////////////////// 
      /////////////////////////

      // update the goal pose
      (*global_plan_)[ p_idx ].pose.position.x = curr_x ;        
      (*global_plan_)[ p_idx ].pose.position.y = curr_y ;
      return true ; 
    }

    bool MoveBase::fitNextGoalToMap( void ) {
      // see :
      // http://docs.ros.org/indigo/api/costmap_2d/html/costmap__2d_8cpp_source.html#l00208
      // for rational on not casting fitted_* ---> m*
      // longstory short, "fun()const"
      
      // TODO : if( p_idx + 1 == indexOutOfBounds ) return false ;
        
      ///////////////////////// 
      // MAIN FIT 
      // get the out of bounds waypoint 
      double next_x , next_y ; 
      next_x = (*global_plan_)[ p_idx + 1 ].pose.position.x ;
      next_y = (*global_plan_)[ p_idx + 1 ].pose.position.y ;

      // fit to map
      int fitted_x ,fitted_y ; // map coordinates--- signed
      controller_costmap_ros_->getCostmap()->worldToMapEnforceBounds(
                                                    next_x , next_y , fitted_x , fitted_y );
      // get the coordinates in the world frame to later send
      unsigned int mx ,my ; // map coordinates ----- unsigned
      mx = (unsigned int)fitted_x ;
      my = (unsigned int)fitted_y ;
      controller_costmap_ros_->getCostmap()->mapToWorld( mx , my , next_x , next_y );
      // END MAIN FIT
      ///////////////////////// 

      ///////////////////////// 
      // CLOSER TO ROVER FIT 
      // get dif between cur goal and rover location
      double diff_x , diff_y ;
      diff_x = next_x - global_pose_.pose.position.x ;        
      diff_y = next_y - global_pose_.pose.position.y ; 

      // scale the resultant vector and add it to rover current location
      // new goal is now garunteed to be within the maps for some driving time.
      double SCALE = 0.80 ;
      next_x = SCALE*diff_x + global_pose_.pose.position.x ;        
      next_y = SCALE*diff_y + global_pose_.pose.position.y ; 
      // CLOSER TO ROVER FIT 
      /////////////////////////

      // update the goal pose
      (*global_plan_)[ p_idx + 1 ].pose.position.x = next_x ;        
      (*global_plan_)[ p_idx + 1 ].pose.position.y = next_y ;
      ROS_INFO_STREAM_THROTTLE(3, "\nFitted_ : ( " << fitted_x << ", "<<fitted_y<<" )"<<
                                       "\nNext    : ( "<< next_x << ", "<< next_y <<" )"); 
      return true ; 
    }

    bool MoveBase::fitCurrGoalToMap( void ) {
      // see :
      // http://docs.ros.org/indigo/api/costmap_2d/html/costmap__2d_8cpp_source.html#l00208
      // for rational on not casting fitted_* ---> m*
      // longstory short, "fun()const"
      
      // TODO : if( p_idx + 1 == indexOutOfBounds ) return false ;
      ///////////////////////// 
      // MAIN FIT 
      // get the out of bounds waypoint 
      double curr_x , curr_y ; 
      curr_x = (*global_plan_)[ p_idx ].pose.position.x ;
      curr_y = (*global_plan_)[ p_idx ].pose.position.y ;

      // fit to map
      int fitted_x ,fitted_y ; // map coordinates--- signed
      controller_costmap_ros_->getCostmap()->worldToMapEnforceBounds(
                                                    curr_x , curr_y , fitted_x , fitted_y );
      // get the coordinates in the world frame to later send
      unsigned int mx ,my ; // map coordinates ----- unsigned
      mx = (unsigned int)fitted_x ;
      my = (unsigned int)fitted_y ;
      controller_costmap_ros_->getCostmap()->mapToWorld( mx , my , curr_x , curr_y );
      // END MAIN FIT
      ///////////////////////// 

      ///////////////////////// 
      // CLOSER TO ROVER FIT 
      // get dif between cur goal and rover location
      double diff_x , diff_y ;
      diff_x = curr_x - global_pose_.pose.position.x ;        
      diff_y = curr_y - global_pose_.pose.position.y ; 

      // scale the resultant vector and add it to rover current location
      // new goal is now garunteed to be within the maps for some driving time.
      double SCALE = 0.80 ;
      curr_x = SCALE*diff_x + global_pose_.pose.position.x ;        
      curr_y = SCALE*diff_y + global_pose_.pose.position.y ; 
      // CLOSER TO ROVER FIT 
      ///////////////////////// 

      // update the goal pose
      (*global_plan_)[ p_idx ].pose.position.x = curr_x ;        
      (*global_plan_)[ p_idx ].pose.position.y = curr_y ;
      ROS_INFO_STREAM_THROTTLE(3, "\nFitted_ : ( " << fitted_x << ", "<<fitted_y<<" )"<<
                                       "\nNext    : ( "<< curr_x << ", "<< curr_y <<" )"); 
      return true ; 
    }
   
    bool MoveBase::notSameGoalAsLast( void ) {

        double curr_goal_x , curr_goal_y ;
        curr_goal_x = (*global_plan_)[ p_idx ].pose.position.x ;        
        curr_goal_y = (*global_plan_)[ p_idx ].pose.position.y ; 

        if( round(curr_goal_x) == round(last_goal_x)
            && round(curr_goal_y) == round(last_goal_y)){
            return false ;
        }
        return true ;
    }
    
    void MoveBase::roverOdomCb(const nav_msgs::Odometry::ConstPtr& odom) {
        // update accumulator with distance from last reading
        double diffFromLastReading = sqrt( pow( odom->pose.pose.position.x - global_pose_.pose.position.x , 2)  \
                                            + pow( odom->pose.pose.position.y - global_pose_.pose.position.y , 2) );
        dist_accumulator( diffFromLastReading ) ; // size 60 // +/- 3 seconds of some place

        // save rover's global_pose
        global_pose_.pose = odom->pose.pose;
        global_pose_.header = odom->header;

        global_pose_.header.frame_id = "map";
    }

    void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
        boost::recursive_mutex::scoped_lock l(configuration_mutex_);

        //The first time we're called, we just want to make sure we have the
        //original configuration
        if(!setup_){
            last_config_ = config;
            default_config_ = config;
            setup_ = true;
            return;
        }

        if(config.restore_defaults) {
            config = default_config_;
            //if someone sets restore defaults on the parameter server, prevent looping
            config.restore_defaults = false;
        }

        if(planner_frequency_ != config.planner_frequency){
            planner_frequency_ = config.planner_frequency;
            p_freq_change_ = true;
        }

        if(controller_frequency_ != config.controller_frequency){
            controller_frequency_ = config.controller_frequency;
            c_freq_change_ = true;
        }

        planner_patience_ = config.planner_patience;
        controller_patience_ = config.controller_patience;
        max_planning_retries_ = config.max_planning_retries;
        conservative_reset_dist_ = config.conservative_reset_dist;

        recovery_behavior_enabled_ = config.recovery_behavior_enabled;
        clearing_rotation_allowed_ = config.clearing_rotation_allowed;
        shutdown_costmaps_ = config.shutdown_costmaps;

        oscillation_timeout_ = config.oscillation_timeout;
        oscillation_distance_ = config.oscillation_distance;
        if(config.base_global_planner != last_config_.base_global_planner) {
            boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
            //initialize the global planner
            ROS_DEBUG("Loading global planner %s", config.base_global_planner.c_str());
            try {
                planner_ = bgp_loader_.createInstance(config.base_global_planner);

                // wait for the current planner to finish planning
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

                // Clean up before initializing the new planner
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

                lock.unlock();
            } catch (const pluginlib::PluginlibException& ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
                planner_ = old_planner;
                config.base_global_planner = last_config_.base_global_planner;
            }
        }

        if(config.base_local_planner != last_config_.base_local_planner){
            boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
            //create a local planner
            try {
                tc_ = blp_loader_.createInstance(config.base_local_planner);
                // Clean up before initializing the new planner
                planner_plan_->clear();
                latest_plan_->clear();
                controller_plan_->clear();
                resetState();
                tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
            } catch (const pluginlib::PluginlibException& ex) {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
                tc_ = old_planner;
                config.base_local_planner = last_config_.base_local_planner;
            }
        }

//        make_plan_clear_costmap_ = config.make_plan_clear_costmap;
//        make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

        last_config_ = config;
    }

    void MoveBase::clearCostmapWindows(double size_x, double size_y){

        // TODO: double check this
        ROS_WARN("This is not safe! ");
        ROS_WARN("clearCostmapWindow has not been tested");

        geometry_msgs::PoseStamped global_pose;

        //clear the planner's costmap
        getRobotPose(global_pose, planner_costmap_ros_);

        std::vector<geometry_msgs::Point> clear_poly;
        double x = global_pose.pose.position.x;
        double y = global_pose.pose.position.y;
        geometry_msgs::Point pt;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

        //clear the controller's costmap
        getRobotPose(global_pose, controller_costmap_ros_);

        clear_poly.clear();
        x = global_pose.pose.position.x;
        y = global_pose.pose.position.y;

        pt.x = x - size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y - size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x + size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        pt.x = x - size_x / 2;
        pt.y = y + size_y / 2;
        clear_poly.push_back(pt);

        controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
    }

    bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
        //clear the costmaps
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
        controller_costmap_ros_->resetLayers();

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
        planner_costmap_ros_->resetLayers();
        return true;
    }


    bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
        if(as_->isActive()){
            ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
            return false;
        }
        //make sure we have a costmap for our planner
        if(planner_costmap_ros_ == nullptr){
            ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
            return false;
        }

        geometry_msgs::PoseStamped start;
        //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
        if(req.start.header.frame_id.empty()){
            start = global_pose_;
        }
        else{
            start = req.start;
        }

        if (make_plan_clear_costmap_) {
            //update the copy of the costmap the planner uses
            clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
        }

        //first try to make a plan to the exact desired goal
        std::vector<geometry_msgs::PoseStamped> global_plan;
        if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
            ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
                            req.goal.pose.position.x, req.goal.pose.position.y);

            //search outwards for a feasible goal within the specified tolerance
            geometry_msgs::PoseStamped p;
            p = req.goal;
            bool found_legal = false;
            float resolution = planner_costmap_ros_->getCostmap()->getResolution();
            float search_increment = resolution*3.0;
            if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
            for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
                for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
                    for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

                        //don't search again inside the current outer layer
                        if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

                        //search to both sides of the desired goal
                        for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

                            //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
                            if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

                            for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                                if(planner_->makePlan(start, p, global_plan)){
                                    if(!global_plan.empty()){

                                        if (make_plan_add_unreachable_goal_) {
                                            //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                                            //(the reachable goal should have been added by the global planner)
                                            global_plan.push_back(req.goal);
                                        }

                                        found_legal = true;
                                        ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                        break;
                                    }
                                }
                                else{
                                    ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                                }
                            }
                        }
                    }
                }
            }
        }

        //copy the plan into a message to send out
        resp.plan.poses.resize(global_plan.size());
        for(unsigned int i = 0; i < global_plan.size(); ++i){
            resp.plan.poses[i] = global_plan[i];
        }

        return true;
    }

    MoveBase::~MoveBase(){
        recovery_behaviors_.clear();

        delete dsrv_;
        delete as_;
        delete planner_costmap_ros_;
        delete controller_costmap_ros_;

        planner_thread_->interrupt();
        planner_thread_->join();

        delete planner_thread_;

        client_thread_->interrupt();
        client_thread_->join();
        delete client_thread_;

        delete planner_plan_;
        delete latest_plan_;
        delete controller_plan_;

        planner_.reset();
        tc_.reset();
    }

    bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

        //make sure to set the plan to be empty initially
        plan.clear();

        //since this gets called on handle activate
        if(planner_costmap_ros_ == nullptr) {
            ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
            return false;
        }

        const geometry_msgs::PoseStamped& start = global_pose_;

        //if the planner fails or returns a zero length plan, planning failed
        if(!planner_->makePlan(start, goal, plan) || plan.empty()){
            ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
            return false;
        }

        return true;
    }

    void MoveBase::publishZeroVelocity(){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub_.publish(cmd_vel);
    }

    bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
        //first we need to check if the quaternion has nan's or infs
        if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
            ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
            return false;
        }

        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

        //next, we need to check if the length of the quaternion is close to zero
        if(tf_q.length2() < 1e-6){
            ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
            return false;
        }

        //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
        tf_q.normalize();

        tf2::Vector3 up(0, 0, 1);

        double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

        if(fabs(dot - 1) > 1e-3){
            ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
            return false;
        }

        return true;
    }

    // TODO: Create a method to transform odom message to map frame

    geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
        std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
        geometry_msgs::PoseStamped goal_pose, global_pose;
        goal_pose = goal_pose_msg;

        //just get the latest available transform... for accuracy they should send
        //goals in the frame of the planner
        goal_pose.header.stamp = ros::Time();

        try{
            tf_.transform(goal_pose_msg, global_pose, global_frame);
        }
        catch(tf2::TransformException& ex){
            ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                     goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
            return goal_pose_msg;
        }

        return global_pose;
    }

    void MoveBase::wakePlanner(const ros::TimerEvent& event)
    {
        // we have slept long enough for rate
        planner_cond_.notify_one();
    }

    void MoveBase::planThread(){
        ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
        ros::NodeHandle n;
        ros::Timer timer;
        bool wait_for_wake = false;
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        while(n.ok()){
            //check if we should run the planner (the mutex is locked)
            while(wait_for_wake || !runPlanner_){
                //if we should not be running the planner then suspend this thread
                ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
                planner_cond_.wait(lock);
                wait_for_wake = false;
            }
            ros::Time start_time = ros::Time::now();

            //time to plan! get a copy of the goal and unlock the mutex
            geometry_msgs::PoseStamped temp_goal = planner_goal_;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

            //run planner
            planner_plan_->clear();
            bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

            if(gotPlan){
                ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
                //pointer swap the plans under mutex (the controller will pull from latest_plan_)
                std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

                lock.lock();
                planner_plan_ = latest_plan_;
                latest_plan_ = temp_plan;
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                new_global_plan_ = true;

                ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

                //make sure we only start the controller if we still haven't reached the goal
                if(runPlanner_)
                    state_ = CONTROLLING;
                if(planner_frequency_ <= 0)
                    runPlanner_ = false;
                lock.unlock();
            }
                //if we didn't get a plan and we are in the planning state (the robot isn't moving)
            else if(state_==PLANNING){
                ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
                ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

                //check if we've tried to make a plan for over our time limit or our maximum number of retries
                //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
                //is negative (the default), it is just ignored and we have the same behavior as ever
                lock.lock();
                planning_retries_++;
                if(runPlanner_ &&
                   (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
                    //we'll move into our obstacle clearing mode
                    state_ = CLEARING;
                    runPlanner_ = false;  // proper solution for issue #523
                    publishZeroVelocity();
                    recovery_trigger_ = PLANNING_R;
                }

                lock.unlock();
            }

            //take the mutex for the next iteration
            lock.lock();

            //setup sleep interface if needed
            if(planner_frequency_ > 0){
                ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
                if (sleep_time > ros::Duration(0.0)){
                    wait_for_wake = true;
                    timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
                }
            }
        }
    }

    void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
    {
        if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
        }

        geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

        publishZeroVelocity();
        //we have a goal so start the planner
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        current_goal_pub_.publish(goal);
        std::vector<geometry_msgs::PoseStamped> global_plan;

        ros::Rate r(controller_frequency_);
        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
            planner_costmap_ros_->start();
            controller_costmap_ros_->start();
        }

        //we want to make sure that we reset the last time we had a valid plan and control
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;

        ros::NodeHandle n;
        while(n.ok())
        {
            if(c_freq_change_)
            {
                ROS_DEBUG("Setting controller frequency to %.2f", controller_frequency_);
                r = ros::Rate(controller_frequency_);
                c_freq_change_ = false;
            }

            if(as_->isPreemptRequested()){
                if(as_->isNewGoalAvailable()){
                    //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
                    move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

                    if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
                        return;
                    }

                    goal = goalToGlobalFrame(new_goal.target_pose);

                    //we'll make sure that we reset our state for the next execution cycle
                    recovery_index_ = 0;
                    state_ = PLANNING;

                    //we have a new goal so make sure the planner is awake
                    lock.lock();
                    planner_goal_ = goal;
                    runPlanner_ = true;
                    planner_cond_.notify_one();
                    lock.unlock();

                    //publish the goal point to the visualizer
                    ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
                    current_goal_pub_.publish(goal);

                    //make sure to reset our timeouts and counters
                    last_valid_control_ = ros::Time::now();
                    last_valid_plan_ = ros::Time::now();
                    last_oscillation_reset_ = ros::Time::now();
                    planning_retries_ = 0;
                }
                else {
                    //if we've been preempted explicitly we need to shut things down
                    resetState();

                    //notify the ActionServer that we've successfully preempted
                    ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
                    as_->setPreempted();

                    //we'll actually return from execute after preempting
                    return;
                }
            }

            //we also want to check if we've changed global frames because we need to transform our goal pose
            if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
                goal = goalToGlobalFrame(goal);

                //we want to go back to the planning state for the next execution cycle
                recovery_index_ = 0;
                state_ = PLANNING;

                //we have a new goal so make sure the planner is awake
                lock.lock();
                planner_goal_ = goal;
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();

                //publish the goal point to the visualizer
                ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
                current_goal_pub_.publish(goal);

                //make sure to reset our timeouts and counters
                last_valid_control_ = ros::Time::now();
                last_valid_plan_ = ros::Time::now();
                last_oscillation_reset_ = ros::Time::now();
                planning_retries_ = 0;
            }

            //for timing that gives real time even in simulation
            ros::WallTime start = ros::WallTime::now();

            //the real work on pursuing a goal is done here
            bool done = executeCycle(goal, global_plan);

            //if we're done, then we'll return from execute
            if(done)
                return;

            //check if execution of the goal has completed in some way

            ros::WallDuration t_diff = ros::WallTime::now() - start;
            ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

            r.sleep();
            //make sure to sleep for the remainder of our cycle time
            if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
                ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
        }

        //wake up the planner thread so that it can exit cleanly
        lock.lock();
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //if the node is killed then we'll abort and return
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
        return;
    }

    double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
    {
        return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
    }

    bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
        boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
        geometry_msgs::Twist cmd_vel;

        //update feedback to correspond to our curent position
        // TODO: might be an place for error
        const geometry_msgs::PoseStamped& current_position = global_pose_;

        //push the feedback out
        move_base_msgs::MoveBaseFeedback feedback;
        feedback.base_position = current_position;
        as_->publishFeedback(feedback);

        //check to see if we've moved far enough to reset our oscillation timeout
        if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
        {
            last_oscillation_reset_ = ros::Time::now();
            oscillation_pose_ = current_position;

            //if our last recovery was caused by oscillation, we want to reset the recovery index
            if(recovery_trigger_ == OSCILLATION_R)
                recovery_index_ = 0;
        }

        //check that the observation buffers for the costmap are current, we don't want to drive blind
        if(!controller_costmap_ros_->isCurrent()){
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
            publishZeroVelocity();
            return false;
        }

        //if we have a new plan then grab it and give it to the controller
        if(new_global_plan_){
            //make sure to set the new plan flag to false
            new_global_plan_ = false;

            ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

            //do a pointer swap under mutex
            std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            controller_plan_ = latest_plan_;
            latest_plan_ = temp_plan;
            lock.unlock();
            ROS_DEBUG_NAMED("move_base","pointers swapped!");

            if(!tc_->setPlan(*controller_plan_)){
                //ABORT and SHUTDOWN COSTMAPS
                ROS_ERROR("Failed to pass global plan to the controller, aborting.");
                resetState();

                //disable the planner thread
                lock.lock();
                runPlanner_ = false;
                lock.unlock();

                as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
                return true;
            }

            //make sure to reset recovery_index_ since we were able to find a valid plan
            if(recovery_trigger_ == PLANNING_R)
                recovery_index_ = 0;
        }

        //the move_base state machine, handles the control logic for navigation
        switch(state_){
            //if we are in a planning state, then we'll attempt to make a plan
            case PLANNING:
            {
                boost::recursive_mutex::scoped_lock lock(planner_mutex_);
                runPlanner_ = true;
                planner_cond_.notify_one();
            }
                ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
                break;

                //if we're controlling, we'll attempt to find valid velocity commands
            case CONTROLLING:
                ROS_DEBUG_NAMED("move_base","In controlling state.");

                //check to see if we've reached our goal
                if(tc_->isGoalReached()){
                    ROS_DEBUG_NAMED("move_base","Goal reached!");
                    resetState();

                    //disable the planner thread
                    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
                    return true;
                }

                //check for an oscillation condition
                if(oscillation_timeout_ > 0.0 &&
                   last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
                {
                    publishZeroVelocity();
                    state_ = CLEARING;
                    recovery_trigger_ = OSCILLATION_R;
                }

                {
                    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

                    if(tc_->computeVelocityCommands(cmd_vel)){
                        ROS_DEBUG_THROTTLE( 1, "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                         cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
                        global_cmd_vel = cmd_vel ;
                        last_valid_control_ = ros::Time::now();
                        //make sure that we send the velocity command to the base
                        vel_pub_.publish(cmd_vel);
                        if(recovery_trigger_ == CONTROLLING_R)
                            recovery_index_ = 0;
                    }
                    else {
                        ROS_DEBUG_THROTTLE(1, "The local planner could not find a valid plan.");
                        ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                        //check if we've tried to find a valid control for longer than our time limit
                        if(ros::Time::now() > attempt_end){
                            //we'll move into our obstacle clearing mode
                            publishZeroVelocity();
                            state_ = CLEARING;
                            recovery_trigger_ = CONTROLLING_R;
                        }
                        else{
                            //otherwise, if we can't find a valid control, we'll go back to planning
                            last_valid_plan_ = ros::Time::now();
                            planning_retries_ = 0;
                            state_ = PLANNING;
                            publishZeroVelocity();

                            //enable the planner thread in case it isn't running on a clock
                            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                            runPlanner_ = true;
                            planner_cond_.notify_one();
                            lock.unlock();
                        }
                    }
                }

                break;

                //we'll try to clear out space with any user-provided recovery behaviors
            case CLEARING:
                ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
                //we'll invoke whatever recovery behavior we're currently on if they're enabled
                if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
                    ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
                    recovery_behaviors_[recovery_index_]->runBehavior();

                    //we at least want to give the robot some time to stop oscillating after executing the behavior
                    last_oscillation_reset_ = ros::Time::now();

                    //we'll check if the recovery behavior actually worked
                    ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
                    last_valid_plan_ = ros::Time::now();
                    planning_retries_ = 0;
                    state_ = PLANNING;

                    //update the index of the next recovery behavior that we'll try
                    recovery_index_++;
                }
                else{
                    ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
                    //disable the planner thread
                    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                    runPlanner_ = false;
                    lock.unlock();

                    ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

                    if(recovery_trigger_ == CONTROLLING_R){
                        ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
                    }
                    else if(recovery_trigger_ == PLANNING_R){
                        ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
                    }
                    else if(recovery_trigger_ == OSCILLATION_R){
                        ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                        as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
                    }
                    resetState();
                    return true;
                }
                break;
            default:
                ROS_ERROR("This case should never be reached, something is wrong, aborting");
                resetState();
                //disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();
                as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
                return true;
        }

        //we aren't done yet
        return false;
    }

    bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
        XmlRpc::XmlRpcValue behavior_list;
        if(node.getParam("recovery_behaviors", behavior_list)){
            if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
                for(int i = 0; i < behavior_list.size(); ++i){
                    if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                        if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
                            //check for recovery behaviors with the same name
                            for(int j = i + 1; j < behavior_list.size(); j++){
                                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                                    if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                                        std::string name_i = behavior_list[i]["name"];
                                        std::string name_j = behavior_list[j]["name"];
                                        if(name_i == name_j){
                                            ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                                      name_i.c_str());
                                            return false;
                                        }
                                    }
                                }
                            }
                        }
                        else{
                            ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
                            return false;
                        }
                    }
                    else{
                        ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                                  behavior_list[i].getType());
                        return false;
                    }
                }

                //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
                for(int i = 0; i < behavior_list.size(); ++i){
                    try{
                        //check if a non fully qualified name has potentially been passed in
                        if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
                            std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                            for(unsigned int i = 0; i < classes.size(); ++i){
                                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                                    ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                             std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                                    behavior_list[i]["type"] = classes[i];
                                    break;
                                }
                            }
                        }

                        boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

                        //shouldn't be possible, but it won't hurt to check
                        if(behavior.get() == NULL){
                            ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
                            return false;
                        }

                        //initialize the recovery behavior with its name
                        behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
                        recovery_behaviors_.push_back(behavior);
                    }
                    catch(pluginlib::PluginlibException& ex){
                        ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
                        return false;
                    }
                }
            }
            else{
                ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                          behavior_list.getType());
                return false;
            }
        }
        else{
            ROS_ERROR("No recovery_behaviors are specified, loading defaults . ");
            return false;
        }

        //if we've made it here... we've constructed a recovery behavior list successfully
        return true;
    }

    //we'll load our default recovery behaviors here
    void MoveBase::loadDefaultRecoveryBehaviors(){
        recovery_behaviors_.clear();
        ROS_ERROR("Custom recovery failed. Loading default recovery_behaviors_ . ");
        try{
            //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
            ros::NodeHandle n("~");
            n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
            n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

            //first, we'll load a recovery behavior to clear the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(cons_clear);

            //next, we'll load a recovery behavior to rotate in place
            boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
            if(clearing_rotation_allowed_){
                rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
                recovery_behaviors_.push_back(rotate);
            }

            //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
            boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
            ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(ags_clear);

            //we'll rotate in-place one more time
            if(clearing_rotation_allowed_)
                recovery_behaviors_.push_back(rotate);
        }
        catch(pluginlib::PluginlibException& ex){
            ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
        }

        return;
    }

    void MoveBase::resetState(){
        // Disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        // Reset statemachine
        state_ = PLANNING;
        recovery_index_ = 0;
        recovery_trigger_ = PLANNING_R;
        publishZeroVelocity();

        //if we shutdown our costmaps when we're deactivated... we'll do that now
        if(shutdown_costmaps_){
            ROS_DEBUG_NAMED("move_base","Stopping costmaps");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }
    }

    bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap){

        tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
        geometry_msgs::PoseStamped robot_pose;
        tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
        robot_pose.header.frame_id = robot_base_frame_;
        robot_pose.header.stamp = ros::Time(); // latest available
        ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

        // get robot pose on the given costmap frame
        try{
            tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
        }
        catch (tf2::LookupException& ex){
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ConnectivityException& ex){
            ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf2::ExtrapolationException& ex){
            ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
            return false;
        }

        // check if global_pose time stamp is within costmap transform tolerance
        if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance()){
            ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                              current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
            return false;
        }

        return true;
    }
};
