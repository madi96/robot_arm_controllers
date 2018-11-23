/**
  08-11-2018- Dream-Project
  Purpose: Provides primitives to move a robot's arm group.

  @author Oussama YAAKOUBI
  @version
*/


#ifndef PRIMITIVE_HPP
#define PRIMITIVE_HPP

#include <iostream>
#include <string>
#include <boost/timer.hpp>
#include <vector>
#include <numeric>

#include <ros/callback_queue.h>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <boost/timer.hpp>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#ifdef ROBOT_IS_BAXTER
#		include <baxter_core_msgs/EndEffectorCommand.h>
#endif

#ifdef ROBOT_IS_PR2
#		include <pr2_controllers_msgs/Pr2GripperCommand.h>
#endif


class Primitive {
	
    protected:
		tf::Transform goalToRobotTransform_, approachToRobotTransform_, extendedGoalToRobotTransform_;
		const std::string group_names_array_[2]  = {"left_arm", "right_arm"};
        std::vector<double> goal_, goal_normal_;
        std::string chosen_arm_group_name_;
        XmlRpc::XmlRpcValue planner_params_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> chosen_arm_group_, left_arm_group_, right_arm_group_;
        std::unique_ptr<ros::AsyncSpinner> asyn_spinner_;
        const double wrist_rot_array_[4] = {0, M_PI/4, M_PI/2, 3*M_PI/4};
        int rand_rotation_value_index_ = rand() % 4;
        ros::Publisher  chosen_gripper_pub_, right_gripper_pub_, left_gripper_pub_;
        robot_state::RobotStatePtr robot_state_;
		std::string base_frame_, left_gripper_link_name_, chosen_gripper_link_name_, right_gripper_link_name_;

        int right_gripper_id_;
        int left_gripper_id_;

        std::unique_ptr<ros::ServiceClient> planning_scene_serv_;
        std::unique_ptr<ros::Publisher> planning_scene_pub_;
        std::shared_ptr<ros::Subscriber> sub_joint_states_, tf_subscriber_;
        robot_model::RobotModelPtr robot_model_;
        std::unique_ptr<ros::ServiceClient> move_baxter_arm_;


        geometry_msgs::PoseStamped transformed_approach_frame_, approach_pose_, goal_pose_, transformed_goal_frame_;

        tf::TransformListener tf_listener_;

        bool goal_aquired_ = false;
        std::vector<double> left_arm_joints_start_positions_,
            right_arm_joints_start_positions_;

        const double kExtentionDistance = 0.05;
        const double  kRetractDistance = 0.2;
        double approach_radius_, x_, y_, z_;
        
    public :

        /**
        @brief Default constructor

        @param .
        @return .
        */
        Primitive();

        /**
        @brief Default destructor

        @param .
        @return .
        */
        ~Primitive() {};

        /**
        @brief Initializes variables.

        @param .
        @return .
        */
        void init();


        /**
        @brief Retrieves parameters from the ros parametres' server,
        	   Initializes publishers and subscribers.

        @param .
        @return .
        */
        void connectToRos();


		 /**
        @brief Transforms the approach pose and goal pose to the robot's base frame.

        @param  * std::string name of the randomly chosen group
				* std::vector<double> the normal to the targeted goal
        @return * .
        */
		void publishTransformToRobotFrame(std::vector<double> goal,
										  std::vector<double> goal_normal);

        /**
        @brief Generates random approach poses to the goal.

        @param .
        @return .
        */
        void defineApproachAndGoalFrames();


        /**
        @brief Generates and approach pose to the goal.

        @param .
        @return * geometry_msgs::PoseStamped the generated pose.
        */
        geometry_msgs::PoseStamped getTargetPose();


        /**
        @brief generate a random approach point around a part of a sphere(between -pi/3 and pi/3 in the goal frame):
        	   where the origin is the goal and the radius is approach radius

        @param * Eigen::Vector3d& outout generated point
        @return .
        */
        virtual void setApproachPoint(Eigen::Vector3d& approach_point);


        /**
        @brief Returns the robot's arms to the starting positions.

        @param * std::string the arm group name.
        @return * bool true if the motion planning and execution have succeeded.
        */
        bool goToStartingPosition(std::string arm_group_name);


        /**
        @brief Reverses back a trajectory

        @param * moveit::planning_interface::MoveGroupInterface::Plan& The plan to be reversed,
        	   * std::string the arm group name,
        	   * std::shared_ptr<moveit::planning_interface::MoveGroupInterface> shared pointer to the arm group.

        @return * bool true if the reseversed motion planning and execution have succeeded..
        */
        bool reverseBackTrajectory(moveit::planning_interface::MoveGroupInterface::Plan& traj_plan);


        /**
        @brief Computes the distance between two std::vectors

        @param * const std::vector<T>& first vector
        	   * const std::vector<T>& second vector
        @return * double: distance between the two vectors.
        */
        template <typename T>
        double  vectorsDistance(const std::vector<T>& a, const std::vector<T>& b)
        {
            std::vector<double> auxiliary;

            std::transform (a.begin(), a.end(), b.begin(), std::back_inserter(auxiliary),//
            [](T element1, T element2) {
                return pow((element1-element2),2);
            });
            auxiliary.shrink_to_fit();

            return  sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0));
        }
         /**
        @brief Computes the distance between two std::vectors

        @param * bool 
        @return .
        */       
		void manipulateOctomap(bool add_octomap_to_acm);
		
		
        /**
        @brief plan and execute the required motion

        @param .

        @return * bool true if the execution has succeeded.
        */
        virtual void execute();
        ros::NodeHandle nh;


		/**
        @brief returns a randomly chosen rotation for the wrist chosen from the wrist_rot_array_

        @param .

        @return * double rotation.
        */						   
		double generateRandomRotationAngle(){
			int random_value = rand()%4;
			// generate random values until we get different value from the previously generated
			while(random_value == rand_rotation_value_index_){
				random_value =rand()%4;
			}
			rand_rotation_value_index_ =random_value;

			return wrist_rot_array_[rand_rotation_value_index_];
		}

		virtual bool planForTrajectory(moveit::planning_interface::MoveGroupInterface::Plan& to_approach_point_plan,
									   moveit::planning_interface::MoveGroupInterface::Plan& to_goal_plan,
									   int iteration);
							    
							    
		virtual bool executePlannedMotion(moveit::planning_interface::MoveGroupInterface::Plan to_approach_point_plan,
										  moveit::planning_interface::MoveGroupInterface::Plan to_goal_plan);

		bool planAndExecute(std::vector<double> goal,	
						    std::vector<double> goal_normal);	
		
		/**
        @brief rotate the wrist 

        @param .

        @return  bool true if the rotation has succeeded.
        */	
		bool rotateWrist(moveit::planning_interface::MoveGroupInterface::Plan wrist_rotation_plan);
};
#endif
