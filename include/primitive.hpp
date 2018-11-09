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
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>


#include <pr2_controllers_msgs/Pr2GripperCommand.h>
class Primitive {

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
        @brief Publishes tf transformations between approach/goal frames and the robot frame.

        @param .
        @return .
        */
        void tfCallback(const tf2_msgs::TFMessageConstPtr& msg);


        /**
        @brief Transforms the approach poses to the robot's base frame.

        @param * std::vector<geometry_msgs::PoseStamped>: input poses
        	   * std::vector<geometry_msgs::PoseStamped>: output poses
        @return * bool true if the transformation is a success.
        */
        bool transformFrames(std::vector<geometry_msgs::PoseStamped> poses_to_transform,
                             std::vector<geometry_msgs::PoseStamped>& output_transform,
                             std::string child_frame_name);


        /**
        @brief Generates random approach poses to the goal.

        @param * int: nbr of approach poses to be generated.
        @return * std::vector<geometry_msgs::PoseStamped>: vector of the generated poses.
        */
        std::vector<geometry_msgs::PoseStamped> getRandomApproachFrames(int number_target);


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
        void getApproachPoint(Eigen::Vector3d& approach_point);


        /**
        @brief Returns the robot's arms to the starting positions.

        @param * std::string the arm group name.
        @return * bool true if the motion planning and execution have succeeded.
        */
        bool goToStartingPosition(std::string arm_group_name);


        /**
        @brief Reverses back a trajectory

        @param * moveit::planning_interface::MoveGroup::Plan& The plan to be reversed,
        	   * std::string the arm group name,
        	   * std::shared_ptr<moveit::planning_interface::MoveGroup> shared pointer to the arm group.

        @return * bool true if the reseversed motion planning and execution have succeeded..
        */
        bool reverseBackTrajectory(moveit::planning_interface::MoveGroup::Plan& traj_plan, std::string arm_group_name,
                                   std::shared_ptr<moveit::planning_interface::MoveGroup> arm_group);


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
        ros::NodeHandle nh;
    private:

        std::vector<double> goal_, goal_normal_;
        XmlRpc::XmlRpcValue planner_params_;
        std::shared_ptr<moveit::planning_interface::MoveGroup> left_arm_group_, right_arm_group_;
        //bool left_arm_in_start_position_,right_arm_in_start_position_;
        std::unique_ptr<ros::AsyncSpinner> asyn_spinner_;
        const double wrist_rot_array_[4] = {0, M_PI/4, M_PI/2, 3*M_PI/4};
        moveit::planning_interface::MoveGroup::Plan _wristRotationPlan;
        std::map<std::string, double> _turn_variable_values;
        int rand_rotation_value_index_ = rand() % 4;
        ros::Publisher  right_gripper_pub_, left_gripper_pub_;
        robot_state::RobotStatePtr robot_state_;

        int right_gripper_id_;
        int left_gripper_id_;
        std::vector<geometry_msgs::PoseStamped> approach_frames_vector_, transformed_approach_frames_vector_,
            goal_frames_vector_, transformed_goal_frames_vector_;

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
};
#endif
