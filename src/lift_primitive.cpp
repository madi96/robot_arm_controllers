/**
  08-11-2018- Dream-Project
  Purpose: Provides primitives to move a robot's arm group ,for the babbling experiment.

  @author Oussama YAAKOUBI
  @version
*/

#include <primitive.hpp>

class LiftPrimitive: public Primitive {

	public:
        // generate a random approach point around a part of a sphere: origin goal, radius: approach radius between -pi/3 and pi/3 in the goal frame
        void setApproachPoint(Eigen::Vector3d& approach_point) {
            approach_point(0) = 0;
            approach_point(1) = 0;
            approach_point(2) = approach_radius_;
        }

// For BAXTER
#ifdef ROBOT_IS_BAXTER
        void calibrateGripper() {
            baxter_core_msgs::EndEffectorCommand commandGripperMsg;

            commandGripperMsg.id = chosen_gripper_id_;
            commandGripperMsg.command = "calibrate";
            chosen_gripper_pub_.publish(commandGripperMsg);
        }
#endif

        void openGripper() {
            ROS_INFO_STREAM("CONTROLLER::Oppening the gripper ");
// For PR2
#ifdef ROBOT_IS_PR2
            pr2_controllers_msgs::Pr2GripperCommand commandGripperMsg;
            commandGripperMsg.position = 0.06;
            commandGripperMsg.max_effort = 100.0;
#endif

// For BAXTER
#ifdef ROBOT_IS_BAXTER
            baxter_core_msgs::EndEffectorCommand commandGripperMsg;
            commandGripperMsg.id = chosen_gripper_id_;
            commandGripperMsg.command = "go";
            commandGripperMsg.args = "{\"position\": 100.0}";
#endif

            chosen_gripper_pub_.publish(commandGripperMsg);
        }


        void closeGripper() {
            ROS_INFO_STREAM("CONTROLLER::Closing the gripper ");
// For PR2
#ifdef ROBOT_IS_PR2
            pr2_controllers_msgs::Pr2GripperCommand commandGripperMsg;
            commandGripperMsg.position = 0.0;
            commandGripperMsg.max_effort = 100.0;
#endif

// For BAXTER
#ifdef ROBOT_IS_BAXTER
            baxter_core_msgs::EndEffectorCommand commandGripperMsg;
            commandGripperMsg.id = chosen_gripper_id_;
            commandGripperMsg.command = "go";
            commandGripperMsg.args = "{\"position\": 40.0}";
            gripper_pub.publish(commandGripperMsg);
#endif

            chosen_gripper_pub_.publish(commandGripperMsg);
        }


        double generateRandomRotationAngle() {
            int random_value = rand()%4;
            // generate random values until we get different value from the previously generated
            while(random_value == rand_rotation_value_index_) {
                random_value =rand()%4;
            }
            rand_rotation_value_index_ =random_value;

            return wrist_rot_array_[rand_rotation_value_index_];
        }


        void setRandomPlacingPose(geometry_msgs::PoseStamped& placing_pose) {
            double placing_radius = std::stod(planner_params_["placing_radius"]);
            double x = placing_radius*(2*rand() / (RAND_MAX + 1.0))-1;
            double y = placing_radius*(2*rand() / (RAND_MAX + 1.0))-1;
            placing_pose.header.frame_id = transformed_approach_frame_.header.frame_id;
            placing_pose.pose.position.x = transformed_approach_frame_.pose.position.x + x;
            placing_pose.pose.position.y = transformed_approach_frame_.pose.position.y + y;
            placing_pose.pose.position.z = transformed_approach_frame_.pose.position.z;
            placing_pose.pose.orientation.w = transformed_approach_frame_.pose.orientation.w;
            placing_pose.pose.orientation.x = transformed_approach_frame_.pose.orientation.x;
            placing_pose.pose.orientation.y = transformed_approach_frame_.pose.orientation.y;
            placing_pose.pose.orientation.z = transformed_approach_frame_.pose.orientation.z;
        }

        bool planForTrajectory(moveit::planning_interface::MoveGroupInterface::Plan& to_approach_point_plan,
                               moveit::planning_interface::MoveGroupInterface::Plan& wrist_rotation_plan,
                               moveit::planning_interface::MoveGroupInterface::Plan& to_goal_plan,
                               moveit::planning_interface::MoveGroupInterface::Plan& to_place_pose_plan,
                               int iteration) {


            bool planning_result = false;
            std::vector< double > arm_group_joints_state;
            geometry_msgs::PoseStamped placing_pose;
            geometry_msgs::Pose gripper_pose;
            double cartesian_path_success_threshold = approach_radius_ /(approach_radius_+ kExtentionDistance) ;
            robot_state::RobotStatePtr start_state_for_goal_trajectory,
                        start_state_for_wrist_rotation, start_state_for_place_trajectory;

            ROS_INFO_STREAM("CONTROLLER:: Planning a trajectory to the selected approach point");
            chosen_arm_group_->setStartState(*chosen_arm_group_->getCurrentState());
            chosen_arm_group_->setPoseTarget(transformed_approach_frame_);
            bool to_approach_point_plan_result = static_cast<bool>(chosen_arm_group_->plan(to_approach_point_plan));


            // plan the wrist rotation
            if (to_approach_point_plan_result) {
                ROS_INFO_STREAM("CONTROLLER:: Planning a random wrist rotation");
                start_state_for_wrist_rotation = chosen_arm_group_->getCurrentState();

                // set the starting position for the wrist rotation
                moveit::core::jointTrajPointToRobotState(to_approach_point_plan.trajectory_.joint_trajectory,
                        to_approach_point_plan.trajectory_.joint_trajectory.points.size() - 1,
                        *start_state_for_wrist_rotation);

                chosen_arm_group_->setStartState(*start_state_for_wrist_rotation);
                start_state_for_wrist_rotation->copyJointGroupPositions(chosen_arm_group_name_, arm_group_joints_state);

                // set a random rotation for the wrist joint and plan the rotation
                arm_group_joints_state[arm_group_joints_state.size()-1] = generateRandomRotationAngle();
                chosen_arm_group_->setJointValueTarget(arm_group_joints_state);
                bool wrist_rotation_plan_result =  static_cast<bool>(chosen_arm_group_->plan(wrist_rotation_plan));

                // plan the final part of the trajectory: from the approach point to the goal
                if (wrist_rotation_plan_result) {

                    start_state_for_goal_trajectory = chosen_arm_group_->getCurrentState();
                    ROS_INFO_STREAM("CONTROLLER:: Planning a trajectory to the specified goal");
                    moveit::core::jointTrajPointToRobotState(wrist_rotation_plan.trajectory_.joint_trajectory,
                            wrist_rotation_plan.trajectory_.joint_trajectory.points.size() - 1,
                            *start_state_for_goal_trajectory);

                    // get the gripper's link pose after the wrist rotation
                    tf::poseEigenToMsg(start_state_for_goal_trajectory->getGlobalLinkTransform(chosen_gripper_link_name_), gripper_pose);

                    // change the orientation of the selected frame to match the orientation of the gripper after rotation
                    transformed_approach_frame_.pose.orientation.x= gripper_pose.orientation.x;
                    transformed_approach_frame_.pose.orientation.y= gripper_pose.orientation.y;
                    transformed_approach_frame_.pose.orientation.z= gripper_pose.orientation.z;
                    transformed_approach_frame_.pose.orientation.w= gripper_pose.orientation.w;

                    transformed_goal_frame_.pose.orientation.x = gripper_pose.orientation.x;
                    transformed_goal_frame_.pose.orientation.y = gripper_pose.orientation.y;
                    transformed_goal_frame_.pose.orientation.z = gripper_pose.orientation.z;
                    transformed_goal_frame_.pose.orientation.w = gripper_pose.orientation.w;

                    //make sure that the octomap is in the collision world
                    manipulateOctomap(true);

                    std::vector<geometry_msgs::Pose> waypoints;
                    waypoints.push_back(transformed_approach_frame_.pose);
                    waypoints.push_back(transformed_goal_frame_.pose);


                    moveit_msgs::RobotTrajectory robot_trajectory;
                    chosen_arm_group_->setStartState(*start_state_for_goal_trajectory);
                    double fraction = chosen_arm_group_->computeCartesianPath(waypoints, 0.01, 0.0, robot_trajectory);

                    // compute the success threshold: a computed cartesian path is considered a success
                    //if it return a faction greater than this threshold

                    if(fraction >= cartesian_path_success_threshold) {
                        to_goal_plan.trajectory_ = robot_trajectory;
                        chosen_arm_group_->setStartState(*start_state_for_wrist_rotation);
                        setRandomPlacingPose(placing_pose);
                        chosen_arm_group_->setPoseTarget(placing_pose);
                        bool to_place_pose_plan_result = static_cast<bool>(chosen_arm_group_->plan(to_place_pose_plan));
                        if (to_place_pose_plan_result) {
                            planning_result = true;
                        } else {
                            ROS_WARN_STREAM("CONTROLLER:: Failed to find plan to the randomly selected placing pose "<< " for the "<<chosen_arm_group_name_);
                        }
                    } else {
                        ROS_WARN_STREAM("CONTROLLER:: Failed to find a straight line motion from the approach point to the goal "<< " for the "<<chosen_arm_group_name_);
                    }
                } else {
                    ROS_WARN_STREAM("CONTROLLER:: Failed to plan the wrist rotation"<< " for the "<<chosen_arm_group_name_);
                }

            } else {
                ROS_WARN_STREAM("CONTROLLER:: Failed to find a plan from the starting position to the approach point"<< " for the "<<chosen_arm_group_name_);
            }
            return planning_result;
        }


        bool executePlannedMotion(moveit::planning_interface::MoveGroupInterface::Plan to_approach_point_plan,
                                  moveit::planning_interface::MoveGroupInterface::Plan wrist_rotation_plan,
                                  moveit::planning_interface::MoveGroupInterface::Plan to_goal_plan,
                                  moveit::planning_interface::MoveGroupInterface::Plan to_place_pose_plan) {

		
            ROS_INFO_STREAM("CONTROLLER:: Executing planned motion");
            bool execution_result = false;
            // Execute plane: from starting position to the approach position
            bool to_approach_point_motion_result = static_cast<bool>(chosen_arm_group_->execute(to_approach_point_plan));
            ros::Duration(1).sleep();
            // random wrist rotation
            bool wrist_rotation_result = rotateWrist(wrist_rotation_plan);
            ros::Duration(1).sleep();
            // open gripper
            openGripper();
            ros::Duration(1).sleep();
            // Execute plane: from approach position to the specified goal
            //~ moveit::core::robotStateToRobotStateMsg(*chosen_arm_group_->getCurrentState(), to_goal_plan.start_state_);
            //chosen_arm_group_->setStartState(*chosen_arm_group_->getCurrentState());
            bool to_goal_motion_result = static_cast<bool>(chosen_arm_group_->execute(to_goal_plan));
            // close gripper
            closeGripper();
            ros::Duration(1).sleep();
            // reverse back motion from approach position to the specified goal
            reverseBackTrajectory(to_goal_plan);
            // reverse back wrist rotation
            reverseBackTrajectory(wrist_rotation_plan);
            //go to placing position
            bool to_placing_pose_motion_result = static_cast<bool>(chosen_arm_group_->execute(to_place_pose_plan));
            ros::Duration(1).sleep();
            openGripper();
            //make sure that the octomap is not in the collision world
            manipulateOctomap(false);
            goToStartingPosition(chosen_arm_group_name_);
            //make sure that the octomap is not in the collision world
            manipulateOctomap(true);

            if (to_approach_point_motion_result && to_goal_motion_result && wrist_rotation_result && to_placing_pose_motion_result) {
                ROS_INFO_STREAM("CONTROLLER:: Planned motion executed successfully");
                execution_result = true;
            } else {
                ROS_WARN_STREAM("CONTROLLER:: Planned motion execution FAILED");
            }
            return execution_result;
        }

        bool planAndExecute(std::vector<double> goal,
                            std::vector<double> goal_normal) {

            moveit::planning_interface::MoveGroupInterface::Plan to_approach_point_plan,
                   wrist_rotation_plan,to_goal_plan, to_place_pose_plan;
            bool planning_result = false;
            bool execution_result = false;
            int  random_frame_budget = 10;
            int random_frame_count = 0;
            while(!planning_result && random_frame_count < random_frame_budget ) {
                publishTransformToRobotFrame(goal, goal_normal);
                planning_result = planForTrajectory(to_approach_point_plan, wrist_rotation_plan, to_goal_plan, to_place_pose_plan, random_frame_count);
                random_frame_count +=1;
            }
            if (planning_result) {
                execution_result = executePlannedMotion(to_approach_point_plan, wrist_rotation_plan, to_goal_plan, to_place_pose_plan);
                if (execution_result) {
                    ROS_INFO_STREAM("CONTROLLER:: Planned motion executed successfully"<< " for the "<<chosen_arm_group_name_);
                    //~ action_serv_->setSucceeded();
                } else {
                    ROS_INFO_STREAM("CONTROLLER:: Planned motion execution FAILED"<< " for the "<<chosen_arm_group_name_);
                    //~ action_serv_->setAborted();
                }
            }
            return planning_result;
        }


        void execute() {

            std::vector<double> goal, goal_normal;
            ros::Publisher gripper_pub;
            bool result = false;

            //make sure that the octomap is in the collision world
            manipulateOctomap(true);

            // put left and right arms in the starting positions
            goToStartingPosition("left");
            goToStartingPosition("right");

            // set the goal
            goal = {x_,y_,z_};
            goal_normal = {0,0,0};


            //always plan for the approach with the octomap in the obstacle domain
            manipulateOctomap(false);

            // Choose a random arm group
            chosen_arm_group_name_ = group_names_array_[rand()%2];

            ROS_INFO_STREAM("CONTROLLER:: chosen group name "<<chosen_arm_group_name_);

            // set the targeted arm joints group and its starting joints values
            if(chosen_arm_group_name_ == "left_arm") {
                ROS_INFO_STREAM("CONTROLLER:: chosen group name left_arm ");
                chosen_arm_group_ = left_arm_group_;
                chosen_gripper_pub_ = left_gripper_pub_;
                chosen_gripper_link_name_ = left_gripper_link_name_;
// For BAXTER
#ifdef ROBOT_IS_BAXTER
                chosen_gripper_id_ = left_gripper_id_;
#endif
            } else {
                ROS_INFO_STREAM("CONTROLLER:: chosen group name right_arm ");
                chosen_arm_group_ = right_arm_group_;
                chosen_gripper_pub_ = right_gripper_pub_;
                chosen_gripper_link_name_ = right_gripper_link_name_;
// For BAXTER
#ifdef ROBOT_IS_BAXTER
                chosen_gripper_id_ = right_gripper_id_;
#endif
            }

            // plan and execute for the the chosen arm
            result= planAndExecute(goal, goal_normal);
            if (! result) { // plan for the other arm
                ROS_INFO_STREAM("CONTROLLER:: planning for the other group, current group_name "<<chosen_arm_group_name_);
                if (chosen_arm_group_name_ == "left_arm") {
                    ROS_INFO_STREAM("CONTROLLER:: second group name right_arm ");
                    chosen_arm_group_name_ = "right_arm";
                    chosen_arm_group_ = right_arm_group_;
                    chosen_gripper_pub_ = right_gripper_pub_;
                    chosen_gripper_link_name_ = right_gripper_link_name_;
// For BAXTER
#ifdef ROBOT_IS_BAXTER
                    chosen_gripper_id_ = right_gripper_id_;
#endif
                } else if (chosen_arm_group_name_ == "right_arm") {
                    ROS_INFO_STREAM("CONTROLLER:: second group name left_arm ");
                    chosen_arm_group_name_ = "left_arm";
                    chosen_arm_group_ = left_arm_group_;
                    chosen_gripper_pub_ = left_gripper_pub_;
                    chosen_gripper_link_name_ = left_gripper_link_name_;
// For BAXTER
#ifdef ROBOT_IS_BAXTER
                    chosen_gripper_id_ = left_gripper_id_;
#endif
                }
                // plan and execute for the other arm if the chosen arm fails
                result= planAndExecute(goal, goal_normal);
                if (! result) {
                    //~ action_serv_->setAborted();
                    ROS_WARN_STREAM("CONTROLLER:: Failed to plan for the second arm: "<<chosen_arm_group_name_);
                }
            }
            ROS_WARN_STREAM("CONTROLLER:: ------------------------------");
        }
    private:
        ros::Publisher  chosen_gripper_pub_;
        std::string chosen_gripper_link_name_;
        // For BAXTER
#ifdef ROBOT_IS_BAXTER
        int chosen_gripper_id_;
        static constexpr int left_gripper_id_ = 131073;
        static constexpr int right_gripper_id_ = 131073;
#endif
};

std::string parse_arg(int& argc, char **& argv, const std::string& default_val)
{
    std::string key;
    std::string value;
    std::string temp_str;
    std::string::size_type res;

    key = "__name:=";
    for (unsigned short i = 0; i < argc; ++i) {
        temp_str = argv[i];
        res = temp_str.find(key);

        if (res != std::string::npos) {
            value = temp_str.erase(res, key.length());
            break;
        }
        else if (i == argc - 1) {
            value = default_val;
        }
    }
    return value;
}

int main(int argc, char** argv) {
    std::string node_name;
    node_name = parse_arg(argc, argv, "lift_primitive_node");
    ros::init(argc, argv, node_name);

    LiftPrimitive controller;

    ROS_INFO_STREAM("CONTROLLER:: NameSpace: "<<controller.nh.getNamespace());

    ROS_INFO_STREAM("CONTROLLER:: Robot controller ready !");
    while (ros::ok()) {
        controller.execute();
        usleep(1000);
        ros::spinOnce();
    }
    return 0;
}
